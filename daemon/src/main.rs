//! ESB daemon: hosts the virtual-disk data and drives the ESP32-S3 over BLE.
//!
//! This is the BLE GATT *server* (peripheral) in the inverted protocol: the
//! ESP32-S3 is the central that *pulls* sectors from us. When the USB host reads
//! an MSC sector, the ESP32 writes the LBA to [`LBA_UUID`] and then reads
//! [`SECTOR_UUID`], which we fill from the backing file. The ESP32 also polls
//! [`USB_ENABLE_UUID`] to decide whether to soft-disconnect from the USB host.
//!
//! The backing data lives in a regular file on this host. Reads happen inside
//! the (synchronous) GATT read handler, so they are blocking `File::read_at`
//! calls — fine for 512-byte sectors.
//!
//! Plug/unplug is driven at runtime by signals or the control IPC:
//!
//! - `SIGUSR1` -> USB attached  (`usb_enable = 1`)
//! - `SIGUSR2` -> USB detached   (`usb_enable = 0`)
//!
//! The initial state is set with `--no-usb`.
//!
//! A Unix-domain control socket (see [`control`]) lets the bundled CLI attach /
//! detach the device and list or copy files in the FAT32 image while the daemon
//! runs. All filesystem access is in userland; nothing is kernel-mounted.

mod control;
mod disk;
mod fat;
mod ipc;
mod protocol;

// Register trouble-host's `security` crypto backend: a std-backed global CSPRNG
// and the AES-128 / P-256 drivers required for LE Secure Connections. The
// side-effectful `use` links each crate's `#[ctor]`/distributed-slice
// registrations into the binary (mirroring the trouble-host Linux examples).
use {embassy_crypto_rand as _, embassy_crypto_rustcrypto as _};

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use embassy_futures::join::join;
use log::{info, warn};
use trouble_host::prelude::*;

use crate::disk::Disk;
use crate::ipc::Request;
use crate::protocol::{
    ProtocolState, DAEMON_ADDR, LBA_UUID, SECTOR_SIZE, SECTOR_UUID, SERVICE_UUID, USB_ENABLE_UUID,
};

/// Max number of connections. Only the ESP32-S3 connects.
const CONNECTIONS_MAX: usize = 1;
/// Max L2CAP channels: L2CAP signaling + ATT (+ headroom).
const L2CAP_CHANNELS_MAX: usize = 3;

/// BLE advertisement / GAP name for this peripheral.
const ADV_NAME: &str = "ESB-Daemon";

// ---------------------------------------------------------------------------
// GATT server definition
// ---------------------------------------------------------------------------

#[gatt_server]
struct Server {
    esb: EsbService,
}

/// The single ESB control + storage service. All three characteristics are
/// *dynamic* (typed `()`) because their values are computed on demand in
/// [`gatt_events_task`]:
///
/// - `usb_enable`: served on read as a single byte (1 = attached, 0 = detached).
/// - `lba`: written by the central to select the next sector to serve.
/// - `sector`: served on read as the 512-byte sector at the last-written LBA.
///
/// `accept_unprocessed(&value)` lets the server emit any value (and auto-chunks
/// the 512-byte sector across Read Blob PDU offsets) regardless of the stored
/// `()` type.
#[gatt_service(uuid = SERVICE_UUID)]
struct EsbService {
    #[characteristic(uuid = USB_ENABLE_UUID, read)]
    usb_enable: (),
    #[characteristic(uuid = LBA_UUID, write)]
    lba: (),
    #[characteristic(uuid = SECTOR_UUID, read)]
    sector: (),
}

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

/// Default backing-image filename.
const DEFAULT_IMAGE: &str = "esb.img";
/// Default `init` size in MiB (large enough for FAT32).
const DEFAULT_IMAGE_SIZE_MIB: u64 = 128;

#[derive(Parser, Debug)]
#[command(
    name = "esb-daemon",
    about = "ESB BLE GATT server hosting a virtual USB disk for an ESP32-S3"
)]
struct Args {
    /// HCI device number to bind to (`0` for the default adapter).
    #[arg(long, default_value_t = 0, value_name = "DEV", global = true)]
    device: u16,

    /// Backing disk image file. Created sparse (128 MiB) if it does not exist.
    #[arg(long, default_value = DEFAULT_IMAGE, value_name = "PATH", global = true)]
    image: PathBuf,

    /// Control IPC socket path. Defaults to `$XDG_RUNTIME_DIR/esb-daemon.sock`
    /// or a per-UID path under the system temp directory.
    #[arg(long, value_name = "PATH", global = true)]
    socket: Option<PathBuf>,

    /// Start with the USB device soft-disconnected (the host sees it unplugged).
    #[arg(long, global = true)]
    no_usb: bool,

    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Run the BLE daemon (the default when no subcommand is given).
    Serve,
    /// Create a fresh FAT32 image in userland (no kernel mount).
    Init {
        /// Image file to initialize (defaults to `--image`).
        path: Option<PathBuf>,
        /// Image size in MiB (must be at least 64 MiB for FAT32).
        #[arg(long, default_value_t = DEFAULT_IMAGE_SIZE_MIB, value_name = "MIB")]
        size_mib: u64,
        /// Overwrite the file if it already exists.
        #[arg(long)]
        force: bool,
    },
    /// Attach (plug in) the virtual USB device.
    Attach,
    /// Detach (unplug) the virtual USB device.
    Detach,
    /// Show USB state and image/filesystem summary.
    Status,
    /// List a directory inside the FAT32 image.
    Ls {
        #[arg(default_value = "/", value_name = "PATH")]
        path: String,
    },
    /// Copy a file out of the image: `get <IMAGE_PATH> <HOST_PATH>`.
    Get {
        image_path: String,
        host_path: PathBuf,
    },
    /// Copy a file into the image: `put <HOST_PATH> <IMAGE_PATH>`.
    Put {
        host_path: PathBuf,
        image_path: String,
    },
}

// ---------------------------------------------------------------------------
// Entry
// ---------------------------------------------------------------------------

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    env_logger::init();
    let Args {
        device,
        image,
        socket,
        no_usb,
        command,
    } = Args::parse();
    let socket = socket.unwrap_or_else(ipc::default_socket_path);

    match command.unwrap_or(Command::Serve) {
        Command::Init {
            path,
            size_mib,
            force,
        } => {
            let target = path.unwrap_or(image);
            let size = size_mib
                .checked_mul(1024 * 1024)
                .context("image size is too large")?;
            let info = fat::init(&target, size, force)?;
            println!(
                "initialized FAT32 image {} ({} bytes, cluster {}, {} free)",
                target.display(),
                size,
                info.cluster_size,
                info.free_bytes
            );
        }
        Command::Attach => ipc::print_response(ipc::request(&socket, &Request::Attach)?)?,
        Command::Detach => ipc::print_response(ipc::request(&socket, &Request::Detach)?)?,
        Command::Status => ipc::print_response(ipc::request(&socket, &Request::Status)?)?,
        Command::Ls { path } => ipc::print_response(ipc::request(&socket, &Request::Ls { path })?)?,
        Command::Get {
            image_path,
            host_path,
        } => ipc::print_response(ipc::request(
            &socket,
            &Request::Get {
                image_path,
                host_path,
            },
        )?)?,
        Command::Put {
            host_path,
            image_path,
        } => ipc::print_response(ipc::request(
            &socket,
            &Request::Put {
                host_path,
                image_path,
            },
        )?)?,
        Command::Serve => {
            let disk = Disk::open_or_create(&image)
                .with_context(|| format!("opening backing image {image:?}"))?;
            info!(
                "backing image {:?} ready ({} bytes per sector, {} sectors, {} bytes)",
                image,
                SECTOR_SIZE,
                crate::protocol::BLOCK_COUNT,
                disk.size()
            );

            let state = ProtocolState::new();
            if no_usb {
                state.set_usb_enable(false);
            }
            // Share the control state with the GATT event task (reads/writes),
            // the signal task and the IPC server (plug/unplug toggles) via `Arc`.
            let state = Arc::new(state);

            control::spawn(&socket, image, Arc::clone(&state), disk.clone())
                .with_context(|| format!("starting control IPC on {socket:?}"))?;

            let transport = bt_hci_linux::Transport::new(device)
                .with_context(|| format!("opening HCI device {device}"))?;
            let controller = ExternalController::<_, 8>::new(transport);

            info!(
                "HCI device {} ready; advertising as {:?}",
                device, DAEMON_ADDR
            );
            run(controller, state, disk).await?;
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// BLE stack + GATT loop
// ---------------------------------------------------------------------------

async fn run<C: Controller>(controller: C, state: Arc<ProtocolState>, disk: Disk) -> Result<()> {
    let address = Address::random(DAEMON_ADDR);
    info!("our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();
    let runner = stack.runner();
    let mut peripheral = stack.peripheral();

    // `new_with_config` returns a plain `&'static str` error (not an `Error`),
    // so `anyhow::Context` is unavailable here; map it into an `anyhow::Error`.
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: ADV_NAME,
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .map_err(|e| anyhow::anyhow!("building GATT server: {e}"))?;

    // Background task: toggle plug/unplug on SIGUSR1 / SIGUSR2.
    tokio::spawn(signal_toggle_task(Arc::clone(&state)));

    info!("advertising; waiting for the ESP32-S3 central to connect");
    // `runner` runs forever in parallel with the advertise/connect loop, which
    // re-advertises on every disconnect. `join` awaits both (i.e. forever).
    let _ = join(ble_task(runner), async {
        loop {
            match advertise(&mut peripheral, &server).await {
                Ok(conn) => {
                    info!("[adv] connection established");
                    if let Err(e) = gatt_events_task(&server, &conn, &state, &disk).await {
                        warn!("[gatt] event task ended: {:?}", e);
                    }
                    info!("[adv] back to advertising");
                }
                Err(e) => warn!("[adv] error: {:?}", e),
            }
        }
    })
    .await;
    Ok(())
}

/// The required background BLE-runner task; must run for the stack's lifetime.
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            warn!("[ble_task] error: {:?}", e);
        }
    }
}

/// Handle GATT read/write requests for the connected central until disconnect.
///
/// - Read `usb_enable`: serve the current `usb_enable` byte.
/// - Write `lba`: parse the 4-byte little-endian LBA and store it.
/// - Read `sector`: read the backing disk at the last-written LBA and serve the
///   full 512-byte sector (the server auto-chunks it across Read Blob offsets).
async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    state: &ProtocolState,
    disk: &Disk,
) -> Result<(), Error> {
    let usb_enable_handle = server.esb.usb_enable.handle;
    let lba_handle = server.esb.lba.handle;
    let sector_handle = server.esb.sector.handle;

    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                let reply = match event {
                    GattEvent::Read(event) => {
                        if event.handle() == usb_enable_handle {
                            let v = state.usb_enable();
                            event.accept_unprocessed(&v)
                        } else if event.handle() == sector_handle {
                            let lba = state.lba();
                            let mut buf = [0u8; SECTOR_SIZE];
                            if let Err(e) = disk.read_sector(lba, &mut buf) {
                                warn!(
                                    "[gatt] sector read @ {} failed: {:?} (serving zeros)",
                                    lba, e
                                );
                            }
                            event.accept_unprocessed(&buf)
                        } else {
                            event.accept()
                        }
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == lba_handle {
                            match event.validate(4, 4) {
                                Ok(()) => {
                                    event.with_data(|offset, data| {
                                        if offset == 0 && data.len() == 4 {
                                            let lba = u32::from_le_bytes([
                                                data[0], data[1], data[2], data[3],
                                            ]);
                                            state.set_lba(lba);
                                            info!("[gatt] LBA set to {}", lba);
                                        }
                                    });
                                    event.accept_unprocessed()
                                }
                                Err(err) => event.reject(err),
                            }
                        } else {
                            event.accept()
                        }
                    }
                    _ => event.accept(),
                };
                // Send the reply *without blocking* the GATT event loop.
                //
                // The outbound L2CAP TX queue is small (8 PDUs, `L2CAP_TX_QUEUE_SIZE`)
                // and is drained by the BLE runner, which is gated on ACL credits.
                // Awaiting `reply.send()` here parks this task for as long as the
                // queue stays full. Normally the runner drains it within a few
                // connection events. But if the central disconnects *while the
                // queue is full*, the runner can no longer drain it (the link is
                // gone), so the `send()` future never completes. The loop is then
                // parked forever at this `await` and can never reach
                // `conn.next().await` below to observe `Disconnected` — a deadlock
                // between "can't see the disconnect" and "the send can't finish
                // because the connection is already gone". That is the permanent
                // wedge: the log freezes, no `Disconnected` is logged, and the
                // process is stuck until killed.
                //
                // `try_send()` never blocks: on a full queue it simply drops the
                // reply. The central's read then times out and it reconnects.
                // Because this loop is never parked, it always advances to
                // `conn.next().await` and a disconnect is *always* observed, so
                // the daemon re-advertises and recovers. We trade a rare,
                // dropped reply (reconnect churn) for no permanent wedge. This
                // is the same non-blocking behaviour `Reply::drop` already applies
                // on the implicit path — just made explicit.
                match reply {
                    Ok(reply) => {
                        if let Err(e) = reply.try_send() {
                            warn!("[gatt] outbound queue full; dropped reply: {:?}", e);
                        }
                    }
                    Err(e) => warn!("[gatt] building response failed: {:?}", e),
                };
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    info!("[gatt] disconnected: {:?}", reason);
    Ok(())
}

/// Advertise connectably and wait for a connection, returning the GATT
/// connection bound to this server.
///
/// The single `'values` lifetime ties together the data the `Peripheral` and
/// `Server` borrow; the returned `GattConnection` shares it (matching the
/// trouble-host Linux examples).
async fn advertise<'values, 'server, C: Controller>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0u8; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(ADV_NAME.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

// ---------------------------------------------------------------------------
// Plug/unplug control
// ---------------------------------------------------------------------------

/// Toggle the USB-enable flag on SIGUSR1 (attach) / SIGUSR2 (detach).
async fn signal_toggle_task(state: Arc<ProtocolState>) {
    let mut usr1 = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::user_defined1())
        .expect("install SIGUSR1 handler");
    let mut usr2 = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::user_defined2())
        .expect("install SIGUSR2 handler");
    info!(
        "[signal] SIGUSR1=attach, SIGUSR2=detach (current usb_enable={})",
        state.usb_enable()
    );
    loop {
        tokio::select! {
            _ = usr1.recv() => {
                state.set_usb_enable(true);
                info!("[signal] SIGUSR1 -> USB attached");
            }
            _ = usr2.recv() => {
                state.set_usb_enable(false);
                info!("[signal] SIGUSR2 -> USB detached");
            }
        }
    }
}
