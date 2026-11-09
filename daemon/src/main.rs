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
//! Plug/unplug is driven at runtime by signals:
//!
//! - `SIGUSR1` -> USB attached  (`usb_enable = 1`)
//! - `SIGUSR2` -> USB detached   (`usb_enable = 0`)
//!
//! The initial state is set with `--no-usb`.

mod disk;
mod protocol;

// Register trouble-host's `security` crypto backend: a std-backed global CSPRNG
// and the AES-128 / P-256 drivers required for LE Secure Connections. The
// side-effectful `use` links each crate's `#[ctor]`/distributed-slice
// registrations into the binary (mirroring the trouble-host Linux examples).
use {embassy_crypto_rand as _, embassy_crypto_rustcrypto as _};

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, Result};
use clap::Parser;
use embassy_futures::join::join;
use log::{info, warn};
use trouble_host::prelude::*;

use crate::disk::Disk;
use crate::protocol::{
    DAEMON_ADDR, LBA_UUID, ProtocolState, SECTOR_SIZE, SECTOR_UUID, SERVICE_UUID, USB_ENABLE_UUID,
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

#[derive(Parser, Debug)]
#[command(name = "esb-daemon", about = "ESB BLE GATT server hosting a virtual USB disk for an ESP32-S3")]
struct Args {
    /// HCI device number to bind to (`0` for the default adapter).
    #[arg(long, default_value_t = 0, value_name = "DEV")]
    device: u16,

    /// Backing disk image file. Created sparse if it does not exist.
    #[arg(long, default_value = "esb.img", value_name = "PATH")]
    image: PathBuf,

    /// Start with the USB device soft-disconnected (the host sees it unplugged).
    #[arg(long)]
    no_usb: bool,
}

// ---------------------------------------------------------------------------
// Entry
// ---------------------------------------------------------------------------

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    env_logger::init();
    let args = Args::parse();

    let disk = Disk::open_or_create(&args.image)
        .with_context(|| format!("opening backing image {:?}", args.image))?;
    info!(
        "backing image {:?} ready ({} bytes per sector, {} sectors)",
        args.image,
        SECTOR_SIZE,
        crate::protocol::BLOCK_COUNT
    );

    let state = ProtocolState::new();
    if args.no_usb {
        state.set_usb_enable(false);
    }
    // Share the control state with the GATT event task (reads/writes) and the
    // signal task (plug/unplug toggles) via `Arc`.
    let state = Arc::new(state);

    let transport = bt_hci_linux::Transport::new(args.device)
        .with_context(|| format!("opening HCI device {}", args.device))?;
    let controller = ExternalController::<_, 8>::new(transport);

    info!("HCI device {} ready; advertising as {:?}", args.device, DAEMON_ADDR);
    run(controller, state, disk).await
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
                                warn!("[gatt] sector read @ {} failed: {:?} (serving zeros)", lba, e);
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
                                            let lba = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
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
                // Send the reply explicitly so it is not left to Drop.
                match reply {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] error sending response: {:?}", e),
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
    info!("[signal] SIGUSR1=attach, SIGUSR2=detach (current usb_enable={})", state.usb_enable());
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
