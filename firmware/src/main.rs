//! ESB firmware: an ESP32-S3 presented to the USB host as a composite device
//! (HID keyboard + MSC disk) whose plug/unplug state and disk contents are
//! driven remotely over BLE.
//!
//! Role inversion: the Linux daemon is the BLE GATT *server* (it hosts the disk
//! data), so the ESP32 is the GATT *central*. When the USB host reads an MSC
//! sector, [`BleBlockDevice`] pulls it over BLE (write the LBA, read the sector).
//! A separate loop polls the daemon's `usb_enable` flag and toggles the USB OTG
//! soft-disconnect bit (`DCTL.SDIS`) so the host sees the device appear/vanish.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::{
    Builder,
    Config as UsbConfig,
    class::hid::{
        HidBootProtocol, HidSubclass, KeyboardReport,
        device::{Config as HidConfig, HidWriter, State as HidState},
    },
    class::msc::device::{Config as MscConfig, MscClass, State as MscState},
};
use embassy_usb_synopsys_otg::otg_v1::Otg;
use esp_hal::{
    clock::CpuClock,
    timer::timg::TimerGroup,
    usb::otg::{
        Usb,
        embassy_usb_device::{Config as DriverConfig, Driver},
    },
};
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

use crate::protocol::{
    LBA_UUID, MAX_SERVICES, SECTOR_SIZE, SECTOR_UUID, SERVICE_UUID, USB_ENABLE_UUID,
};
use crate::storage::BleBlockDevice;
use crate::ws2812::Ws2812;

mod csprng;
mod protocol;
mod storage;
mod ws2812;

esp_bootloader_esp_idf::esp_app_desc!();
use {embassy_crypto_rustcrypto as _, esp_alloc as _, esp_backtrace as _};

/// Fixed random BLE identity the daemon advertises under. The daemon must use
/// exactly this address so the central's accept list matches it.
const DAEMON_ADDR: [u8; 6] = [0x81, 0x90, 0xa1, 0xb2, 0xc3, 0x0e];
/// Our own (arbitrary) BLE identity as the central.
const OUR_ADDR: [u8; 6] = [0x02, 0x02, 0x02, 0x02, 0x02, 0x02];
/// USB_FS base address, for the soft-disconnect (`DCTL.SDIS`) register handle.
const USB_FS_BASE: usize = 0x6008_0000;
/// HID usage code for 'A' — used by the periodic heartbeat keystroke.
const KEY_A: u8 = 0x04;

// ---- Status LED shared state (poll-driven, no locks) ----
//
// The LED task (a separate executor task) samples these atomics on a timer and
// derives the pixel color. Writers just store; the task reads. Three signals:
//   CONNECTED  – we are paired (a GATT connection to the daemon is up).
//   ATTACHED   – the USB OTG soft-disconnect bit is cleared (host sees the key).
//   ACCESS     – latched by the MSC read path, consumed (cleared) by the LED
//                task; a raw 64-bit timestamp can't live in a static because
//                this target has no `AtomicU64`.
static CONNECTED: AtomicBool = AtomicBool::new(false);
static ATTACHED: AtomicBool = AtomicBool::new(true);
static ACCESS: AtomicBool = AtomicBool::new(false);

/// Steady-state pixel colors (R, G, B) at a comfortable brightness.
const LED_ORANGE: (u8, u8, u8) = (200, 96, 0); // unpaired (no daemon connection)
const LED_BLUE: (u8, u8, u8) = (0, 0, 200); // paired (connected to the daemon)
const LED_GREEN: (u8, u8, u8) = (0, 180, 0); // plugged (attached to the host)
const LED_OFF: (u8, u8, u8) = (0, 0, 0);

/// Half-period of the "accessed" green blink (on for this, off for this).
const LED_BLINK_MS: u64 = 250;
/// How long after a disk access the green keeps blinking.
const LED_ACCESS_WINDOW: Duration = Duration::from_millis(1_000);

/// Record that the disk was just read (called from the MSC read path).
///
/// Latches a flag the LED task consumes; it does not need to be precise, only
/// "a read happened since we last looked".
pub(crate) fn mark_accessed() {
    ACCESS.store(true, Ordering::Release);
}

/// Drive the on-board WS2812 from the shared state.
///
/// Priority (most to least specific): unpaired → orange; recent disk access →
/// green blink; attached → green; otherwise (paired, not attached) → blue.
///
/// This task owns the (non-`Send`) RMT channel, so it must be spawned with the
/// plain `Spawner` from [`#[esp_rtos::main]`], never with a `SendSpawner`.
#[embassy_executor::task]
async fn led_task(mut led: Ws2812) {
    let mut on = true;
    let mut access_until: Option<Instant> = None;
    loop {
        // A fresh disk read latches ACCESS; when we see it, extend the window.
        // `swap` is the atomic consume: we take the flag and clear it in one
        // step, so a read that lands while we're computing still latches for
        // the next pass.
        if ACCESS.swap(false, Ordering::AcqRel) {
            access_until = Some(Instant::now() + LED_ACCESS_WINDOW);
        }
        // `Some` while the window is open, `None` once it has expired.
        let accessed = match access_until {
            Some(u) => match u.checked_duration_since(Instant::now()) {
                Some(_) => true,
                None => {
                    access_until = None;
                    false
                }
            },
            None => false,
        };

        let connected = CONNECTED.load(Ordering::Acquire);
        let attached = ATTACHED.load(Ordering::Acquire);
        let (r, g, b) = if !connected {
            LED_ORANGE
        } else if accessed {
            if on { LED_GREEN } else { LED_OFF }
        } else if attached {
            LED_GREEN
        } else {
            LED_BLUE
        };
        let _ = led.set_rgb(r, g, b).await;
        Timer::after_millis(LED_BLINK_MS).await;
        on = !on;
    }
}

#[esp_rtos::main]
async fn main(s: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    // Seed the CSPRNG (required by trouble-host `security`).
    let mut seed = [0u8; 32];
    esp_hal::rng::Rng::new().read(&mut seed);
    csprng::seed(seed);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // ---- BLE controller + stack (ESP32 is the central) ----
    let connector = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    let mut resources: HostResources<DefaultPacketPool, 1, 3> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random(OUR_ADDR))
        .build();
    let mut runner = stack.runner();
    let mut central = stack.central();

    // ---- USB composite device (HID keyboard + MSC disk) ----
    let usb = Usb::new_fs(peripherals.USB_FS, peripherals.GPIO20, peripherals.GPIO19);
    let mut ep_out_buffer = [0u8; 1024];
    let driver = Driver::new(usb, &mut ep_out_buffer, DriverConfig::default());

    let mut usb_config = UsbConfig::new(0x1234, 0x5678);
    usb_config.manufacturer = Some("ESB");
    usb_config.product = Some("ESB Remote USB");
    usb_config.serial_number = Some("esb0000000001");

    let mut config_descriptor = [0u8; 256];
    let mut bos_descriptor = [0u8; 256];
    let mut control_buf = [0u8; 64];

    // Class states must outlive the Builder: its destructor drops the registered
    // classes, which borrow these states. Declaring them first guarantees they are
    // dropped *after* the builder (drops run in reverse declaration order).
    let mut hid_state = HidState::new();
    let mut msc_state = MscState::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no MSOS descriptors
        &mut control_buf,
    );

    // Function 1: HID keyboard (write-only, 8-byte boot report).
    let hid_config = HidConfig {
        report_descriptor: KeyboardReport::DESCRIPTOR,
        request_handler: None,
        poll_ms: 10,
        max_packet_size: 64,
        hid_subclass: HidSubclass::Boot,
        hid_boot_protocol: HidBootProtocol::Keyboard,
    };
    let mut hid_writer: HidWriter<_, 8> = HidWriter::new(&mut builder, &mut hid_state, hid_config);

    // Function 2: MSC disk, backed by the remote daemon over BLE.
    let msc_config = MscConfig::new(64).vendor_id("ESB").product_id("ESB-DISK");
    let mut msc = MscClass::new(&mut builder, &mut msc_state, msc_config);

    let mut usb = builder.build();

    // Soft-disconnect register handle (DCTL.SDIS).
    let otg = unsafe { Otg::from_ptr(USB_FS_BASE as *mut ()) };

    // ---- Status LED (on-board WS2812 on GPIO48, driven over RMT ch 0) ----
    // The channel is non-`Send`, so the `Ws2812` handle must move into a task
    // spawned on the plain (non-`Send`) `Spawner`, not held in `main`.
    let led = Ws2812::new(peripherals.RMT, peripherals.GPIO48)
        .expect("init RMT for status LED");
    s.spawn(led_task(led).expect("spawn status LED task"));

    // ---- Connect to the daemon (GATT server) and discover the protocol ----
    let target = Address::random(DAEMON_ADDR);
    let connect_config = ConnectConfig {
        // The USB host issues multi-sector reads (e.g. 48 KiB) and usb-storage
        // gives up after 30 s. At the 80 ms default connection interval each
        // 512-byte sector costs several round-trips, so such a read would take
        // longer than the timeout. Request a low-latency link instead.
        connect_params: RequestedConnParams {
            min_connection_interval: Duration::from_micros(7_500),
            max_connection_interval: Duration::from_millis(15),
            max_latency: 0,
            min_event_length: Duration::from_secs(0),
            max_event_length: Duration::from_secs(0),
            supervision_timeout: Duration::from_secs(8),
        },
        scan_config: ScanConfig {
            filter_accept_list: &[target],
            ..Default::default()
        },
    };

    let _ = join(runner.run(), async {
        esp_println::println!("[esb] scanning for daemon...");
        let conn = central.connect(&connect_config).await.expect("connect to daemon");
        esp_println::println!("[esb] connected");
        CONNECTED.store(true, Ordering::Release);

        let client = GattClient::<_, DefaultPacketPool, MAX_SERVICES>::new(&stack, &conn)
            .await
            .expect("gatt client");

        // The GATT client's `task()` must run concurrently with every GATT
        // request: it forwards incoming ATT responses into the client's
        // response channel that `services_by_uuid`/`read_characteristic`
        // await. Starting it only after discovery deadlocks (the daemon's
        // replies are never delivered and the 30 s ATT timeout fires).
        let _ = join(client.task(), async {
            let services = client
                .services_by_uuid(&Uuid::new_long(SERVICE_UUID))
                .await
                .expect("find service");
            let service = services.first().expect("service present").clone();

            let usb_enable: Characteristic<u8> = client
                .characteristic_by_uuid(&service, &Uuid::new_long(USB_ENABLE_UUID))
                .await
                .expect("usb_enable char");
            let lba: Characteristic<u32> = client
                .characteristic_by_uuid(&service, &Uuid::new_long(LBA_UUID))
                .await
                .expect("lba char");
            let sector: Characteristic<[u8; SECTOR_SIZE]> = client
                .characteristic_by_uuid(&service, &Uuid::new_long(SECTOR_UUID))
                .await
                .expect("sector char");
            esp_println::println!("[esb] discovered ESB service");

            // All ATT requests through this `GattClient` must be serialized: it
            // has a single, uncorrelated response channel, so the MSC read path
            // and the `usb_enable` poller would otherwise consume each other's
            // responses (`UnexpectedGattResponse`).
            let att_lock = Mutex::new(());

            let mut block_device =
                BleBlockDevice::new(&client, lba, sector, &att_lock);
            // Two blocks so MSC can overlap a USB transfer with a BLE pull.
            let mut block_buf = [0u8; 2 * SECTOR_SIZE];

            // Concurrency: USB device loop + MSC block loop +
            // soft-disconnect poll + HID heartbeat.
            let usb_task = usb.run();
            let msc_task = msc.run(&mut block_device, &mut block_buf);
            let sdis_task = soft_disconnect_loop(&client, &usb_enable, otg, &att_lock);
            let hid_task = hid_heartbeat(&mut hid_writer);

            let _ = join(usb_task, join(msc_task, join(sdis_task, hid_task))).await;
        })
        .await;
    })
    .await;
}

/// Poll the daemon's `usb_enable` flag and mirror it to the OTG soft-disconnect
/// bit (`DCTL.SDIS`): enabled -> attached, disabled -> looks unplugged.
async fn soft_disconnect_loop<'a, 'c, C: Controller>(
    client: &'c GattClient<'a, C, DefaultPacketPool, MAX_SERVICES>,
    usb_enable: &Characteristic<u8>,
    otg: Otg,
    att_lock: &'c Mutex<NoopRawMutex, ()>,
) {
    let mut attached = true; // start attached
    loop {
        let mut val = [0u8; 1];
        let enabled = {
            // Serialize with the MSC read path (single response channel).
            let _guard = att_lock.lock().await;
            match client.read_characteristic(usb_enable, &mut val).await {
                Ok(n) => n == 1 && val[0] != 0,
                // On a transient BLE failure keep the last known state.
                Err(_) => attached,
            }
        };
        if enabled != attached {
            attached = enabled;
            ATTACHED.store(attached, Ordering::Release);
            let sdis = !enabled;
            otg.dctl().modify(|r| r.set_sdis(sdis));
            esp_println::println!("[esb] USB {} to host", if sdis { "unplugged" } else { "attached" });
        }
        Timer::after_millis(500).await;
    }
}

/// Periodically press/release 'A' so the USB host sees the keyboard working.
async fn hid_heartbeat<'d, D: embassy_usb::driver::Driver<'d>>(writer: &mut HidWriter<'d, D, 8>) {
    let key_down = KeyboardReport {
        modifiers: 0,
        keycodes: [KEY_A, 0, 0, 0, 0, 0],
    };
    let key_up = KeyboardReport::default();
    loop {
        writer.ready().await;
        let _ = writer.write(&key_down.to_bytes()).await;
        Timer::after_millis(40).await;
        let _ = writer.write(&key_up.to_bytes()).await;
        Timer::after_millis(10_000).await;
    }
}
