//! Shared BLE GATT protocol between the ESP32-S3 (GATT *central*) and this
//! Linux daemon (GATT *server* / peripheral).
//!
//! The daemon hosts the virtual-disk data, so the ESP32 must *pull* sectors
//! from it. That inverts the usual roles: the ESP32 is the central and issues
//! ATT reads/writes, the daemon is the peripheral and answers them.
//!
//! The single ESB service exposes three characteristics:
//!
//! - [`USB_ENABLE_UUID`]: `u8`, read by the central. `1` = USB attached,
//!   `0` = soft-disconnected (device looks unplugged to the USB host).
//! - [`LBA_UUID`]: `u32` (little-endian), written by the central *before*
//!   reading a sector, telling the daemon which logical block to serve.
//! - [`SECTOR_UUID`]: 512 bytes, read by the central. The daemon fills it with
//!   the contents of the LBA most recently written to [`LBA_UUID`].
//!
//! These UUIDs must match the firmware side (`firmware/src/protocol.rs`)
//! exactly.
use std::sync::atomic::{AtomicU32, AtomicU8, Ordering};

/// 128-bit UUID of the ESB control + storage service.
pub const SERVICE_UUID: [u8; 16] = [
    0x6e, 0x7d, 0x5c, 0x4b, 0x3a, 0x29, 0x18, 0x07, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1, 0x90, 0x81,
];

/// USB enable state (read, 1 byte). `1` = attached, `0` = unplugged.
pub const USB_ENABLE_UUID: [u8; 16] = [
    0x6e, 0x7d, 0x5c, 0x4b, 0x3a, 0x29, 0x18, 0x07, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1, 0x90, 0x82,
];

/// LBA selector (write, 4 bytes little-endian). Set before reading a sector.
pub const LBA_UUID: [u8; 16] = [
    0x6e, 0x7d, 0x5c, 0x4b, 0x3a, 0x29, 0x18, 0x07, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1, 0x90, 0x83,
];

/// Sector contents (read, 512 bytes) for the LBA currently in [`LBA_UUID`].
pub const SECTOR_UUID: [u8; 16] = [
    0x6e, 0x7d, 0x5c, 0x4b, 0x3a, 0x29, 0x18, 0x07, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1, 0x90, 0x84,
];

/// Sector (logical block) size in bytes.
pub const SECTOR_SIZE: usize = 512;

/// Fixed BLE identity address the daemon advertises under. The ESP32-S3 central
/// expects this exact address in its connection `filter_accept_list`, so it must
/// match `DAEMON_ADDR` on the firmware side.
pub const DAEMON_ADDR: [u8; 6] = [0x81, 0x90, 0xa1, 0xb2, 0xc3, 0x0e];

/// Total number of logical blocks the daemon exposes (10 MiB of 512-byte blocks).
pub const BLOCK_COUNT: u32 = (10 * 1024 * 1024 / SECTOR_SIZE) as u32;

/// Server-side protocol state shared between the GATT read handlers and the
/// control loop.
pub struct ProtocolState {
    /// Which LBA the central last requested (written to [`LBA_UUID`]).
    pub lba: AtomicU32,
    /// Whether the USB device should appear attached (served on [`USB_ENABLE_UUID`]).
    pub usb_enable: AtomicU8,
}

impl ProtocolState {
    pub fn new() -> Self {
        Self {
            lba: AtomicU32::new(0),
            // Start attached so the device is visible on first connect.
            usb_enable: AtomicU8::new(1),
        }
    }

    /// The LBA the central most recently requested.
    #[inline]
    pub fn lba(&self) -> u32 {
        self.lba.load(Ordering::Acquire)
    }

    /// Record an LBA written by the central.
    #[inline]
    pub fn set_lba(&self, lba: u32) {
        self.lba.store(lba, Ordering::Release);
    }

    /// Current USB-enable flag value.
    #[inline]
    pub fn usb_enable(&self) -> u8 {
        self.usb_enable.load(Ordering::Acquire)
    }

    /// Set the USB-enable flag (drives plug/unplug on the ESP32).
    #[inline]
    pub fn set_usb_enable(&self, on: bool) {
        self.usb_enable.store(on as u8, Ordering::Release);
    }
}
