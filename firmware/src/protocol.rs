//! Shared BLE GATT protocol between the ESP32-S3 (GATT *central*) and the
//! Linux daemon (GATT *server* / peripheral).
//!
//! The daemon hosts the virtual-disk data, so the ESP32 must *pull* sectors
//! from it. That inverts the usual roles: the ESP32 is the central and issues
//! ATT reads/writes, the daemon is the peripheral and answers them.
//!
//! The single ESB service exposes three characteristics:
//!
//! - [`USB_ENABLE`]: `u8`, read by the central. `1` = USB attached, `0` =
//!   soft-disconnected (device looks unplugged to the USB host).
//! - [`LBA`]: `u32` (little-endian), written by the central *before* reading a
//!   sector, telling the daemon which logical block to serve.
//! - [`SECTOR`]: 512 bytes, read by the central. The daemon fills it with the
//!   contents of the LBA most recently written to [`LBA`].
//!
//! These UUIDs must match the daemon side exactly.

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

/// Total number of logical blocks exposed to the USB host as the virtual disk.
///
/// The daemon is authoritative for the real size; this is the value the
/// firmware reports in the MSC capacity (10 MiB = 20480 blocks of 512 bytes).
pub const BLOCK_COUNT: u32 = 10 * 1024 * 1024 / SECTOR_SIZE as u32;

/// Max number of GATT services the client keeps. The trouble-host `GattClient`
/// is const-generic over this with no default, so it must be named explicitly.
/// One service is all we really use; the extra headroom is cheap.
pub const MAX_SERVICES: usize = 10;
