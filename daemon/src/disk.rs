//! The virtual-disk backing store. The daemon owns the actual sector data; the
//! ESP32-S3 (GATT central) pulls it over BLE, and the USB host reads it via MSC.
//!
//! Data is read from a regular file on disk. Reads are synchronous
//! (`File::read_at`) because they happen inside the GATT read handler, which is
//! a synchronous callback in trouble-host. A sector past the end of the file
//! reads back as zeros, mimicking an empty/sparse disk region.

use std::fs::File;
use std::os::unix::fs::FileExt;
use std::path::Path;
use std::sync::Arc;

use crate::protocol::{BLOCK_COUNT, SECTOR_SIZE};

/// Total size of the virtual disk in bytes (`BLOCK_COUNT * SECTOR_SIZE`).
const DISK_SIZE: u64 = (BLOCK_COUNT as u64) * (SECTOR_SIZE as u64);

/// A read-only view of the virtual disk backed by a file on disk.
///
/// Cloning is cheap (shares the underlying `File`); `read_sector` takes `&self`,
/// so a single instance can be shared with the GATT task via `Arc`.
#[derive(Clone)]
pub struct Disk {
    file: Arc<File>,
}

impl Disk {
    /// Open the backing file, creating it (sparse, `DISK_SIZE` bytes) if it does
    /// not already exist. An existing file is used as-is; if it is shorter than
    /// the virtual disk, the missing tail reads back as zeros.
    pub fn open_or_create(path: &Path) -> std::io::Result<Self> {
        let file = if path.exists() {
            File::options().read(true).write(true).open(path)?
        } else {
            let f = File::create(path)?;
            f.set_len(DISK_SIZE)?;
            f
        };
        Ok(Self {
            file: Arc::new(file),
        })
    }

    /// Read the sector at `lba` into `buf`. `buf` must be exactly one sector
    /// (`SECTOR_SIZE` bytes). Bytes past the end of the backing file are zeroed.
    pub fn read_sector(&self, lba: u32, buf: &mut [u8; SECTOR_SIZE]) -> std::io::Result<()> {
        let pos = (lba as u64) * (SECTOR_SIZE as u64);
        let n = self.file.read_at(buf, pos)?;
        if n < SECTOR_SIZE {
            buf[n..].fill(0);
        }
        Ok(())
    }
}
