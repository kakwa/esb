//! The virtual-disk backing store. The daemon owns the actual sector data; the
//! ESP32-S3 (GATT central) pulls it over BLE, and the USB host reads it via MSC.
//!
//! Data lives in a regular file on disk. Reads and writes use positional I/O
//! (`File::read_at` / `File::write_at`) so the GATT handler and the IPC control
//! task can share one open file without a seek-position race.
//!
//! A process-wide (per-`Disk`) read/write lock keeps the two access paths
//! coherent: sector reads take it shared, while a whole FAT operation (list,
//! copy, format) takes it exclusively through [`Disk::with_exclusive_io`]. This
//! prevents a USB host read from observing a half-updated FAT metadata sector.

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::os::unix::fs::FileExt;
use std::path::Path;
use std::sync::{Arc, RwLock};

use crate::protocol::{BLOCK_COUNT, SECTOR_SIZE};

/// Default total size of the virtual disk in bytes (`BLOCK_COUNT * SECTOR_SIZE`).
pub const DISK_SIZE: u64 = (BLOCK_COUNT as u64) * (SECTOR_SIZE as u64);

/// A view of the virtual disk backed by a file on disk.
///
/// Cloning is cheap (shares the underlying `File`, size and lock); `read_sector`
/// takes `&self`, so a single instance can be shared with the GATT task, the IPC
/// task and client operations via `Arc`/clones.
#[derive(Clone)]
pub struct Disk {
    file: Arc<File>,
    size: u64,
    io_lock: Arc<RwLock<()>>,
}

impl Disk {
    /// Open the backing file, creating it (sparse, [`DISK_SIZE`] bytes) if it
    /// does not already exist. An existing file is used as-is; if it is shorter
    /// than the virtual disk, the missing tail reads back as zeros.
    pub fn open_or_create(path: &Path) -> io::Result<Self> {
        match OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .open(path)
        {
            Ok(file) => {
                file.set_len(DISK_SIZE)?;
                Self::from_file(file)
            }
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {
                let file = OpenOptions::new().read(true).write(true).open(path)?;
                Self::from_file(file)
            }
            Err(e) => Err(e),
        }
    }

    /// Create (or, with `force`, truncate) a backing file of exactly `size`
    /// bytes. Refuses to overwrite an existing file unless `force` is set.
    pub fn create_sized(path: &Path, size: u64, force: bool) -> io::Result<Self> {
        let file = if force {
            OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(true)
                .open(path)?
        } else {
            OpenOptions::new()
                .read(true)
                .write(true)
                .create_new(true)
                .open(path)?
        };
        file.set_len(size)?;
        Self::from_file(file)
    }

    fn from_file(file: File) -> io::Result<Self> {
        let size = file.metadata()?.len();
        Ok(Self {
            file: Arc::new(file),
            size,
            io_lock: Arc::new(RwLock::new(())),
        })
    }

    /// Current backing-file size in bytes.
    #[inline]
    pub fn size(&self) -> u64 {
        self.size
    }

    /// Read the sector at `lba` into `buf`. `buf` must be exactly one sector
    /// (`SECTOR_SIZE` bytes). Bytes past the end of the backing file are zeroed.
    pub fn read_sector(&self, lba: u32, buf: &mut [u8; SECTOR_SIZE]) -> io::Result<()> {
        let _guard = self.read_guard();
        let pos = (lba as u64) * (SECTOR_SIZE as u64);
        let mut n = 0;
        while n < SECTOR_SIZE {
            match self.file.read_at(&mut buf[n..], pos + n as u64) {
                Ok(0) => break,
                Ok(read) => n += read,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e),
            }
        }
        if n < SECTOR_SIZE {
            buf[n..].fill(0);
        }
        Ok(())
    }

    /// Run a block-level FAT operation with exclusive access to the disk. The
    /// closure receives a seekable `Read + Write` view over the whole image.
    ///
    /// Holding the write lock for the entire operation keeps the on-disk
    /// filesystem consistent and prevents concurrent FAT sessions (which would
    /// corrupt metadata).
    pub fn with_exclusive_io<R>(
        &self,
        f: impl FnOnce(&mut DiskIo) -> io::Result<R>,
    ) -> io::Result<R> {
        let _guard = self
            .io_lock
            .write()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        let mut io = DiskIo {
            file: Arc::clone(&self.file),
            size: self.size,
            pos: 0,
        };
        f(&mut io)
    }

    fn read_guard(&self) -> std::sync::RwLockReadGuard<'_, ()> {
        self.io_lock
            .read()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

/// Seekable `Read + Write` adapter over a [`Disk`], suitable for `fatfs`.
pub struct DiskIo {
    file: Arc<File>,
    size: u64,
    pos: u64,
}

impl Read for DiskIo {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.pos >= self.size || buf.is_empty() {
            return Ok(0);
        }
        let remaining = (self.size - self.pos) as usize;
        let len = remaining.min(buf.len());
        let n = self.file.read_at(&mut buf[..len], self.pos)?;
        self.pos += n as u64;
        Ok(n)
    }
}

impl Write for DiskIo {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        if self.pos >= self.size {
            return Err(io::Error::new(
                io::ErrorKind::WriteZero,
                "write past the end of the disk image",
            ));
        }
        let remaining = (self.size - self.pos) as usize;
        let len = remaining.min(buf.len());
        let n = self.file.write_at(&buf[..len], self.pos)?;
        self.pos += n as u64;
        Ok(n)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.file.sync_data()
    }
}

impl Seek for DiskIo {
    fn seek(&mut self, from: SeekFrom) -> io::Result<u64> {
        let new_pos = match from {
            SeekFrom::Start(pos) => pos as i128,
            SeekFrom::End(off) => self.size as i128 + off as i128,
            SeekFrom::Current(off) => self.pos as i128 + off as i128,
        };
        if new_pos < 0 || new_pos > self.size as i128 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "seek outside the disk image",
            ));
        }
        self.pos = new_pos as u64;
        Ok(self.pos)
    }
}
