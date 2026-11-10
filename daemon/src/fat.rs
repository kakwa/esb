//! Userland FAT32 access to the backing image.
//!
//! The daemon owns the image file, so all filesystem operations run in-process
//! against the same blocking `Disk` used by the BLE sector server. There is no
//! kernel mount, loop device or external `mkfs`/`mount` tool involved: `fatfs`
//! reads and writes the image through [`DiskIo`].
//!
//! Only FAT32 is accepted for exploration/copy. The formatter always requests
//! FAT32; [`FAT32_MIN_SIZE`] is a conservative lower bound (the FAT32 cluster
//! count minimum means a useful image is at least ~33 MiB).

use std::fs::File;
use std::io::{self, Seek, Write};
use std::path::Path;

use anyhow::{bail, Context, Result};
use fatfs::{format_volume, FatType, FileSystem, FormatVolumeOptions, FsOptions};
use serde::{Deserialize, Serialize};

use crate::disk::{Disk, DiskIo, DISK_SIZE};
use crate::protocol::SECTOR_SIZE;

/// Conservative minimum image size accepted by `init` for a FAT32 volume.
pub const FAT32_MIN_SIZE: u64 = 64 * 1024 * 1024;

/// FAT volume label written by `init`.
const VOLUME_LABEL: [u8; 11] = *b"ESB        ";

/// One directory entry returned by [`list`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EntryInfo {
    /// Long file name (or short name when no LFN exists).
    pub name: String,
    pub is_dir: bool,
    pub size: u64,
    /// DOS timestamp formatted as `YYYY-MM-DD HH:MM:SS`.
    pub modified: String,
}

/// Filesystem summary returned by `status`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FsInfo {
    pub fat_type: String,
    pub label: String,
    pub total_bytes: u64,
    pub free_bytes: u64,
    pub cluster_size: u32,
}

/// Create a fresh FAT32 image of `size` bytes at `path`.
///
/// Refuses to overwrite an existing file unless `force` is set. Returns the
/// resulting filesystem summary after re-opening the image to verify it really
/// is FAT32.
pub fn init(path: &Path, size: u64, force: bool) -> Result<FsInfo> {
    if size < FAT32_MIN_SIZE {
        bail!(
            "image size {} bytes is too small for FAT32 (minimum {})",
            size,
            FAT32_MIN_SIZE
        );
    }
    if size > DISK_SIZE {
        bail!(
            "image size {} bytes exceeds the exposed virtual disk size {} bytes",
            size,
            DISK_SIZE
        );
    }
    if size % SECTOR_SIZE as u64 != 0 {
        bail!("image size must be a multiple of {SECTOR_SIZE} bytes");
    }

    let disk = Disk::create_sized(path, size, force)
        .with_context(|| format!("creating backing image {path:?}"))?;

    disk.with_exclusive_io(|io| {
        let options = FormatVolumeOptions::new()
            .fat_type(FatType::Fat32)
            .volume_label(VOLUME_LABEL);
        format_volume(io, options)
    })
    .with_context(|| format!("formatting {path:?} as FAT32"))?;

    let info = info(&disk).with_context(|| format!("verifying FAT32 image {path:?}"))?;
    if info.fat_type != "FAT32" {
        bail!(
            "formatting produced {} instead of FAT32; use a larger image (minimum {})",
            info.fat_type,
            FAT32_MIN_SIZE
        );
    }
    Ok(info)
}

/// List a directory inside the image. `path` may be `"/"`, `"/sub/dir"`, or
/// relative (`"sub/dir"`).
pub fn list(disk: &Disk, path: &str) -> Result<(String, Vec<EntryInfo>)> {
    let clean = clean_dir_path(path);
    let mut entries = disk
        .with_exclusive_io(|io| {
            let fs = open_fat32(io)?;
            let entries = {
                let root = fs.root_dir();
                let dir = if clean == "/" {
                    root
                } else {
                    root.open_dir(&clean)?
                };

                let mut entries = Vec::new();
                for entry in dir.iter() {
                    let entry = entry?;
                    let modified = entry.modified();
                    entries.push(EntryInfo {
                        name: entry.file_name(),
                        is_dir: entry.is_dir(),
                        size: entry.len(),
                        modified: format!(
                            "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
                            modified.date.year,
                            modified.date.month,
                            modified.date.day,
                            modified.time.hour,
                            modified.time.min,
                            modified.time.sec,
                        ),
                    });
                }
                entries
            };
            fs.unmount()?;
            Ok(entries)
        })
        .with_context(|| format!("listing {clean:?} in FAT32 image"))?;

    entries.sort_by(|a, b| {
        b.is_dir
            .cmp(&a.is_dir)
            .then_with(|| a.name.to_lowercase().cmp(&b.name.to_lowercase()))
    });
    Ok((clean, entries))
}

/// Copy `image_path` out of the image to `host_path`. Returns bytes copied.
pub fn copy_out(disk: &Disk, image_path: &str, host_path: &Path) -> Result<u64> {
    let clean = clean_file_path(image_path)?;
    let copied = disk
        .with_exclusive_io(|io| {
            let fs = open_fat32(io)?;
            let mut src = fs.root_dir().open_file(&clean)?;
            let mut dst = File::create(host_path)?;
            let copied = io::copy(&mut src, &mut dst)?;
            dst.flush()?;
            drop(src);
            fs.unmount()?;
            Ok(copied)
        })
        .with_context(|| format!("copying {clean:?} from image to {host_path:?}"))?;
    Ok(copied)
}

/// Copy `host_path` into the image at `image_path`, creating or truncating it.
/// Returns bytes copied.
pub fn copy_in(disk: &Disk, host_path: &Path, image_path: &str) -> Result<u64> {
    let clean = clean_file_path(image_path)?;
    let mut src =
        File::open(host_path).with_context(|| format!("opening host file {host_path:?}"))?;

    let copied = disk
        .with_exclusive_io(|io| {
            let fs = open_fat32(io)?;
            let mut dst = fs.root_dir().create_file(&clean)?;
            dst.seek(std::io::SeekFrom::Start(0))?;
            dst.truncate()?;
            let copied = io::copy(&mut src, &mut dst)?;
            dst.flush()?;
            drop(dst);
            fs.unmount()?;
            Ok(copied)
        })
        .with_context(|| format!("copying {host_path:?} into image as {clean:?}"))?;
    Ok(copied)
}

/// Read filesystem metadata/summary from the image.
pub fn info(disk: &Disk) -> Result<FsInfo> {
    disk.with_exclusive_io(|io| {
        let fs = open_fat32(io)?;
        let stats = fs.stats()?;
        let label = fs.volume_label();
        let fat_type = fat_type_name(fs.fat_type());
        let cluster_size = stats.cluster_size();
        let total_bytes = u64::from(stats.total_clusters()) * u64::from(cluster_size);
        let free_bytes = u64::from(stats.free_clusters()) * u64::from(cluster_size);
        fs.unmount()?;
        Ok(FsInfo {
            fat_type,
            label,
            total_bytes,
            free_bytes,
            cluster_size,
        })
    })
    .map_err(Into::into)
}

fn open_fat32<'a>(io: &'a mut DiskIo) -> io::Result<FileSystem<&'a mut DiskIo>> {
    let fs = FileSystem::new(io, FsOptions::new())?;
    if fs.fat_type() != FatType::Fat32 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "image contains {}; only FAT32 is supported",
                fat_type_name(fs.fat_type())
            ),
        ));
    }
    Ok(fs)
}

fn fat_type_name(fat_type: FatType) -> String {
    match fat_type {
        FatType::Fat12 => "FAT12",
        FatType::Fat16 => "FAT16",
        FatType::Fat32 => "FAT32",
    }
    .to_string()
}

fn clean_dir_path(path: &str) -> String {
    let trimmed = path.trim_matches('/');
    if trimmed.is_empty() || trimmed == "." {
        "/".to_string()
    } else {
        format!("/{trimmed}")
    }
}

fn clean_file_path(path: &str) -> Result<String> {
    let trimmed = path.trim_matches('/');
    if trimmed.is_empty() || trimmed == "." || trimmed == ".." {
        bail!("file path must name a file inside the image");
    }
    Ok(trimmed.to_string())
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::sync::atomic::{AtomicU64, Ordering};

    use super::*;

    static NEXT_ID: AtomicU64 = AtomicU64::new(0);

    fn temp_path(tag: &str) -> std::path::PathBuf {
        let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir().join(format!("esb-daemon-test-{}-{tag}-{id}", std::process::id()))
    }

    #[test]
    fn fat32_init_put_list_get_roundtrip() {
        let image = temp_path("image.img");
        let host_in = temp_path("in.bin");
        let host_out = temp_path("out.bin");
        let payload = b"hello from the userland FAT32 test\n";

        let info = init(&image, FAT32_MIN_SIZE, true).expect("format FAT32");
        assert_eq!(info.fat_type, "FAT32");
        let disk = Disk::open_or_create(&image).expect("open image");

        fs::write(&host_in, payload).expect("write host input");
        let copied = copy_in(&disk, &host_in, "/docs/hello.txt");
        // The parent directory does not exist, so this must fail cleanly.
        assert!(copied.is_err());

        let copied = copy_in(&disk, &host_in, "/hello.txt").expect("copy in");
        assert_eq!(copied, payload.len() as u64);

        let (path, entries) = list(&disk, "/").expect("list root");
        assert_eq!(path, "/");
        let entry = entries
            .iter()
            .find(|e| e.name == "hello.txt")
            .expect("file listed");
        assert!(!entry.is_dir);
        assert_eq!(entry.size, payload.len() as u64);

        let copied = copy_out(&disk, "/hello.txt", &host_out).expect("copy out");
        assert_eq!(copied, payload.len() as u64);
        assert_eq!(fs::read(&host_out).expect("read host output"), payload);

        let _ = fs::remove_file(image);
        let _ = fs::remove_file(host_in);
        let _ = fs::remove_file(host_out);
    }

    #[test]
    fn rejects_non_fat32_image() {
        let image = temp_path("empty.img");
        let disk = Disk::create_sized(&image, 1024 * 1024, true).expect("create image");
        let error = info(&disk).expect_err("zeroed image is not FAT");
        assert!(
            format!("{error:#}").contains("FAT32")
                || format!("{error:#}").contains("BPB")
                || format!("{error:#}").contains("boot"),
            "unexpected error: {error:#}"
        );
        let _ = fs::remove_file(image);
    }
}
