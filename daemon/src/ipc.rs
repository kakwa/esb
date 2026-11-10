//! Control IPC protocol between the daemon and its CLI client.
//!
//! Transport is a Unix-domain stream socket carrying newline-delimited JSON:
//! the client sends one [`Request`] per line and reads one [`Response`] per
//! line. The daemon handles each connection on a dedicated thread, so a long
//! `put`/`get` does not block attach/detach or status requests.
//!
//! File contents are not embedded in the JSON. `put`/`get` name a path on the
//! daemon host (the client and daemon normally run on the same machine), which
//! keeps the wire protocol small and avoids base64 overhead.

use std::env;
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};

use crate::fat::{EntryInfo, FsInfo};

/// Requests accepted by the daemon control socket.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
pub enum Request {
    /// Attach (plug in) the virtual USB device.
    Attach,
    /// Detach (unplug) the virtual USB device.
    Detach,
    /// Return USB state and image/filesystem summary.
    Status,
    /// List a directory inside the FAT32 image.
    Ls { path: String },
    /// Copy a file from the image to a host path.
    Get {
        image_path: String,
        host_path: PathBuf,
    },
    /// Copy a host file into the image.
    Put {
        host_path: PathBuf,
        image_path: String,
    },
}

/// One response line from the daemon.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "result", rename_all = "snake_case")]
pub enum Response {
    Ok {
        message: String,
    },
    Status {
        status: StatusInfo,
    },
    Listing {
        path: String,
        entries: Vec<EntryInfo>,
    },
    Error {
        message: String,
    },
}

/// Snapshot returned by [`Request::Status`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatusInfo {
    pub usb_enable: bool,
    pub image: String,
    pub image_size: u64,
    pub filesystem: Option<FsInfo>,
    pub filesystem_error: Option<String>,
}

/// Default control socket path. Prefer the per-user runtime directory when it
/// is available so multiple users can run daemons independently.
pub fn default_socket_path() -> PathBuf {
    if let Some(dir) = env::var_os("XDG_RUNTIME_DIR") {
        return PathBuf::from(dir).join("esb-daemon.sock");
    }
    let uid = unsafe { libc::getuid() };
    env::temp_dir().join(format!("esb-daemon-{uid}.sock"))
}

/// Send one request and parse the daemon's response.
pub fn request(socket: &Path, request: &Request) -> Result<Response> {
    let mut stream = UnixStream::connect(socket)
        .with_context(|| format!("connecting to daemon control socket {socket:?}"))?;
    serde_json::to_writer(&mut stream, request).context("encoding IPC request")?;
    stream.write_all(b"\n").context("sending IPC request")?;
    stream.flush().context("sending IPC request")?;

    let mut reader = BufReader::new(stream);
    let mut line = String::new();
    reader
        .read_line(&mut line)
        .context("reading IPC response")?;
    if line.trim().is_empty() {
        bail!("daemon closed the control connection without a response");
    }
    let response = serde_json::from_str(&line).context("decoding IPC response")?;
    Ok(response)
}

/// Pretty-print a response for the CLI, converting `Error` into a process error.
pub fn print_response(response: Response) -> Result<()> {
    match response {
        Response::Ok { message } => println!("{message}"),
        Response::Error { message } => bail!("{message}"),
        Response::Status { status } => {
            println!(
                "USB:   {}",
                if status.usb_enable {
                    "attached"
                } else {
                    "detached"
                }
            );
            println!(
                "Image: {} ({} bytes, {})",
                status.image,
                status.image_size,
                format_size(status.image_size)
            );
            match status.filesystem {
                Some(fs) => {
                    println!(
                        "FS:    {} label {:?}, cluster {}, {} free of {}",
                        fs.fat_type,
                        fs.label,
                        format_size(u64::from(fs.cluster_size)),
                        format_size(fs.free_bytes),
                        format_size(fs.total_bytes),
                    );
                }
                None => {
                    if let Some(error) = status.filesystem_error {
                        println!("FS:    unavailable: {error}");
                    }
                }
            }
        }
        Response::Listing { path, entries } => {
            println!("{path}");
            if entries.is_empty() {
                println!("  (empty)");
            }
            for entry in entries {
                let suffix = if entry.is_dir { "/" } else { "" };
                println!(
                    "  {:>10}  {}  {}{}",
                    if entry.is_dir {
                        String::from("<dir>")
                    } else {
                        format_size(entry.size)
                    },
                    entry.modified,
                    entry.name,
                    suffix
                );
            }
        }
    }
    Ok(())
}

fn format_size(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = 1024 * KB;
    const GB: u64 = 1024 * MB;
    if bytes < KB {
        format!("{bytes}B")
    } else if bytes < MB {
        format!("{}KiB", bytes / KB)
    } else if bytes < GB {
        format!("{}MiB", bytes / MB)
    } else {
        format!("{}GiB", bytes / GB)
    }
}
