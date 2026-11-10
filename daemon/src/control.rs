//! Unix-domain control server.
//!
//! The daemon listens on a local socket so another process (the bundled CLI or
//! any script) can plug/unplug the virtual USB device and inspect or modify the
//! FAT32 image while the daemon is running. All filesystem work happens in this
//! process, in userland, through [`crate::fat`]; no mount is performed.

use std::fs;
use std::io::{BufRead, BufReader, Write};
use std::os::unix::fs::{FileTypeExt, PermissionsExt};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{bail, Context, Result};
use log::{info, warn};

use crate::disk::Disk;
use crate::fat;
use crate::ipc::{Request, Response, StatusInfo};
use crate::protocol::ProtocolState;

/// Bind the control socket and serve requests in the background.
///
/// Returns once the listener is ready; the accept loop runs on its own thread.
pub fn spawn(
    socket_path: &Path,
    image_path: PathBuf,
    state: Arc<ProtocolState>,
    disk: Disk,
) -> Result<()> {
    prepare_socket(socket_path)?;
    let listener = UnixListener::bind(socket_path)
        .with_context(|| format!("binding control socket {socket_path:?}"))?;
    // Keep the control channel private to the user running the daemon.
    let permissions = fs::Permissions::from_mode(0o600);
    fs::set_permissions(socket_path, permissions)
        .with_context(|| format!("setting permissions on {socket_path:?}"))?;

    info!("control IPC listening on {:?}", socket_path);

    std::thread::Builder::new()
        .name("esb-control".to_string())
        .spawn(move || {
            for connection in listener.incoming() {
                match connection {
                    Ok(stream) => {
                        let state = Arc::clone(&state);
                        let disk = disk.clone();
                        let image_path = image_path.clone();
                        std::thread::Builder::new()
                            .name("esb-control-client".to_string())
                            .spawn(move || {
                                if let Err(e) =
                                    handle_connection(stream, &image_path, &state, &disk)
                                {
                                    warn!("[control] client error: {e:#}");
                                }
                            })
                            .ok();
                    }
                    Err(e) => warn!("[control] accept failed: {e}"),
                }
            }
        })
        .context("spawning control IPC thread")?;

    Ok(())
}

fn prepare_socket(path: &Path) -> Result<()> {
    match fs::symlink_metadata(path) {
        Ok(metadata) if metadata.file_type().is_socket() => {
            // A previous daemon exited without unlinking its socket.
            fs::remove_file(path)
                .with_context(|| format!("removing stale control socket {path:?}"))?;
        }
        Ok(_) => bail!("refusing to replace {path:?}: it exists and is not a Unix socket"),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => return Err(e).with_context(|| format!("checking {path:?}")),
    }
    Ok(())
}

fn handle_connection(
    stream: UnixStream,
    image_path: &Path,
    state: &ProtocolState,
    disk: &Disk,
) -> Result<()> {
    let reader = BufReader::new(stream.try_clone().context("cloning control socket")?);
    let mut writer = stream;

    for line in reader.lines() {
        let line = line.context("reading control request")?;
        if line.trim().is_empty() {
            continue;
        }

        let response = match serde_json::from_str::<Request>(&line) {
            Ok(request) => handle_request(request, image_path, state, disk),
            Err(e) => Response::Error {
                message: format!("invalid request: {e}"),
            },
        };

        serde_json::to_writer(&mut writer, &response).context("encoding control response")?;
        writer
            .write_all(b"\n")
            .context("sending control response")?;
        writer.flush().context("sending control response")?;
    }
    Ok(())
}

fn handle_request(
    request: Request,
    image_path: &Path,
    state: &ProtocolState,
    disk: &Disk,
) -> Response {
    match request {
        Request::Attach => {
            state.set_usb_enable(true);
            info!("[control] USB attached");
            Response::Ok {
                message: "USB attached".to_string(),
            }
        }
        Request::Detach => {
            state.set_usb_enable(false);
            info!("[control] USB detached");
            Response::Ok {
                message: "USB detached".to_string(),
            }
        }
        Request::Status => {
            let (filesystem, filesystem_error) = match fat::info(disk) {
                Ok(fs) => (Some(fs), None),
                Err(e) => (None, Some(format!("{e:#}"))),
            };
            Response::Status {
                status: StatusInfo {
                    usb_enable: state.usb_enable() != 0,
                    image: image_path.display().to_string(),
                    image_size: disk.size(),
                    filesystem,
                    filesystem_error,
                },
            }
        }
        Request::Ls { path } => match fat::list(disk, &path) {
            Ok((path, entries)) => Response::Listing { path, entries },
            Err(e) => Response::Error {
                message: format!("{e:#}"),
            },
        },
        Request::Get {
            image_path,
            host_path,
        } => match fat::copy_out(disk, &image_path, &host_path) {
            Ok(bytes) => Response::Ok {
                message: format!(
                    "copied {bytes} bytes from {} to {}",
                    image_path,
                    host_path.display()
                ),
            },
            Err(e) => Response::Error {
                message: format!("{e:#}"),
            },
        },
        Request::Put {
            host_path,
            image_path,
        } => match fat::copy_in(disk, &host_path, &image_path) {
            Ok(bytes) => Response::Ok {
                message: format!(
                    "copied {bytes} bytes from {} to {}",
                    host_path.display(),
                    image_path
                ),
            },
            Err(e) => Response::Error {
                message: format!("{e:#}"),
            },
        },
    }
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::sync::atomic::{AtomicU64, Ordering};

    use super::*;
    use crate::fat::FAT32_MIN_SIZE;

    static NEXT_ID: AtomicU64 = AtomicU64::new(0);

    fn temp_path(tag: &str) -> PathBuf {
        let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir().join(format!(
            "esb-daemon-ipc-test-{}-{tag}-{id}",
            std::process::id()
        ))
    }

    #[test]
    fn ipc_attach_status_and_file_copy() {
        let image = temp_path("image.img");
        let socket = temp_path("control.sock");
        let host_in = temp_path("in.txt");
        let host_out = temp_path("out.txt");
        let payload = b"ipc roundtrip";

        fat::init(&image, FAT32_MIN_SIZE, true).expect("format image");
        let disk = Disk::open_or_create(&image).expect("open image");
        let state = Arc::new(ProtocolState::new());
        spawn(&socket, image.clone(), Arc::clone(&state), disk).expect("spawn server");

        let response = crate::ipc::request(&socket, &Request::Detach).expect("detach");
        assert!(matches!(response, Response::Ok { .. }));
        assert_eq!(state.usb_enable(), 0);

        let response = crate::ipc::request(&socket, &Request::Attach).expect("attach");
        assert!(matches!(response, Response::Ok { .. }));
        assert_eq!(state.usb_enable(), 1);

        fs::write(&host_in, payload).expect("write host input");
        let response = crate::ipc::request(
            &socket,
            &Request::Put {
                host_path: host_in.clone(),
                image_path: "/hello.txt".to_string(),
            },
        )
        .expect("put");
        assert!(matches!(response, Response::Ok { .. }), "{response:?}");

        let response = crate::ipc::request(
            &socket,
            &Request::Ls {
                path: "/".to_string(),
            },
        )
        .expect("ls");
        match response {
            Response::Listing { entries, .. } => {
                assert!(entries.iter().any(|e| e.name == "hello.txt"));
            }
            other => panic!("unexpected ls response: {other:?}"),
        }

        let response = crate::ipc::request(
            &socket,
            &Request::Get {
                image_path: "/hello.txt".to_string(),
                host_path: host_out.clone(),
            },
        )
        .expect("get");
        assert!(matches!(response, Response::Ok { .. }), "{response:?}");
        assert_eq!(fs::read(&host_out).expect("read host output"), payload);

        let response = crate::ipc::request(&socket, &Request::Status).expect("status");
        match response {
            Response::Status { status } => {
                assert!(status.usb_enable);
                assert_eq!(status.image_size, FAT32_MIN_SIZE);
                assert_eq!(status.filesystem.expect("FAT info").fat_type, "FAT32");
            }
            other => panic!("unexpected status response: {other:?}"),
        }

        let _ = fs::remove_file(socket);
        let _ = fs::remove_file(image);
        let _ = fs::remove_file(host_in);
        let _ = fs::remove_file(host_out);
    }
}
