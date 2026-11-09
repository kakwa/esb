# ESB — ESP32-S3 BLE-controlled virtual USB device

An ESP32-S3 presented to a USB host computer as a **composite USB device**
(HID keyboard + MSC mass-storage disk) whose **plug/unplug state** and whose
**disk contents** are controlled remotely over **Bluetooth Low Energy**.

The virtual-disk data does not live on the ESP32 at all: it is hosted in a file
on a remote Linux machine and streamed over BLE on demand.

```
┌──────────────────┐   BLE (GATT)   ┌──────────────────┐   USB   ┌────────────┐
│  Linux host      │◄──────────────►│   ESP32-S3       │────────►│ USB host   │
│                  │  central pulls │                  │         │ computer   │
│  esb-daemon      │  sectors,      │  firmware        │         │            │
│  (GATT server /  │  polls flag    │  (GATT central + │         │ sees a     │
│   peripheral)    │                │   USB composite) │         │ keyboard + │
│  esb.img         │                │                  │         │ MSC disk   │
│  (backing file)  │                │                  │         │ that can   │
└──────────────────┘                └──────────────────┘         │ "unplug"   │
                                                                 └────────────┘
```

## How it works

**Role inversion.** The daemon *hosts the data*, so the usual BLE roles are
swapped: the daemon is the GATT **server** (peripheral, advertises as
`ESB-Daemon`) and the ESP32 is the GATT **central** (connects to the daemon,
pulls data). The ESP32 whitelists the daemon's fixed random address, connects,
and then:

1. **Polls `usb_enable`** (read, `u8`). The firmware compares the new value
   against the previous one and toggles the USB-OTG soft-disconnect bit
   (`DCTL.SDIS` on the Synopsys OTG controller). When soft-disconnected the
   ESP32 looks unplugged to the USB host; when re-enabled it re-enumerates.
2. **Serves disk reads.** When the USB host reads an MSC sector, the
   firmware's `BleBlockDevice` (an `AsyncBlockDevice`) performs a
   *write-LBA-then-read-sector* sequence: it writes the requested LBA to the
   daemon, then reads the 512-byte sector characteristic, which the daemon
   fills from the backing file.
3. **HID heartbeat.** Every 10 s the keyboard emits a press/release of the
   `A` key, so the host can see the device is alive.

**Plug/unplug is driven at runtime by signals to the daemon:**

| Signal    | Effect                         |
|-----------|--------------------------------|
| `SIGUSR1` | USB attached  (`usb_enable=1`) |
| `SIGUSR2` | USB detached  (`usb_enable=0`) |

The initial state is set with `--no-usb` (start detached).

## Repository layout

```
Cargo.toml        workspace root (also holds the [patch.crates-io] pins)
firmware/         ESP32-S3 firmware (no_std, pure Rust, no ESP-IDF)
  src/main.rs     startup, USB composite device, BLE central, soft-disconnect loop
  src/protocol.rs shared GATT UUIDs / addresses / constants
  src/storage.rs  BleBlockDevice (AsyncBlockDevice backed by BLE)
  src/csprng.rs   ChaCha12 CSPRNG registered as the embassy-crypto RNG driver
daemon/           Linux daemon (std, tokio)
  src/main.rs     GATT server, advertisement, event dispatch, signal toggles, CLI
  src/protocol.rs shared GATT UUIDs / addresses / constants
  src/disk.rs     file-backed sector store (File::read_at, zero-filled tail)
```

## GATT protocol

One service, three dynamic (event-served) characteristics. **The UUIDs and
addresses in `firmware/src/protocol.rs` and `daemon/src/protocol.rs` must stay
byte-identical** — that is the whole contract between the two sides.

| Characteristic | UUID suffix | Access  | Type             | Meaning                                    |
|----------------|-------------|---------|------------------|--------------------------------------------|
| `usb_enable`   | `0x82`      | read    | `u8`             | `1` = attached, `0` = soft-disconnected    |
| `lba`          | `0x83`      | write   | `u32` LE         | LBA the central wants next                 |
| `sector`       | `0x84`      | read    | 512 bytes        | contents of the last-written LBA           |

- Service UUID: `…a1:90:81` (full value in `protocol.rs` on both sides).
- Disk geometry: 512-byte sectors, 20 480 sectors = **10 MiB** virtual disk.
- Daemon address (advertised random, whitelisted by the firmware):
  `0E:C3:B2:A1:90:81` (`DAEMON_ADDR = [0x81,0x90,0xa1,0xb2,0xc3,0x0e]`,
  byte array is LSB-first, so the display order is the reverse).
- Firmware address: `02:02:02:02:02:02`.
- Read/write handlers are dynamic: the daemon serves values in its GATT event
  task (`accept_unprocessed` auto-chunks the 512-byte sector across Read-Blob
  PDUs); the firmware pulls with auto-Read-Blob.

## Building

### Firmware (ESP32-S3)

Pure Rust — no ESP-IDF, no C. Requires the esp Rust toolchain
(`esp` nightly from [espup](https://espup.rs/)) with the
`xtensa-esp32s3-none-elf` target, and a `LIBCLANG_PATH` pointing at
esp-clang (needed by `esp-backtrace`/bindgen-style metadata at build time).

```sh
# example env (adjust paths to your espup install):
export LIBCLANG_PATH="$RUSTUP_HOME/toolchains/esp/xtensa-esp32-elf-clang/<ver>/esp-clang/lib"
export PATH="$RUSTUP_HOME/toolchains/esp/bin:$RUSTUP_HOME/toolchains/esp/xtensa-esp-elf/<ver>/xtensa-esp-elf/bin:$PATH"
export RUSTC="$RUSTUP_HOME/toolchains/esp/bin/rustc"

cargo build --release --target xtensa-esp32s3-none-elf
# artifact:
#   target/xtensa-esp32s3-none-elf/release/firmware   (ELF, ~524 KB)
```

**The artifact is the ELF — espflash programs it directly.** espflash also
bundles and flashes a prebuilt ESP-IDF v6.1 second-stage bootloader and
partition table, so no separate `.bin` or bootloader is needed.

The workspace root `[patch.crates-io]` pins `esp-*` to one git rev and
`embassy-usb`/`embassy-time`/etc. to the git revs the firmware needs (the
crates.io `embassy-usb 0.6.0` predates the MSC/HID *device* classes).
`[patch]` only takes effect at the workspace root — build from the repo root
or inside the workspace.

### Daemon (Linux)

Plain stable Rust:

```sh
cargo build --release -p daemon
# artifact: target/release/daemon
```

The daemon links the same `trouble-host` git source as the firmware (with the
`security` feature enabled) so both sides share one copy of the stack.

## Flashing the firmware

The bring-up board is an ESP32-S3 connected through a **CH341 USB-UART bridge**
(`/dev/ttyACM0`, `1a86:55d3`). Use [espflash](https://espflash.rs/)
(the esp-rs org):

```sh
# port access via dialout (no root needed):
sg dialout -c '/path/to/espflash flash --chip esp32s3 --port /dev/ttyACM0 --baud 460800 \
    target/xtensa-esp32s3-none-elf/release/firmware'
```

espflash handles entering download mode (auto-reset), then writes **bootloader
+ partition table + app** in one step and verifies.

### Bring-up notes specific to this board

- **Serial access:** the port is `root:dialout`; if your session is not in
  `dialout`, wrap port-touching commands in `sg dialout -c '...'`.
- **Reset is on RTS, not DTR.** On this board the reset line is driven by RTS
  (through an inverter), so a DTR-only pulse does *not* reset the chip.
  Additionally, `TIOCMSET` is ignored on the CH341 — individual bits only work
  via `TIOCMBIS` (0x5416) / `TIOCMBCL` (0x5417). The reliable reset is
  espflash's own (CTRL+R in `espflash monitor`), which is the path used for
  boot captures.
- **Verifying the flash** without root:
  `sg dialout -c 'espflash read-flash --chip esp32s3 --port /dev/ttyACM0 0x0 0x11000 head.bin'`
  — expected layout: bootloader @0x0, partition table @0x8000 (nvs, phy_init,
  factory), app @0x10000.
- **Capturing boot output** (the monitor needs a real TTY):
  `sg dialout -c 'espflash monitor --port /dev/ttyACM0'` and press CTRL+R, or
  drive it under a PTY. A successful boot ends with:
  `boot: Loaded app from partition at offset 0x10000` followed by
  `[esb] scanning for daemon...` and the `trouble-host` GATT-central init
  log lines.

## Running the daemon

```sh
# BlueZ must be running and the HCI device must be up (usually needs root or
# CAP_NET_ADMIN, e.g. `sudo hci0 up` / `hciconfig hci0 up`).
./target/release/daemon \
    --device 0 \          # HCI device number (default 0)
    --image esb.img \     # backing file; created sparse (10 MiB) if missing
    --no-usb              # optional: start in the detached (unplugged) state
```

Then toggle plug/unplug at runtime:

```sh
kill -USR1 <daemon-pid>   # attach  -> host sees the device appear
kill -USR2 <daemon-pid>   # detach  -> host sees the device vanish
```

To put content on the virtual disk, just write to the backing file before or
while running (reads past EOF come back as zeros, like a sparse disk):

```sh
dd if=/dev/urandom of=esb.img bs=512 count=4   # fill the first 2 KiB
```

## Security

Both sides enable `trouble-host`'s `security` feature (LE pairing / link-layer
security state machine; the daemon additionally links `embassy-crypto-rand` +
`embassy-crypto-rustcrypto` as the std crypto backend). **The firmware never
calls `pair()` during bring-up**, so the expected path is unsecured ATT:
the central connects and reads/writes without pairing. Pairing can be layered
on later without changing the protocol.

## Status / known limitations

- **Bring-up verified:** firmware boots on the board (app segments load,
  `[esb] scanning for daemon...` printed, BLE central stack initialized and
  scanning). Daemon builds and its CLI is verified.
- **Live end-to-end BLE not yet exercised:** the host's `hci0` was down and
  could not be brought up without root in the bring-up environment, so the
  daemon↔ESP32 connection, sector streaming, and soft-disconnect toggle still
  need a first live run on a host with a working, upped HCI radio.
- The virtual disk is **read-only** (the MSC read path only); writes are not
  implemented.
- Single connection, single service — sized for exactly the ESP32↔daemon link.
