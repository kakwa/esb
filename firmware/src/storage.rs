//! BLE-backed block device for the MSC class.
//!
//! The actual disk contents live on the Linux daemon (the GATT *server*). When
//! the USB host issues an MSC READ, [`AsyncBlockDevice::read_blocks`] is called;
//! for each 512-byte block we first *write* the desired LBA to the [`LBA`]
//! characteristic and then *read* the [`SECTOR`] characteristic, which the daemon
//! fills with that block. This "write-LBA-then-read-sector" pull sequence is the
//! protocol defined in [`crate::protocol`].
//!
//! [`LBA`]: crate::protocol::LBA_UUID
//! [`SECTOR`]: crate::protocol::SECTOR_UUID

use embassy_usb::class::msc::device::AsyncBlockDevice;
use trouble_host::prelude::*;

use crate::protocol::{BLOCK_COUNT, MAX_SERVICES, SECTOR_SIZE};

/// Error surfaced to the MSC layer. The MSC layer only ever inspects `.is_err()`,
/// so a zero-sized marker is enough to carry a failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BleError;

/// A block device whose data is hosted on the remote daemon and fetched over BLE.
///
/// Borrows a live [`GattClient`] (the GATT central) plus the two characteristics
/// used by the pull protocol.
pub struct BleBlockDevice<'a, 'c, C: Controller> {
    client: &'c GattClient<'a, C, DefaultPacketPool, MAX_SERVICES>,
    lba: Characteristic<u32>,
    sector: Characteristic<[u8; SECTOR_SIZE]>,
}

impl<'a, 'c, C: Controller> BleBlockDevice<'a, 'c, C> {
    pub fn new(
        client: &'c GattClient<'a, C, DefaultPacketPool, MAX_SERVICES>,
        lba: Characteristic<u32>,
        sector: Characteristic<[u8; SECTOR_SIZE]>,
    ) -> Self {
        Self { client, lba, sector }
    }
}

impl<C: Controller> AsyncBlockDevice for BleBlockDevice<'_, '_, C> {
    type Error = BleError;

    fn block_size(&self) -> u32 {
        SECTOR_SIZE as u32
    }

    fn block_count(&self) -> u32 {
        BLOCK_COUNT
    }

    async fn read_blocks(&mut self, lba: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        for (offset, chunk) in buf.chunks_exact_mut(SECTOR_SIZE).enumerate() {
            let cur = lba + offset as u32;
            // 1. Tell the daemon which LBA to serve.
            self.client
                .write_characteristic(&self.lba, &cur.to_le_bytes())
                .await
                .map_err(|_| BleError)?;
            // 2. Fetch the 512-byte sector. `read_characteristic` uses ATT Read Blob
            //    internally, looping until `chunk` is full, so no manual chunking.
            let n = self
                .client
                .read_characteristic(&self.sector, chunk)
                .await
                .map_err(|_| BleError)?;
            if n != SECTOR_SIZE {
                return Err(BleError);
            }
        }
        Ok(())
    }

    async fn write_blocks(&mut self, _lba: u32, _data: &[u8]) -> Result<(), Self::Error> {
        // The media is hosted read-only on the daemon; writes are a no-op.
        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn is_write_protected(&self) -> bool {
        // The media is hosted read-only on the daemon.
        true
    }
}
