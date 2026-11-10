//! On-board WS2812 status LED (single pixel, GRB, GPIO48) driven over RMT.
//!
//! The Freenove ESP32-S3 WROOM exposes one addressable WS2812 on GPIO48. We
//! drive it directly with the esp-hal RMT TX channel 0. The RMT counter clock
//! is the (fixed) 80 MHz APB clock divided by 9 → 8 MHz → 0.125 µs per tick.
//!
//! WS2812 timing in ticks (0.125 µs each):
//!   0-bit = 3 ticks high + 7 ticks low   (375 ns high / 875 ns low)
//!   1-bit = 10 ticks high + 3 ticks low  (1.25 µs high / 375 ns low)
//!   reset = >= 240 µs low                (we emit 2240 ticks ≈ 280 µs)

use esp_hal::{
    gpio::{Level, interconnect::PeripheralOutput},
    peripherals::RMT,
    rmt::{
        Channel, ConfigError, Error, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator,
    },
    time::Rate,
    Async,
};

/// RMT counter-clock divider: 80 MHz / 9 = 8 MHz (0.125 µs/tick).
const DIV: u8 = 9;

/// 0-bit: high ticks / low ticks.
const T0H: u16 = 3;
const T0L: u16 = 7;
/// 1-bit: high ticks / low ticks.
const T1H: u16 = 10;
const T1L: u16 = 3;
/// Reset gap (held low) after the final bit; >= 240 µs required.
const RESET_LOW: u16 = 2240;

/// A single-pixel WS2812 (GRB order) driven over RMT TX channel 0.
pub struct Ws2812 {
    ch: Channel<'static, Async, Tx>,
}

impl Ws2812 {
    /// Bring up RMT TX channel 0 on `pin` (GPIO48).
    ///
    /// `rmt` is the raw RMT peripheral, moved in by value.
    pub fn new(
        rmt: RMT<'static>,
        pin: impl PeripheralOutput<'static>,
    ) -> Result<Self, ConfigError> {
        let rmt = Rmt::new(rmt, Rate::from_mhz(80))?;
        // Partial-move `channel0` out of the owned `Rmt` (which has no `Drop`)
        // so the live `ChannelGuards` travel with the `Channel` we keep.
        let ch = rmt
            .into_async()
            .channel0
            .configure_tx(&TxChannelConfig::default().with_clk_divider(DIV))?
            .with_pin(pin);
        Ok(Self { ch })
    }

    /// Write one pixel in (R, G, B) and drive it. Awaits until the RMT has
    /// emitted the full frame including the trailing reset gap.
    pub async fn set_rgb(&mut self, r: u8, g: u8, b: u8) -> Result<(), Error> {
        // 24 data bits (GRB, MSB first) + 1 reset code.
        let mut codes = [PulseCode::new(Level::High, T0H, Level::Low, T0L); 25];
        for (i, &byte) in [g, r, b].iter().enumerate() {
            for (code, bit) in codes[i * 8..(i + 1) * 8].iter_mut().zip((0u8..8).rev()) {
                *code = if byte & (1 << bit) != 0 {
                    PulseCode::new(Level::High, T1H, Level::Low, T1L)
                } else {
                    PulseCode::new(Level::High, T0H, Level::Low, T0L)
                };
            }
        }
        // Reset gap: hold low, then stop (`length2 == 0` is the end marker).
        codes[24] = PulseCode::new(Level::Low, RESET_LOW, Level::Low, 0);
        self.ch.transmit(&codes).await
    }
}
