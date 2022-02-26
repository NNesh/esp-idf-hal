//! Blinks an LED
//!
//! This assumes that a LED is connected to GPIO4.
//! Depending on your target and the board you are using you should change the pin.
//! If your board doesn't have on-board LEDs don't forget to add an appropriate resistor.
//!

use std::mem::zeroed;
use std::thread;
use std::time::Duration;

use esp_idf_hal::adc::ChannelData;
use esp_idf_hal::adc::ContinuousADC;
use esp_idf_hal::adc::PoweredAdc;
use esp_idf_hal::adc::config::dma::AttenChannel;
use esp_idf_hal::adc::config::dma::DmaChannel;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::adc::config::Config;
use esp_idf_hal::adc::config::dma::Config as AdcDmaConfig;
use esp_idf_hal::adc::config::Resolution::*;

extern crate alloc;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let adc1 = peripherals.adc1;
    let pin1 = peripherals.pins.gpio32.into_analog_atten_0db().unwrap();

    let adc = PoweredAdc::new(adc1, Config {
        resolution: Resolution12Bit,
        calibration: true,
    })?;

    let channels: [Box<dyn AttenChannel<_>>; 1] = [
        Box::new(DmaChannel::new(pin1)),
    ];
    let dma_adc_config = AdcDmaConfig::new(
        1000,
        100,
        1024,
        &channels,
    );
    let mut dma_adc = ContinuousADC::new(adc, &dma_adc_config).unwrap();

    loop {
        let mut read_buf: [ChannelData; 12] = unsafe { zeroed() };
        dma_adc.read_buf(&mut read_buf, 10000).unwrap();
        thread::sleep(Duration::from_millis(1000));
    }
}
