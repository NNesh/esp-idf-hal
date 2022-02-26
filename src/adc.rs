use core::marker::PhantomData;
use core::mem::size_of;

#[cfg(not(feature = "riscv-ulp-hal"))]
use esp_idf_sys::*;

#[cfg(feature = "riscv-ulp-hal")]
use crate::riscv_ulp_hal::sys::*;

#[cfg(all(esp32, not(feature = "riscv-ulp-hal")))]
use crate::hall;

pub trait Adc: Send {
    fn unit() -> adc_unit_t;
}

pub trait Analog<ADC: Adc>: Send {
    fn attenuation() -> adc_atten_t;
}

pub struct Atten0dB<ADC: Adc> {
    _adc: PhantomData<ADC>,
}

pub struct Atten2p5dB<ADC: Adc> {
    _adc: PhantomData<ADC>,
}

pub struct Atten6dB<ADC: Adc> {
    _adc: PhantomData<ADC>,
}

pub struct Atten11dB<ADC: Adc> {
    _adc: PhantomData<ADC>,
}

impl<ADC: Adc> Analog<ADC> for Atten0dB<ADC> {
    fn attenuation() -> adc_atten_t {
        adc_atten_t_ADC_ATTEN_DB_0
    }
}

impl<ADC: Adc> Analog<ADC> for Atten2p5dB<ADC> {
    fn attenuation() -> adc_atten_t {
        adc_atten_t_ADC_ATTEN_DB_2_5
    }
}

impl<ADC: Adc> Analog<ADC> for Atten6dB<ADC> {
    fn attenuation() -> adc_atten_t {
        adc_atten_t_ADC_ATTEN_DB_6
    }
}

impl<ADC: Adc> Analog<ADC> for Atten11dB<ADC> {
    fn attenuation() -> adc_atten_t {
        adc_atten_t_ADC_ATTEN_DB_11
    }
}

/// ADC configuration
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod config {
    use esp_idf_sys::*;

    pub mod dma {
        use core::marker::PhantomData;
        use embedded_hal::adc::nb::{Channel};
        use crate::adc::{Adc, Analog};

        pub trait AttenChannel<ADC: Adc>: Channel<ADC, ID = u8> {
            fn attenuation(&self) -> esp_idf_sys::adc_atten_t;
        }
    
        pub struct DmaChannel<ADC, AN, PIN>
        where
            ADC: Adc,
            AN: Analog<ADC>,
            PIN: Channel<AN, ID = u8>
        {
            _adc: PhantomData<ADC>,
            _atten: PhantomData<AN>,
            pub pin: PIN,
        }
    
        impl<ADC, AN, PIN> Channel<ADC> for DmaChannel<ADC, AN, PIN>
        where
            ADC: Adc,
            AN: Analog<ADC>,
            PIN: Channel<AN, ID = u8>
        {
            type ID = u8;
    
            fn channel(&self) -> Self::ID {
                self.pin.channel()
            }
        }

        impl<ADC, AN, PIN> AttenChannel<ADC> for DmaChannel<ADC, AN, PIN>
        where
            ADC: Adc,
            AN: Analog<ADC>,
            PIN: Channel<AN, ID = u8>
        {
            fn attenuation(&self) -> esp_idf_sys::adc_atten_t {
                AN::attenuation()
            }
        }
    
        impl<ADC, AN, PIN> DmaChannel<ADC, AN, PIN>
        where
            ADC: Adc,
            AN: Analog<ADC>,
            PIN: Channel<AN, ID = u8>
        {
            pub fn new(pin: PIN) -> DmaChannel<ADC, AN, PIN> {
                DmaChannel {
                    _adc: PhantomData,
                    _atten: PhantomData,
                    pin,
                }
            }
        }
    
        pub struct AdcDmaConfig<'a, ADC: Adc> {
            sample_rate: u32,
            conv_num: u32,
            max_buffer_size: u32,
            pub channels: &'a [Box<dyn AttenChannel<ADC>>],
        }

        impl<ADC: Adc> AdcDmaConfig<'_, ADC> {
            pub fn new<'a>(sample_rate: u32, conv_num: u32, max_buffer_size: u32, channels: &'a [Box<dyn AttenChannel<ADC>>]) -> AdcDmaConfig<'a, ADC> {
                AdcDmaConfig {
                    sample_rate,
                    channels,
                    conv_num,
                    max_buffer_size,
                }
            }

            #[inline]
            pub fn sample_rate(&self) -> u32 {
                self.sample_rate
            }

            #[inline]
            pub fn conv_num(&self) -> u32 {
                self.conv_num
            }

            #[inline]
            pub fn max_buffer_size(&self) -> u32 {
                self.max_buffer_size
            }
        }
    }

    /// The sampling/readout resolution of the ADC
    #[derive(Debug, PartialEq, Eq, Clone, Copy)]
    pub enum Resolution {
        #[cfg(esp32)]
        Resolution9Bit,
        #[cfg(esp32)]
        Resolution10Bit,
        #[cfg(esp32)]
        Resolution11Bit,
        #[cfg(any(esp32, esp32c3, esp32s3))]
        Resolution12Bit,
        #[cfg(esp32s2)]
        Resolution13Bit,
    }

    impl Default for Resolution {
        #[cfg(any(esp32, esp32c3, esp32s3))]
        fn default() -> Self {
            Self::Resolution12Bit
        }

        #[cfg(esp32s2)]
        fn default() -> Self {
            Self::Resolution13Bit
        }
    }

    impl From<Resolution> for adc_bits_width_t {
        fn from(resolution: Resolution) -> Self {
            match resolution {
                #[cfg(esp32)]
                Resolution::Resolution9Bit => adc_bits_width_t_ADC_WIDTH_BIT_9,
                #[cfg(esp32)]
                Resolution::Resolution10Bit => adc_bits_width_t_ADC_WIDTH_BIT_10,
                #[cfg(esp32)]
                Resolution::Resolution11Bit => adc_bits_width_t_ADC_WIDTH_BIT_11,
                #[cfg(any(esp32, esp32s3, esp32c3))]
                Resolution::Resolution12Bit => adc_bits_width_t_ADC_WIDTH_BIT_12,
                #[cfg(esp32s2)]
                Resolution::Resolution13Bit => adc_bits_width_t_ADC_WIDTH_BIT_13,
            }
        }
    }

    #[derive(Debug, Copy, Clone, Default)]
    pub struct Config {
        pub resolution: Resolution,
        pub calibration: bool,
    }

    impl Config {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn resolution(mut self, resolution: Resolution) -> Self {
            self.resolution = resolution;
            self
        }

        #[must_use]
        pub fn calibration(mut self, calibration: bool) -> Self {
            self.calibration = calibration;
            self
        }
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
pub struct PoweredAdc<ADC: Adc> {
    adc: ADC,
    resolution: config::Resolution,
    cal_characteristics:
        Option<[Option<esp_adc_cal_characteristics_t>; adc_atten_t_ADC_ATTEN_MAX as usize + 1]>,
}

#[cfg(not(feature = "riscv-ulp-hal"))]
unsafe impl<ADC: Adc> Send for PoweredAdc<ADC> {}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl<ADC: Adc> PoweredAdc<ADC> {
    #[cfg(esp32)]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t = esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_VREF;

    #[cfg(any(esp32c3, esp32s2))]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t = esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_TP;

    #[cfg(esp32s3)]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t =
        esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_TP_FIT;

    #[cfg(not(esp32s2))]
    const MAX_READING: u32 = 4095;

    #[cfg(esp32s2)]
    const MAX_READING: u32 = 8191;

    pub fn new(adc: ADC, config: config::Config) -> Result<Self, EspError> {
        if config.calibration {
            esp!(unsafe { esp_adc_cal_check_efuse(Self::CALIBRATION_SCHEME) })?;
        }

        if ADC::unit() == adc_unit_t_ADC_UNIT_1 {
            esp!(unsafe { adc1_config_width(config.resolution.into()) })?;
        }

        Ok(Self {
            adc,
            resolution: config.resolution,
            cal_characteristics: if config.calibration {
                Some(Default::default())
            } else {
                None
            },
        })
    }

    pub fn release(self) -> ADC {
        self.adc
    }

    fn raw_to_voltage(
        &mut self,
        measurement: c_types::c_int,
        attenuation: adc_atten_t,
    ) -> Result<u16, EspError> {
        let mv = if let Some(cal) = self.get_cal_characteristics(attenuation)? {
            unsafe { esp_adc_cal_raw_to_voltage(measurement as u32, &cal as *const _) as u16 }
        } else {
            (measurement as u32 * Self::get_max_mv(attenuation) / Self::MAX_READING) as u16
        };

        Ok(mv)
    }

    #[allow(non_upper_case_globals)]
    fn get_max_mv(attenuation: adc_atten_t) -> u32 {
        #[cfg(esp32)]
        let mv = match attenuation {
            adc_atten_t_ADC_ATTEN_DB_0 => 950,
            adc_atten_t_ADC_ATTEN_DB_2_5 => 1250,
            adc_atten_t_ADC_ATTEN_DB_6 => 1750,
            adc_atten_t_ADC_ATTEN_DB_11 => 2450,
            other => panic!("Unknown attenuation: {}", other),
        };

        #[cfg(any(esp32c3, esp32s2))]
        let mv = match attenuation {
            adc_atten_t_ADC_ATTEN_DB_0 => 750,
            adc_atten_t_ADC_ATTEN_DB_2_5 => 1050,
            adc_atten_t_ADC_ATTEN_DB_6 => 1300,
            adc_atten_t_ADC_ATTEN_DB_11 => 2500,
            other => panic!("Unknown attenuation: {}", other),
        };

        #[cfg(esp32s3)]
        let mv = match attenuation {
            adc_atten_t_ADC_ATTEN_DB_0 => 950,
            adc_atten_t_ADC_ATTEN_DB_2_5 => 1250,
            adc_atten_t_ADC_ATTEN_DB_6 => 1750,
            adc_atten_t_ADC_ATTEN_DB_11 => 3100,
            other => panic!("Unknown attenuation: {}", other),
        };

        mv
    }

    fn get_cal_characteristics(
        &mut self,
        attenuation: adc_atten_t,
    ) -> Result<Option<esp_adc_cal_characteristics_t>, EspError> {
        if let Some(characteristics) = &mut self.cal_characteristics {
            if let Some(cal) = characteristics[attenuation as usize] {
                Ok(Some(cal))
            } else {
                esp!(unsafe { esp_adc_cal_check_efuse(Self::CALIBRATION_SCHEME) })?;

                let mut cal: esp_adc_cal_characteristics_t = Default::default();
                unsafe {
                    esp_adc_cal_characterize(
                        ADC::unit(),
                        attenuation,
                        self.resolution.into(),
                        0,
                        &mut cal as *mut _,
                    )
                };

                characteristics[attenuation as usize] = Some(cal);

                Ok(Some(cal))
            }
        } else {
            Ok(None)
        }
    }

    fn read(
        &mut self,
        unit: adc_unit_t,
        channel: adc_channel_t,
        atten: adc_atten_t,
    ) -> nb::Result<u16, EspError> {
        let mut measurement = 0_i32;

        if unit == adc_unit_t_ADC_UNIT_1 {
            measurement = unsafe { adc1_get_raw(channel) };
        } else {
            let res = unsafe {
                adc2_get_raw(channel, self.resolution.into(), &mut measurement as *mut _)
            };

            if res == ESP_ERR_INVALID_STATE as i32 {
                return Err(nb::Error::WouldBlock);
            } else if res < 0 {
                return Err(nb::Error::Other(EspError::from(res).unwrap()));
            }
        };

        Ok(self.raw_to_voltage(measurement, atten)?)
    }

    #[cfg(esp32)]
    fn read_hall(&mut self) -> nb::Result<u16, EspError> {
        let measurement = unsafe { hall_sensor_read() };

        Ok(self.raw_to_voltage(measurement, adc_atten_t_ADC_ATTEN_DB_0)?)
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl<ADC, AN, PIN> embedded_hal_0_2::adc::OneShot<AN, u16, PIN> for PoweredAdc<ADC>
where
    ADC: Adc,
    AN: Analog<ADC>,
    PIN: embedded_hal_0_2::adc::Channel<AN, ID = u8>,
{
    type Error = EspError;

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<u16, Self::Error> {
        self.read(
            ADC::unit(),
            PIN::channel() as adc_channel_t,
            AN::attenuation(),
        )
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl<ADC, AN, PIN> embedded_hal::adc::nb::OneShot<AN, u16, PIN> for PoweredAdc<ADC>
where
    ADC: Adc,
    AN: Analog<ADC>,
    PIN: embedded_hal::adc::nb::Channel<AN, ID = u8>,
{
    type Error = EspError;

    fn read(&mut self, pin: &mut PIN) -> nb::Result<u16, Self::Error> {
        self.read(
            ADC::unit(),
            pin.channel() as adc_channel_t,
            AN::attenuation(),
        )
    }
}

#[cfg(all(esp32, not(feature = "riscv-ulp-hal")))]
impl embedded_hal_0_2::adc::OneShot<ADC1, u16, hall::HallSensor> for PoweredAdc<ADC1> {
    type Error = EspError;

    fn read(&mut self, _hall_sensor: &mut hall::HallSensor) -> nb::Result<u16, Self::Error> {
        self.read_hall()
    }
}

#[cfg(all(esp32, not(feature = "riscv-ulp-hal")))]
impl embedded_hal::adc::nb::OneShot<ADC1, u16, hall::HallSensor> for PoweredAdc<ADC1> {
    type Error = EspError;

    fn read(&mut self, _hall_sensor: &mut hall::HallSensor) -> nb::Result<u16, Self::Error> {
        self.read_hall()
    }
}

pub struct ContinuousADC<ADC: Adc> {
    adc: Option<PoweredAdc<ADC>>,
    channel_atten: [adc_atten_t; 64],
}


impl<ADC: Adc> ContinuousADC<ADC> {
    const CONV_LIMIT: u32 = 250;

    fn new_internal<'a>(adc: PoweredAdc<ADC>, config: &config::dma::AdcDmaConfig<'a, ADC>) -> nb::Result<Self, EspError> {
        let mut result;

        let mut mask: u32 = 0;
        for channel in config.channels {
            result = unsafe {
                if ADC::unit() == adc_unit_t_ADC_UNIT_1 {
                    adc1_config_channel_atten(channel.channel() as u32, channel.attenuation() as u32)
                } else {
                    adc2_config_channel_atten(channel.channel() as u32, channel.attenuation() as u32)
                }
            };

            if result != ESP_OK {
                return Err(nb::Error::Other(EspError::from(result).unwrap()));
            }

            mask |= 1 << channel.channel();
        }

        let adc_dma_config = adc_digi_init_config_t {
            max_store_buf_size: config.max_buffer_size(),
            conv_num_each_intr: config.conv_num(),
            adc1_chan_mask: if ADC::unit() == adc_unit_t_ADC_UNIT_1 { mask } else { 0 },
            adc2_chan_mask: if ADC::unit() != adc_unit_t_ADC_UNIT_1 { mask } else { 0 },
        };

        result = unsafe { adc_digi_initialize(&adc_dma_config) };
        if result != ESP_OK {
            return Err(nb::Error::Other(EspError::from(result).unwrap()));
        }

        let mut chatten: [adc_atten_t; 64] = unsafe { std::mem::zeroed() };
        let mut pattern_table: [adc_digi_pattern_config_t; 10] = unsafe { std::mem::zeroed() };
        let mut idx: usize = 0;
        for channel in config.channels {
            let ch = channel.channel() as u8;
            let atten = channel.attenuation();
            pattern_table[idx] = adc_digi_pattern_config_t {
                atten: atten as u8,
                channel: ch,
                unit: ADC::unit() as u8 - 1,
                bit_width: SOC_ADC_DIGI_MAX_BITWIDTH as u8,
            };
            chatten[ch as usize] = atten;
            idx += 1;
        }
        
        let dig_cfg = adc_digi_configuration_t {
            conv_limit_en: true,
            conv_limit_num: Self::CONV_LIMIT,
            sample_freq_hz: config.sample_rate(),
            pattern_num: config.channels.len() as u32,
            adc_pattern: pattern_table.as_mut_ptr(),
            conv_mode: 0,
            format: adc_digi_output_format_t_ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        };

        result = unsafe { adc_digi_controller_configure(&dig_cfg) };
        if result != ESP_OK {
            return Err(nb::Error::Other(EspError::from(result).unwrap()));
        }

        Ok(ContinuousADC {
            adc: Some(adc),
            channel_atten: chatten.clone()
        })
    }

    fn release_internal(mut self) -> PoweredAdc<ADC> {
        self.adc.take().unwrap()
    }

    fn deinitialize(&mut self) -> nb::Result<(), EspError> {
        let result = unsafe { adc_digi_deinitialize() };
        if result != ESP_OK {
            return Err(nb::Error::Other(EspError::from(result).unwrap()));
        }
        Ok(())
    }
}

#[cfg(esp32)]
impl ContinuousADC<ADC1> {
    pub fn new<'a>(adc: PoweredAdc<ADC1>, config: &config::dma::AdcDmaConfig<'a, ADC1>) -> nb::Result<Self, EspError> {
        Self::new_internal(adc, config)
    }

    pub fn release(mut self) ->PoweredAdc<ADC1> {
        self.release_internal()
    }
}

#[cfg(not(esp32))]
impl ContinuousADC<ADC> {
    pub fn new<'a>(adc: PoweredAdc<ADC>, config: &config::dma::AdcDmaConfig<'a, ADC>) -> nb::Result<Self, EspError> {
        Self::new_internal(adc, config)
    }

    pub fn release(mut self) ->PoweredAdc<ADC> {
        self.release_internal()
    }
}

impl<ADC: Adc> Drop for ContinuousADC<ADC> {
    fn drop(&mut self) {
        if let Err(_) = self.deinitialize() {
            panic!("Unable to deinitialize DMA for ADC")
        }
    }
}

#[derive(Clone, Copy)]
pub struct ChannelData(adc_digi_output_data_t);

impl ChannelData {
    #[inline(always)]
    fn set_value(&mut self, value: u16) {
        unsafe { self.0.__bindgen_anon_1.type1.set_data(value) }
    }

    #[inline(always)]
    pub fn value(&self) -> u16 {
        unsafe { self.0.__bindgen_anon_1.type1.data() }
    }

    #[inline(always)]
    pub fn channel(&self) -> u16 {
        unsafe { self.0.__bindgen_anon_1.type1.channel() }
    }
}

impl<ADC: Adc> ContinuousADC<ADC> {
    fn read_buf(&mut self, buf: &mut [ChannelData], timeout: u32) -> nb::Result<usize, EspError> {
        let mut result;
        let mut out_len: u32 = 0;

        result = unsafe { adc_digi_start() };
        if result != ESP_OK {
            return Err(nb::Error::Other(EspError::from(result).unwrap()));
        }

        result = unsafe { adc_digi_read_bytes(buf.as_mut_ptr() as *mut u8, (2 * buf.len()) as u32, &mut out_len, timeout) };
        if result != ESP_OK {
            return Err(nb::Error::Other(EspError::from(result).unwrap()));
        }
        
        result = unsafe { adc_digi_stop() };
        if result != ESP_OK {
            return Err(nb::Error::Other(EspError::from(result).unwrap()));
        }

        let adc = self.adc.as_mut().unwrap();
        if let Some(_) = adc.cal_characteristics {
            for item in buf {
                let mv = adc.raw_to_voltage(item.value().into(), self.channel_atten[item.channel() as usize])?;
                item.set_value(mv);
            }
        }

        Ok((out_len / size_of::<ChannelData>() as u32) as usize)
    }
}


macro_rules! impl_adc {
    ($adc:ident: $unit:expr) => {
        pub struct $adc(::core::marker::PhantomData<*const ()>);

        impl $adc {
            /// # Safety
            ///
            /// Care should be taken not to instnatiate this ADC instance, if it is already instantiated and used elsewhere
            pub unsafe fn new() -> Self {
                $adc(::core::marker::PhantomData)
            }
        }

        unsafe impl Send for $adc {}

        impl Adc for $adc {
            #[inline(always)]
            fn unit() -> adc_unit_t {
                $unit
            }
        }
    };
}

impl_adc!(ADC1: adc_unit_t_ADC_UNIT_1);
impl_adc!(ADC2: adc_unit_t_ADC_UNIT_2);
