use stm32f1xx_hal::{pac::ADC1, rcc::{APB2, Enable, Reset}, rcc::Clocks};
use cortex_m::asm;

pub struct Adc1 {
	adc1: ADC1,
}

#[repr(u8)]
pub enum Channel {
	Channel0  = 0,
	Channel1  = 1,
	Channel2  = 2,
	Channel3  = 3,
	Channel4  = 4,
	Channel5  = 5,
	Channel6  = 6,
	Channel7  = 7,
	Channel8  = 8,
	Channel9  = 9,
	Channel10 = 10,
	Channel11 = 11,
	Channel12 = 12,
	Channel13 = 13,
	Channel14 = 14,
	Channel15 = 15,
}
pub use Channel::*;

#[repr(u8)]
pub enum SampleTime {
	Cycles1_5   = 0b000,
	Cycles7_5   = 0b001,
	Cycles13_5  = 0b010,
	Cycles28_5  = 0b011,
	Cycles41_5  = 0b100,
	Cycles55_5  = 0b101,
	Cycles71_5  = 0b110,
	Cycles239_5 = 0b111,
}
pub use SampleTime::*;

impl Adc1 {
	pub fn init(adc1: ADC1, apb2: &mut APB2, clocks: &Clocks) -> Self {
		ADC1::enable(apb2);
		ADC1::reset(apb2);

		adc1.cr1.modify(|_, w|
			w
				.eocie().enabled()
		);

		adc1.cr2.modify(|_, w|
			w
				.cont().single()
				.exttrig().enabled()
				.extsel().swstart()
				.adon().enabled()
		);

		asm::delay((clocks.sysclk().0 + clocks.adcclk().0 - 1) / clocks.adcclk().0 * 2);

		let adc = Self { adc1 };
		adc.calibrate();

		adc.adc1.cr2.modify(|_, w|
			w
				.adon().enabled()
		);

		adc.set_channel(Channel0);
		adc
	}

	pub fn set_sample_time(&self, sample_time: SampleTime) {
		self.adc1.smpr2.write(|w| w .smp0().bits(sample_time as u8));
	}

	pub fn set_channel(&self, channel: Channel) {
		self.adc1.sqr1.write(|w| w.l().bits(0)); // 0000: 1 conversions
		self.adc1.sqr3.write(|w|
			unsafe { w.sq1().bits(channel as u8) }
		);
	}

	pub fn start_convert(&self) {
		self.adc1.cr2.modify(|_, w| w .swstart().start());
	}

	/// Read the data register and clear the interrupt flag.
	///
	/// If no data was available, None is returned and the interrupt flag is not cleared.
	pub fn read(&self) -> Option<u16> {
		if self.adc1.sr.read().eoc().is_not_complete() {
			None
		} else {
			Some(self.adc1.dr.read().data().bits())
		}
	}

	fn calibrate(&self) {
		// Reset calibration.
		self.adc1.cr2.modify(|_, w| w.rstcal().set_bit());
		while self.adc1.cr2.read().rstcal().bit_is_set() {}

		// Calibrate.
		self.adc1.cr2.modify(|_, w| w.cal().set_bit());
		while self.adc1.cr2.read().cal().bit_is_set() {}
	}
}
