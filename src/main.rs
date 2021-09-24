#![no_std]
#![no_main]
#![feature(panic_info_message)]

use rtic::app;
use stm32f1xx_hal::gpio::gpioc::PC13;
use stm32f1xx_hal::pac::TIM2;
use stm32f1xx_hal::timer::CountDownTimer;
use usb_device::bus;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::time::U32Ext;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::rcc::RccExt;
use stm32f1xx_hal::gpio::{GpioExt, Output, PushPull};

use embedded_hal::digital::v2::OutputPin;

use cortex_m::asm;
use cortex_m_semihosting::hprintln;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use core::panic::PanicInfo;

use crate::adc::Adc1;

mod adc;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {

	match (info.location(), info.message()) {
		(None, None) => hprintln!("program panicked").ok(),
		(Some(location), Some(message)) => hprintln!("program panicked at {}:{}: {}", location.file(), location.line(), message).ok(),
		(None, Some(message)) => hprintln!("program panicked: {}", message).ok(),
		(Some(location), None) => hprintln!("program panicked at {}:{}", location.file(), location.line()).ok(),
	};

	let gpioc = unsafe { &*stm32f1xx_hal::device::GPIOC::ptr() };

	loop {
		gpioc.bsrr.write(|w| {
			w.bs13().set()
		});
		asm::delay(4_000_000);
		gpioc.bsrr.write(|w| {
			w.br13().reset()
		});
		asm::delay(4_000_000);
	}
}

#[app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
	struct Resources {
		usb_dev: UsbDevice<'static, UsbBusType>,
		serial: SerialPort<'static, UsbBusType>,
		led: PC13<Output<PushPull>>,
		timer: CountDownTimer<TIM2>,
		adc1: Adc1,
	}

	#[init]
	fn init(cx: init::Context) -> init::LateResources {
		static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

		// Set the clock speed.
		let mut flash = cx.device.FLASH.constrain();
		let mut rcc = cx.device.RCC.constrain();
		let mut gpioc = cx.device.GPIOC.split(&mut rcc.apb2);

		let clocks = rcc
			.cfgr
			.use_hse(8.mhz())
			.sysclk(72.mhz())
			.pclk1(36.mhz())
			.pclk2(72.mhz())
			.adcclk(12.mhz())
			.freeze(&mut flash.acr);
		assert!(clocks.usbclk_valid());

		// Start the timer.
		let mut timer = stm32f1xx_hal::timer::Timer::tim2(cx.device.TIM2, &clocks, &mut rcc.apb1)
			.start_count_down(10.hz());
		timer.listen(stm32f1xx_hal::timer::Event::Update);

		// Turn on the LED.
		let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
		led.set_low().ok();

		// Enable ADC.
		let adc1 = Adc1::init(cx.device.ADC1, &mut rcc.apb2, &clocks);
		adc1.set_channel(adc::Channel1);
		adc1.set_sample_time(adc::Cycles239_5);

		// BluePill board has a pull-up resistor on the D+ line.
		// Pull the D+ pin down to send a RESET condition to the USB bus.
		// This forced reset is needed only for development, without it host
		// will not reset your device when you upload new firmware.
		let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);
		let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
		usb_dp.set_low().unwrap();
		asm::delay(clocks.sysclk().0 / 100);

		let usb_dm = gpioa.pa11;
		let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

		// Create the USB bus.
		let usb = Peripheral {
			usb: cx.device.USB,
			pin_dm: usb_dm,
			pin_dp: usb_dp,
		};

		*USB_BUS = Some(UsbBus::new(usb));

		// Start a serial communication.
		let serial = SerialPort::new(USB_BUS.as_ref().unwrap());
		let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
			.manufacturer("The Robot Engineers")
			.product("USB ADC")
			.serial_number("TEST")
			.device_class(USB_CLASS_CDC)
			.build();

		init::LateResources { usb_dev, serial, led, timer, adc1 }
	}

	#[task(binds = ADC1_2, resources = [serial, adc1, led])]
	fn adc1_2(cx: adc1_2::Context) {
		if let Some(data) = cx.resources.adc1.read() {
			cx.resources.serial.write(&data.to_le_bytes()).unwrap();
			cx.resources.led.set_high().ok();
		}
	}

	#[task(binds = TIM2, resources = [serial, led, adc1, timer])]
	fn timer2(cx: timer2::Context) {
		cx.resources.adc1.start_convert();
		cx.resources.led.set_low().ok();
		cx.resources.timer.clear_update_interrupt_flag();
	}

	#[task(binds = USB_HP_CAN_TX, resources = [usb_dev, serial, led])]
	fn usb_hp(cx: usb_hp::Context) {
		let mut buffer = [0; 128];
		while cx.resources.serial.read(&mut buffer).is_ok() {}
		cx.resources.usb_dev.poll(&mut [cx.resources.serial]);
	}

	#[task(binds = USB_LP_CAN_RX0, resources = [usb_dev, serial, led])]
	fn usb_lp(cx: usb_lp::Context) {
		let mut buffer = [0; 128];
		while cx.resources.serial.read(&mut buffer).is_ok() {}
		cx.resources.usb_dev.poll(&mut [cx.resources.serial]);
	}
};
