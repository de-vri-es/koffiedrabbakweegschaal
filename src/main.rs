#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m_rt::entry;
use cortex_m::asm;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
	loop {}
}

#[entry]
fn main() -> ! {
	let peripherals = stm32f1::stm32f103::Peripherals::take().unwrap();

	peripherals.FLASH.acr.write(|w| {
		w
			.prftbe().set_bit()
			.hlfcya().clear_bit()
			.latency().ws2()
	});

	loop {
		if peripherals.FLASH.acr.read().latency().is_ws2() {
			break;
		}
	}

	// Configure PLL and APB1 prescaler for 8 MHz crystal.
	// Set PLL  = 8 (crystal) * 9   = 72 MHz
	// Set APB1 = 72 (sysclk) / 2   = 36 MHz
	// Set USB  = 72 (sysclk) / 1.5 = 48 MHz
	peripherals.RCC.cfgr.write(|w| {
		w
			.pllsrc().hse_div_prediv()
			.pllmul().mul9()
			.ppre1().div2()
			.usbpre().div1_5()
	});

	// Enable HSE (crystal), PLL and clock security.
	peripherals.RCC.cr.write(|w| {
		w
			.csson().on()
			.hseon().on()
			.pllon().on()
	});

	// Wait for PLL to become ready.
	loop {
		if peripherals.RCC.cr.read().pllrdy().is_ready() {
			break;
		}
	}

	peripherals.RCC.cfgr.modify(|_, w| {
		w
			.sw().pll()
	});

	// Wait for switch to complete.
	loop {
		if peripherals.RCC.cfgr.read().sws().is_pll() {
			break;
		}
	}

	peripherals.RCC.apb2enr.modify(|_, w| {
		w.iopcen().enabled()
	});
	peripherals.GPIOC.crh.modify(|_, w| {
		w
			.cnf13().open_drain()
			.mode13().output50()
	});

	loop {
		peripherals.GPIOC.bsrr.write(|w| {
			w.bs13().set()
		});
		asm::delay(4_000_000);
		peripherals.GPIOC.bsrr.write(|w| {
			w.br13().reset()
		});
		asm::delay(1_000_000);
	}
}
