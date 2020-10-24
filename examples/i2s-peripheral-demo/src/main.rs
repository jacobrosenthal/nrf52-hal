#![no_std]
#![no_main]

// I2S `peripheral mode` demo
// Signal average level indicator using INMP441 and an RGB LED

use {
    core::{
        panic::PanicInfo,
        sync::atomic::{compiler_fence, Ordering},
    },
    hal::{gpio::p0, gpio::Level, gpio::Output, gpio::PushPull, i2s::*},
    nrf52840_hal as hal,
    nrf52840_hal::prelude::OutputPin,
    rtt_target::{rprintln, rtt_init_print},
};

#[repr(align(4))]
struct Aligned<T: ?Sized>(T);

#[rtic::app(device = crate::hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        g_led: p0::P0_22<Output<PushPull>>,
        r_led: p0::P0_23<Output<PushPull>>,
        b_led: p0::P0_24<Output<PushPull>>,
        transfer: Option<Transfer<&'static mut [i16; 128]>>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        // The I2S buffer address must be 4 byte aligned.
        static mut RX_BUF: Aligned<[i16; 128]> = Aligned([0; 128]);

        let _clocks = hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();
        rtt_init_print!();
        rprintln!("Play me some audio...");

        let p0 = hal::gpio::p0::Parts::new(ctx.device.P0);
        let sck_pin = p0.p0_21.into_floating_input().degrade();
        let lrck_pin = p0.p0_17.into_floating_input().degrade();
        let sdin_pin = p0.p0_25.into_floating_input().degrade();

        let i2s = I2S::new_peripheral(
            ctx.device.I2S,
            None,
            &sck_pin,
            &lrck_pin,
            Some(&sdin_pin),
            None,
        );
        i2s.enable_interrupt(I2SEvent::RxPtrUpdated).start();

        // Configure RGB LED control
        let g_led = p0.p0_22.into_push_pull_output(Level::High);
        let r_led = p0.p0_23.into_push_pull_output(Level::High);
        let b_led = p0.p0_24.into_push_pull_output(Level::High);

        init::LateResources {
            r_led,
            g_led,
            b_led,
            transfer: i2s.rx(&mut RX_BUF.0).ok(),
        }
    }

    #[task(binds = I2S, resources = [r_led, g_led, b_led, transfer])]
    fn on_i2s(ctx: on_i2s::Context) {
        let (rx_buf, i2s) = ctx.resources.transfer.take().unwrap().wait();
        let r_led = ctx.resources.r_led;
        let g_led = ctx.resources.g_led;
        let b_led = ctx.resources.b_led;

        if i2s.is_event_triggered(I2SEvent::RxPtrUpdated) {
            i2s.reset_event(I2SEvent::RxPtrUpdated);

            // Calculate mono summed average of received buffer
            let avg = (rx_buf.iter().map(|x| (*x).abs() as u32).sum::<u32>() / rx_buf.len() as u32)
                as u16;
            match avg {
                0..=4 => {
                    let _ = g_led.set_high();
                    let _ = b_led.set_high();
                    let _ = r_led.set_high();
                }
                5..=10_337 => {
                    let _ = g_led.set_low();
                    let _ = b_led.set_high();
                    let _ = r_led.set_high();
                }
                10_338..=16_383 => {
                    let _ = g_led.set_high();
                    let _ = b_led.set_low();
                    let _ = r_led.set_high();
                }
                _ => {
                    let _ = g_led.set_high();
                    let _ = b_led.set_high();
                    let _ = r_led.set_low();
                }
            };
        }
        *ctx.resources.transfer = i2s.rx(rx_buf).ok();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    rprintln!("{}", info);
    loop {
        compiler_fence(Ordering::SeqCst);
    }
}
