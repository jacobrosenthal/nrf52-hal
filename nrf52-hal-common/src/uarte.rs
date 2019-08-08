//! HAL interface to the UARTE peripheral
//!
//! See product specification:
//!
//! - nrf52832: Section 35
//! - nrf52840: Section 6.34
use core::fmt;
use core::ops::Deref;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};

use crate::target::{uarte0, UARTE0};

use crate::gpio::{Floating, Input, Output, Pin, PushPull};
use crate::prelude::*;
use crate::target_constants::EASY_DMA_SIZE;

use crate::timer::{self, Timer};
use embedded_hal::timer::Cancel;

use heapless::{
    consts::*,
    pool,
    pool::singleton::{Box, Pool},
    spsc::{Consumer, Queue},
    ArrayLength,
};

// Re-export SVD variants to allow user to directly set values
pub use crate::target::uarte0::{baudrate::BAUDRATEW as Baudrate, config::PARITYW as Parity};

/// Interface to a UARTE instance
///
/// This is a very basic interface that comes with the following limitations:
/// - The UARTE instances share the same address space with instances of UART.
///   You need to make sure that conflicting instances
///   are disabled before using `Uarte`. See product specification:
///     - nrf52832: Section 15.2
///     - nrf52840: Section 6.1.2
pub struct Uarte<T>(T);

impl<T> Uarte<T>
where
    T: Instance,
{
    pub fn new(uarte: T, mut pins: Pins, parity: Parity, baudrate: Baudrate) -> Self {
        // Select pins
        uarte.psel.rxd.write(|w| {
            let w = unsafe { w.pin().bits(pins.rxd.pin) };
            #[cfg(feature = "52840")]
            let w = w.port().bit(pins.rxd.port);
            w.connect().connected()
        });
        pins.txd.set_high();
        uarte.psel.txd.write(|w| {
            let w = unsafe { w.pin().bits(pins.txd.pin) };
            #[cfg(feature = "52840")]
            let w = w.port().bit(pins.txd.port);
            w.connect().connected()
        });

        // Optional pins
        uarte.psel.cts.write(|w| {
            if let Some(ref pin) = pins.cts {
                let w = unsafe { w.pin().bits(pin.pin) };
                #[cfg(feature = "52840")]
                let w = w.port().bit(pin.port);
                w.connect().connected()
            } else {
                w.connect().disconnected()
            }
        });

        uarte.psel.rts.write(|w| {
            if let Some(ref pin) = pins.rts {
                let w = unsafe { w.pin().bits(pin.pin) };
                #[cfg(feature = "52840")]
                let w = w.port().bit(pin.port);
                w.connect().connected()
            } else {
                w.connect().disconnected()
            }
        });

        // Enable UARTE instance
        uarte.enable.write(|w| w.enable().enabled());

        // Configure
        let hardware_flow_control = pins.rts.is_some() && pins.cts.is_some();
        uarte
            .config
            .write(|w| w.hwfc().bit(hardware_flow_control).parity().variant(parity));

        // Configure frequency
        uarte.baudrate.write(|w| w.baudrate().variant(baudrate));

        Uarte(uarte)
    }

    /// Write via UARTE
    ///
    /// This method uses transmits all bytes in `tx_buffer`
    ///
    /// The buffer must have a length of at most 255 bytes on the nRF52832
    /// and at most 65535 bytes on the nRF52840.
    pub fn write(&mut self, tx_buffer: &[u8]) -> Result<(), Error> {
        if tx_buffer.len() > EASY_DMA_SIZE {
            return Err(Error::TxBufferTooLong);
        }

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // before any DMA action has started
        compiler_fence(SeqCst);

        // Set up the DMA write
        self.0.txd.ptr.write(|w|
            // We're giving the register a pointer to the stack. Since we're
            // waiting for the UARTE transaction to end before this stack pointer
            // becomes invalid, there's nothing wrong here.
            //
            // The PTR field is a full 32 bits wide and accepts the full range
            // of values.
            unsafe { w.ptr().bits(tx_buffer.as_ptr() as u32) });
        self.0.txd.maxcnt.write(|w|
            // We're giving it the length of the buffer, so no danger of
            // accessing invalid memory. We have verified that the length of the
            // buffer fits in an `u8`, so the cast to `u8` is also fine.
            //
            // The MAXCNT field is 8 bits wide and accepts the full range of
            // values.
            unsafe { w.maxcnt().bits(tx_buffer.len() as _) });

        // Start UARTE Transmit transaction
        self.0.tasks_starttx.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });

        // Wait for transmission to end
        while self.0.events_endtx.read().bits() == 0 {}

        // Reset the event, otherwise it will always read `1` from now on.
        self.0.events_endtx.write(|w| w);

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // after all possible DMA actions have completed
        compiler_fence(SeqCst);

        if self.0.txd.amount.read().bits() != tx_buffer.len() as u32 {
            return Err(Error::Transmit);
        }

        Ok(())
    }

    /// Read via UARTE
    ///
    /// This method fills all bytes in `rx_buffer`, and blocks
    /// until the buffer is full.
    ///
    /// The buffer must have a length of at most 255 bytes
    pub fn read(&mut self, rx_buffer: &mut [u8]) -> Result<(), Error> {
        self.start_read(rx_buffer)?;

        // Wait for transmission to end
        while self.0.events_endrx.read().bits() == 0 {}

        self.finalize_read();

        if self.0.rxd.amount.read().bits() != rx_buffer.len() as u32 {
            return Err(Error::Receive);
        }

        Ok(())
    }

    /// Read via UARTE
    ///
    /// This method fills all bytes in `rx_buffer`, and blocks
    /// until the buffer is full or the timeout expires, whichever
    /// comes first.
    ///
    /// If the timeout occurs, an `Error::Timeout(n)` will be returned,
    /// where `n` is the number of bytes read successfully.
    ///
    /// This method assumes the interrupt for the given timer is NOT enabled,
    /// and in cases where a timeout does NOT occur, the timer will be left running
    /// until completion.
    ///
    /// The buffer must have a length of at most 255 bytes
    pub fn read_timeout<I>(
        &mut self,
        rx_buffer: &mut [u8],
        timer: &mut Timer<I>,
        cycles: u32,
    ) -> Result<(), Error>
    where
        I: timer::Instance,
    {
        // Start the read
        self.start_read(rx_buffer)?;

        // Start the timeout timer
        timer.start(cycles);

        // Wait for transmission to end
        let mut event_complete = false;
        let mut timeout_occured = false;

        loop {
            event_complete |= self.0.events_endrx.read().bits() != 0;
            timeout_occured |= timer.wait().is_ok();
            if event_complete || timeout_occured {
                break;
            }
        }

        if !event_complete {
            // Cancel the reception if it did not complete until now
            self.cancel_read();
        }

        // Cleanup, even in the error case
        self.finalize_read();

        let bytes_read = self.0.rxd.amount.read().bits() as usize;

        if timeout_occured && !event_complete {
            return Err(Error::Timeout(bytes_read));
        }

        if bytes_read != rx_buffer.len() as usize {
            return Err(Error::Receive);
        }

        Ok(())
    }

    /// Start a UARTE read transaction by setting the control
    /// values and triggering a read task
    fn start_read(&mut self, rx_buffer: &mut [u8]) -> Result<(), Error> {
        // This is overly restrictive. See (similar SPIM issue):
        // https://github.com/nrf-rs/nrf52/issues/17
        if rx_buffer.len() > u8::max_value() as usize {
            return Err(Error::TxBufferTooLong);
        }

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // before any DMA action has started
        compiler_fence(SeqCst);

        // Set up the DMA read
        self.0.rxd.ptr.write(|w|
            // We're giving the register a pointer to the stack. Since we're
            // waiting for the UARTE transaction to end before this stack pointer
            // becomes invalid, there's nothing wrong here.
            //
            // The PTR field is a full 32 bits wide and accepts the full range
            // of values.
            unsafe { w.ptr().bits(rx_buffer.as_ptr() as u32) });
        self.0.rxd.maxcnt.write(|w|
            // We're giving it the length of the buffer, so no danger of
            // accessing invalid memory. We have verified that the length of the
            // buffer fits in an `u8`, so the cast to `u8` is also fine.
            //
            // The MAXCNT field is at least 8 bits wide and accepts the full
            // range of values.
            unsafe { w.maxcnt().bits(rx_buffer.len() as _) });

        // Start UARTE Receive transaction
        self.0.tasks_startrx.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });

        Ok(())
    }

    /// Finalize a UARTE read transaction by clearing the event
    fn finalize_read(&mut self) {
        // Reset the event, otherwise it will always read `1` from now on.
        self.0.events_endrx.write(|w| w);

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // after all possible DMA actions have completed
        compiler_fence(SeqCst);
    }

    /// Stop an unfinished UART read transaction and flush FIFO to DMA buffer
    fn cancel_read(&mut self) {
        // Stop reception
        self.0.tasks_stoprx.write(|w| unsafe { w.bits(1) });

        // Wait for the reception to have stopped
        while self.0.events_rxto.read().bits() == 0 {}

        // Reset the event flag
        self.0.events_rxto.write(|w| w);

        // Ask UART to flush FIFO to DMA buffer
        self.0.tasks_flushrx.write(|w| unsafe { w.bits(1) });

        // Wait for the flush to complete.
        while self.0.events_endrx.read().bits() == 0 {}

        // The event flag itself is later reset by `finalize_read`.
    }

    /// Return the raw interface to the underlying UARTE peripheral
    pub fn free(self) -> T {
        self.0
    }

    /// Splits the UARTE into a transmitter and receiver for interrupt driven use
    ///
    /// In needs a `Queue` to place received DMA chunks, and a `Consumer` to place DMA chunks to
    /// send
    ///
    /// Note: The act of splitting might not be needed on the nRF52 chips, as they map to the same
    /// interrupt in the end. Kept as a split for now, but might be merged in the future.
    pub fn split<S, I>(
        self,
        txc: Consumer<'static, Box<UARTEDMAPool>, S>,
        timer: Timer<I>,
        dma_pool_memory: &'static mut [u8],
    ) -> (UarteRX<T, I>, UarteTX<T, S>)
    where
        S: ArrayLength<heapless::pool::singleton::Box<UARTEDMAPool>>,
        I: timer::Instance,
    {
        debug_assert!(
            dma_pool_memory.len() >= core::mem::size_of::<UARTEDMAPoolNode>() * 2,
            "The memory pool needs at least space for 2 `UARTEDMAPoolNode`s"
        );
        UARTEDMAPool::grow(dma_pool_memory);

        let mut rx = UarteRX::<T, I>::new(timer);
        rx.enable_interrupts();
        rx.prepare_read().unwrap();
        rx.start_read();

        let tx = UarteTX::<T, S>::new(txc);
        tx.enable_interrupts();
        (rx, tx)
    }
}

/// DMA block size
/// Defaults to UARTE_DMA_SIZE = 16 if not explicitly set
#[cfg(not(any(
    feature = "UARTE_DMA_SIZE_4",
    feature = "UARTE_DMA_SIZE_8",
    feature = "UARTE_DMA_SIZE_16",
    feature = "UARTE_DMA_SIZE_32",
    feature = "UARTE_DMA_SIZE_64",
    feature = "UARTE_DMA_SIZE_128",
    feature = "UARTE_DMA_SIZE_256"
)))]
pub const UARTE_DMA_SIZE: usize = 16;

#[cfg(feature = "UARTE_DMA_SIZE_4")]
pub const UARTE_DMA_SIZE: usize = 4;

#[cfg(feature = "UARTE_DMA_SIZE_8")]
pub const UARTE_DMA_SIZE: usize = 8;

#[cfg(feature = "UARTE_DMA_SIZE_16")]
pub const UARTE_DMA_SIZE: usize = 16;

#[cfg(feature = "UARTE_DMA_SIZE_32")]
pub const UARTE_DMA_SIZE: usize = 32;

#[cfg(feature = "UARTE_DMA_SIZE_64")]
pub const UARTE_DMA_SIZE: usize = 64;

#[cfg(feature = "UARTE_DMA_SIZE_128")]
pub const UARTE_DMA_SIZE: usize = 128;

// Currently causes internal OOM, needs fixing
// Maximum DMA size is 255 of the UARTE peripheral, see `MAXCNT` for details
#[cfg(feature = "UARTE_DMA_SIZE_255")]
pub const UARTE_DMA_SIZE: usize = 255;

// An alternative solution to the above is to define the UARTE_DMA_SIZE
// in a separate (default) crate, which can be overridden
// by a patch in the user Cargo.toml, pointing to
// a local crate with the user defined UARTE_DMA_SIZE constant.
// What would you prefer?

// The DMA implementation shuld be hidden behind a feature POOL
// This likely requires a mod {} around the related code
// or the non-ergonomic repetition of the gate.
// Is there a better solution?
//
// The reason to have a POOL gate is that we don't want the POOL
// to cause memory OH if not used by the application
#[derive(Debug)]
pub struct UARTEDMAPoolNode {
    len: u8,
    buf: [u8; UARTE_DMA_SIZE],
}

impl UARTEDMAPoolNode {
    pub fn new() -> Self {
        Self {
            len: 0,
            buf: [0; UARTE_DMA_SIZE],
        }
    }

    pub fn write(&mut self, buf: &[u8]) -> usize {
        if buf.len() > UARTE_DMA_SIZE {
            self.len = UARTE_DMA_SIZE as u8;
        } else {
            self.len = buf.len() as u8;
        }

        let internal_buffer = &mut self.buf[..self.len as usize];
        internal_buffer.copy_from_slice(&buf[..self.len as usize]);

        return self.len as usize;
    }

    pub fn read(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }

    pub fn len(&self) -> usize {
        self.len as usize
    }

    fn set_len(&mut self, len: u8) {
        self.len = len;
    }

    fn max_len(&self) -> usize {
        UARTE_DMA_SIZE
    }

    fn buffer_address(&self) -> u32 {
        self.buf.as_ptr() as u32
    }
}

pool!(
    #[allow(non_upper_case_globals)]
    UARTEDMAPool: UARTEDMAPoolNode
);

/// UARTE RX part, used in interrupt driven contexts
pub struct UarteRX<T, I> {
    rxq: Queue<Box<UARTEDMAPool>, U2>, // double buffering of DMA chunks
    timer: Timer<I>,                   // Timer for handling timeouts
    _marker: core::marker::PhantomData<T>,
}

/// Receive error in interrupt driven context
#[derive(Debug)]
pub enum RXError {
    /// Out of memory error, global pool is depleted
    ///
    /// Potential causes:
    /// 1. User code is saving the `Box<UARTEDMAPool>` (and not dropping them)
    /// 2. User code running `mem::forget` on the `Box<UARTEDMAPool>`
    /// 3. The pool is too small for the use case
    OOM,
}

impl<T, I> UarteRX<T, I>
where
    T: Instance,
    I: timer::Instance,
{
    /// Construct new UARTE RX, hidden from users - used internally
    fn new(timer: Timer<I>) -> Self {
        Self {
            rxq: Queue::new(),
            timer,
            _marker: core::marker::PhantomData,
        }
    }

    /// Used internally to set up the proper interrupts
    fn enable_interrupts(&mut self) {
        // This operation is safe due to type-state programming guaranteeing that the RX and TX are
        // unique within the driver
        let uarte = unsafe { &*T::ptr() };

        uarte.inten.modify(|_, w| {
            w.endrx()
                .set_bit()
                .rxstarted()
                .set_bit()
                .rxto()
                .set_bit()
                .rxdrdy()
                .set_bit()
        });

        self.timer.enable_interrupt_generation();
    }

    /// Start a UARTE read transaction
    fn start_read(&mut self) {
        // This operation is safe due to type-state programming guaranteeing that the RX and TX are
        // unique within the driver
        let uarte = unsafe { &*T::ptr() };

        // Start UARTE Receive transaction
        uarte.tasks_startrx.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });
    }

    /// Prepare UARTE read transaction
    fn prepare_read(&mut self) -> Result<(), RXError> {
        // This operation is safe due to type-state programming guaranteeing that the RX and TX are
        // unique within the driver
        let uarte = unsafe { &*T::ptr() };

        let b = UARTEDMAPool::alloc()
            .ok_or(RXError::OOM)?
            .init(UARTEDMAPoolNode::new()); // TODO: Find a way to not need new, the zeroing of the
                                            // internal buffer is unnecessary

        compiler_fence(SeqCst);

        // setup start address
        uarte
            .rxd
            .ptr
            .write(|w| unsafe { w.ptr().bits(b.buffer_address()) });
        // setup length
        uarte
            .rxd
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(b.max_len() as _) });

        if self.rxq.enqueue(b).is_err() {
            panic!("Internal driver error, RX Queue Overflow");
        } else {
            Ok(())
        }
    }

    /// Timeout handling for the RX driver - Must be called from the corresponding TIMERx Interrupt
    /// handler/task.
    pub fn process_timeout_interrupt(&mut self) {
        // This operation is safe due to type-state programming guaranteeing that the RX and TX are
        // unique within the driver
        let uarte = unsafe { &*T::ptr() };

        // Reset the event, otherwise it will always read `1` from now on.
        self.timer.clear_interrupt();

        // Stop UARTE Receive transaction to generate the Timeout Event
        uarte.tasks_stoprx.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });
    }

    /// Main entry point for the RX driver - Must be called from the corresponding UARTE Interrupt
    /// handler/task.
    ///
    /// Will return:
    /// 1. `Ok(Some(Box<UARTEDMAPool>))` if data was received
    /// 2. `Ok(None)` if there was no data, i.e. UARTE interrupt was due to other events
    /// 3. `Err(RXError::OOM)` if the memory pool was depleted, see `RXError` for mitigations
    pub fn process_interrupt(&mut self) -> Result<Option<Box<UARTEDMAPool>>, RXError> {
        // This operation is safe due to type-state programming guaranteeing that the RX and TX are
        // unique within the driver
        let uarte = unsafe { &*T::ptr() };

        // Handles the byte timeout timer
        if uarte.events_rxdrdy.read().bits() == 1 {
            self.timer.cancel().unwrap(); // Never fails
            self.timer.start(10_000_u32); // 10 ms timeout for now

            // Reset the event, otherwise it will always read `1` from now on.
            uarte.events_rxdrdy.write(|w| w);
        }

        if uarte.events_rxto.read().bits() == 1 {
            // Tell UARTE to flush FIFO to DMA buffer
            uarte.tasks_flushrx.write(|w| unsafe { w.bits(1) });

            // Reset the event, otherwise it will always read `1` from now on.
            uarte.events_rxto.write(|w| w);
        }

        // check if dma rx transaction has started
        if uarte.events_rxstarted.read().bits() == 1 {
            // DMA transaction has started
            self.prepare_read()?;

            // Reset the event, otherwise it will always read `1` from now on.
            uarte.events_rxstarted.write(|w| w);
        }

        // check if dma transaction finished
        if uarte.events_endrx.read().bits() == 1 {
            self.timer.cancel().unwrap(); // Never fails

            // our transaction has finished
            if let Some(mut ret_b) = self.rxq.dequeue() {
                // Read the true number of bytes and set the correct length of the packet before
                // returning it
                let bytes_read = uarte.rxd.amount.read().bits() as u8;
                ret_b.set_len(bytes_read);

                // Reset the event, otherwise it will always read `1` from now on.
                uarte.events_endrx.write(|w| w);

                self.start_read();

                return Ok(Some(ret_b)); // ok to return, rx started will be caught later
            } else {
                panic!("Internal driver error, RX Queue Underflow");
            }
        }

        // the interrupt was not RXSTARTED or ENDRX, so no action
        Ok(None)
    }
}

/// UARTE TX part, used in interrupt driven contexts
/// S is the queue length, can be U3, U4 etc.
pub struct UarteTX<T, S>
where
    S: ArrayLength<heapless::pool::singleton::Box<UARTEDMAPool>>,
{
    txc: Consumer<'static, Box<UARTEDMAPool>, S>, // chunks to transmit
    current: Option<Box<UARTEDMAPool>>,
    _marker: core::marker::PhantomData<T>,
}

impl<T, S> UarteTX<T, S>
where
    T: Instance,
    S: ArrayLength<heapless::pool::singleton::Box<UARTEDMAPool>>,
{
    /// Construct new UARTE TX, hidden from users - used internally
    fn new(txc: Consumer<'static, Box<UARTEDMAPool>, S>) -> Self {
        Self {
            txc,
            current: None,
            _marker: core::marker::PhantomData,
        }
    }

    /// Used internally to set up the proper interrupts
    fn enable_interrupts(&self) {
        // This operation is safe due to type-state programming guaranteeing that the RX and TX are
        // unique within the driver
        let uarte = unsafe { &*T::ptr() };

        uarte.inten.modify(|_, w| w.endtx().set_bit());
    }

    /// Sets up the UARTE to send DMA chunk
    fn start_write(&mut self, b: Box<UARTEDMAPool>) {
        // This operation is safe due to type-state programming guaranteeing that the RX and TX are
        // unique within the driver
        let uarte = unsafe { &*T::ptr() };

        // Only send if the DMA chunk has data, else drop the `Box`
        if b.len() > 0 {
            compiler_fence(SeqCst);

            // setup start address
            uarte
                .txd
                .ptr
                .write(|w| unsafe { w.ptr().bits(b.buffer_address()) });

            // setup length
            uarte
                .txd
                .maxcnt
                .write(|w| unsafe { w.maxcnt().bits(b.len() as _) });

            // Start UARTE transmit transaction
            uarte.tasks_starttx.write(|w| unsafe { w.bits(1) });
            self.current = Some(b); // drops the previous current package
        }
    }

    /// Main entry point for the TX driver - Must be called from the corresponding UARTE Interrupt
    /// handler/task.
    pub fn process_interrupt(&mut self) {
        // This operation is safe due to type-state programming guaranteeing that the RX and TX are
        // unique within the driver
        let uarte = unsafe { &*T::ptr() };

        // ENDTX event? (DMA transaction finished)
        if uarte.events_endtx.read().bits() == 1 {
            // our transaction has finished
            match self.txc.dequeue() {
                None => {
                    // a ENDTX without an started transaction is an error
                    if self.current.is_none() {
                        panic!("Internal error, ENDTX without current transaction.")
                    }
                    // we don't have any more to send, so drop the current buffer
                    self.current = None;
                }
                Some(b) => {
                    self.start_write(b);
                }
            }

            // Reset the event, otherwise it will always read `1` from now on.
            uarte.events_endtx.write(|w| w);
        } else {
            if self.current.is_none() {
                match self.txc.dequeue() {
                    Some(b) =>
                    // we were idle, so start a new transaction
                    {
                        self.start_write(b)
                    }
                    None => (),
                }
            }
        }
    }
}

impl<T> fmt::Write for Uarte<T>
where
    T: Instance,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        // Copy all data into an on-stack buffer so we never try to EasyDMA from
        // flash
        let buf = &mut [0; 16][..];
        for block in s.as_bytes().chunks(16) {
            buf[..block.len()].copy_from_slice(block);
            self.write(&buf[..block.len()]).map_err(|_| fmt::Error)?;
        }

        Ok(())
    }
}

pub struct Pins {
    pub rxd: Pin<Input<Floating>>,
    pub txd: Pin<Output<PushPull>>,
    pub cts: Option<Pin<Input<Floating>>>,
    pub rts: Option<Pin<Output<PushPull>>>,
}

#[derive(Debug)]
pub enum Error {
    TxBufferTooLong,
    RxBufferTooLong,
    Transmit,
    Receive,
    Timeout(usize),
}

pub trait Instance: Deref<Target = uarte0::RegisterBlock> {
    fn ptr() -> *const uarte0::RegisterBlock;
}

impl Instance for UARTE0 {
    fn ptr() -> *const uarte0::RegisterBlock {
        UARTE0::ptr()
    }
}
