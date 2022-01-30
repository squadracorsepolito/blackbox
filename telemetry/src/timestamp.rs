use gd32vf103xx_hal::rcu::Clocks;

#[derive(Copy, Clone)]
pub struct Timestamp {
    core_frequency: u32
}

impl Timestamp {
    /// Constructs the delay provider
    pub fn new(clocks: &Clocks) -> Self {
        Self {
            core_frequency: clocks.sysclk().0
        }
    }

    pub fn now(&mut self) -> u64 {
        (riscv::register::mcycle::read64() * 1_000_000) / self.core_frequency as u64
    }
}
