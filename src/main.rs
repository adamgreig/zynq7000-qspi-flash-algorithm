#![no_std]
#![no_main]

use flash_algorithm::*;
use aarch32_cpu::asm::dmb;

struct Algorithm;

algorithm!(Algorithm, {
    device_name: "X7Z",
    device_type: DeviceType::ExtSpi,
    flash_address: 0xfc000000,
    flash_size: 16777216,
    page_size: 256,
    empty_value: 0xFF,
    program_time_out: 1000,
    erase_time_out: 2000,
    sectors: [{
        size: 4096,
        address: 0x0,
    }]
});

const SLCR_LOCK: *mut u32 = 0xf8000004 as *mut u32;
const SLCR_UNLOCK: *mut u32 = 0xf8000008 as *mut u32;
const GPIO_RST_CTRL: *mut u32 = 0xF800022C as *mut u32;
const MIO_PIN_13: *mut u32 = 0xF8000734 as *mut u32;
const DIRM_0: *mut u32 = 0xE000A204 as *mut u32;
const OEN_0: *mut u32 = 0xE000A208 as *mut u32;
const DATA_0: *mut u32 = 0xE000A040 as *mut u32;

impl FlashAlgorithm for Algorithm {
    #[instruction_set(arm::t32)]
    fn new(_address: u32, _clock: u32, _function: Function) -> Result<Self, ErrorCode> {
        unsafe {
            dmb();
            SLCR_UNLOCK.write_volatile(0xdf0d);
            dmb();
            GPIO_RST_CTRL.write_volatile(0);
            MIO_PIN_13.write_volatile(0b1_0_011_0_000_00_0_0_0);
            DIRM_0.write_volatile(0x0000_2000);
            OEN_0.write_volatile(0x0000_2000);
            DATA_0.write_volatile(0x0000_0000);
            dmb();
            SLCR_LOCK.write_volatile(0x767b);
            dmb();
        }
        Ok(Self)
    }

    #[instruction_set(arm::t32)]
    fn erase_all(&mut self) -> Result<(), ErrorCode> {
        unsafe {
            dmb();
            SLCR_UNLOCK.write_volatile(0xdf0d);
            dmb();
            DATA_0.write_volatile(0x0000_2000);
            dmb();
            SLCR_LOCK.write_volatile(0x767b);
            dmb();
        }

        Ok(())
    }

    #[instruction_set(arm::t32)]
    fn erase_sector(&mut self, addr: u32) -> Result<(), ErrorCode> {
        Ok(())
    }

    #[instruction_set(arm::t32)]
    fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), ErrorCode> {
        Ok(())
    }

    #[instruction_set(arm::t32)]
    fn read_flash(&mut self, addr: u32, data: &mut [u8]) -> Result<(), ErrorCode> {
        Ok(())
    }

    #[instruction_set(arm::t32)]
    fn blank_check(&mut self, addr: u32, size: u32, pattern: u8) -> Result<(), ErrorCode> {
        Ok(())
    }

    #[instruction_set(arm::t32)]
    fn verify(&mut self, addr: u32, size: u32, data: Option<&[u8]>) -> u32 {
        addr + size
    }
}

impl Drop for Algorithm {
    #[instruction_set(arm::t32)]
    fn drop(&mut self) {
    }
}
