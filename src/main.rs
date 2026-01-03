#![no_std]
#![no_main]

use flash_algorithm::*;
use aarch32_cpu::asm::dmb;

struct Algorithm;

const FLASH_ADDR: u32 = 0xFC00_0000;
const FLASH_SIZE: u32 = 16 * 1024 * 1024;

algorithm!(Algorithm, {
    device_name: "X7Z",
    device_type: DeviceType::ExtSpi,
    flash_address: FLASH_ADDR,
    flash_size: FLASH_SIZE,
    page_size: 256,
    empty_value: 0xFF,
    program_time_out: 1000,
    erase_time_out: 2000,
    sectors: [{
        size: 4096,
        address: 0x0,
    }]
});

const SLCR_LOCK: *mut u32 = 0xf800_0004 as *mut u32;
const SLCR_UNLOCK: *mut u32 = 0xf800_0008 as *mut u32;
const SLCR_IO_PLL_CTRL: *mut u32 = 0xf800_0108 as *mut u32;
const SLCR_QSPI_CLK_CTRL: *mut u32 = 0xf800_014c as *mut u32;
const SLCR_QSPI_RST_CTRL: *mut u32 = 0xf800_0230 as *mut u32;
const MIO_PIN_01: *mut u32 = 0xF8000704 as *mut u32;
const MIO_PIN_02: *mut u32 = 0xF8000708 as *mut u32;
const MIO_PIN_03: *mut u32 = 0xF800070C as *mut u32;
const MIO_PIN_04: *mut u32 = 0xF8000710 as *mut u32;
const MIO_PIN_05: *mut u32 = 0xF8000714 as *mut u32;
const MIO_PIN_06: *mut u32 = 0xF8000718 as *mut u32;
const MIO_PIN_08: *mut u32 = 0xF8000720 as *mut u32;
const QSPI_CONFIG_REG: *mut u32 = 0xE000_D000 as *mut u32;
const QSPI_STATUS_REG: *mut u32 = 0xE000_D004 as *mut u32;
const QSPI_INT_DIS_REG: *mut u32 = 0xE000_D00C as *mut u32;
const QSPI_EN_REG: *mut u32 = 0xE000_D014 as *mut u32;
const QSPI_TXD0: *mut u32 = 0xE000_D01C as *mut u32;
const QSPI_TXD1: *mut u32 = 0xE000_D080 as *mut u32;
const QSPI_TXD2: *mut u32 = 0xE000_D084 as *mut u32;
const QSPI_TXD3: *mut u32 = 0xE000_D088 as *mut u32;
const QSPI_RXD: *mut u32 = 0xE000_D020 as *mut u32;
const QSPI_LPBK_DLY_ADJ: *mut u32 = 0xE000_D038 as *mut u32;
const QSPI_LQSPI_CONFIG: *mut u32 = 0xE000_D0A0 as *mut u32;

/// Set QSPI controller to linear mode; flash is read-only and mapped from 0xFC00_0000.
fn linear_mode() {
    unsafe {
        // Ensure disabled before reconfiguring.
        QSPI_EN_REG.write_volatile(0);
        // Configure controller for automatic flash mode at 25MHz.
        QSPI_CONFIG_REG.write_volatile((1 << 31) | (0b11 << 6) | (0b111 << 3) | 1);
        // Configure linear mode for Quad Out Read Fast (0x6B).
        // We avoid Quad I/O Read Fast because the number of dummy bytes required differs
        // between Winbond and Micron devices (2 vs 4).
        QSPI_LQSPI_CONFIG.write_volatile(0x8000_016B);
        // Leave enabled while in linear mode.
        QSPI_EN_REG.write_volatile(1);
    }
}

/// Set QSPI controller to IO mode; flash is accessed indirectly but writes are possible.
fn io_mode() {
    unsafe {
        // Ensure disabled before reconfiguring.
        QSPI_EN_REG.write_volatile(0);
        // Configure controller for manual start and manual CS flash mode at 25MHz,
        // with PCS deasserted.
        QSPI_CONFIG_REG.write_volatile(
            (1 << 31) | (1 << 15) | (1 << 14) | (1 << 10) | (0b11 << 6) | (0b111 << 3) | 1
        );
        // Set up flash controller for IO mode operations
        QSPI_LQSPI_CONFIG.write_volatile(0x0000_016B);
        // Leave disabled, operations will enable as required.
    }
}

/// Set PCS in QSPI_CONFIG_REG.
fn set_pcs(pcs: u8) {
    unsafe {
        let mut cfg = QSPI_CONFIG_REG.read_volatile();
        if pcs == 0 {
            cfg &= !(1 << 10);
        } else {
            cfg |= 1 << 10;
        }
        QSPI_CONFIG_REG.write_volatile(cfg);
    }
}

/// Enable/disable QSPI controller.
fn set_enable(enable: bool) {
    unsafe { QSPI_EN_REG.write_volatile(enable as u32) }
}

/// Manual start data transfer.
fn start_transfer() {
    unsafe {
        // Clear interrupts
        QSPI_STATUS_REG.write_volatile(0b100_0001);
        // Set manual start bit.
        let mut cfg = QSPI_CONFIG_REG.read_volatile();
        cfg |= 1 << 16;
        QSPI_CONFIG_REG.write_volatile(cfg);
    }
}

/// Get RX FIFO not-empty flag
fn rxne() -> bool {
    unsafe {
        ((QSPI_STATUS_REG.read_volatile() >> 4) & 1) != 0
    }
}

/// Get TX FIFO full flag
fn txf() -> bool {
    unsafe {
        ((QSPI_STATUS_REG.read_volatile() >> 3) & 1) != 0
    }
}

/// Read a single 32-bit entry from the RX FIFO or timeout, deassert PCS, and swap to linear mode.
fn read_rx() -> Result<u32, ErrorCode> {
    for _ in 0..100000 {
        if rxne() {
            return Ok(unsafe { QSPI_RXD.read_volatile() });
        }
    }

    // Timeout
    set_pcs(1);
    linear_mode();
    return Err(ErrorCode::new(100).unwrap());
}

/// Send a single-byte command and ignore the result.
fn send_command(cmd: u8) -> Result<(), ErrorCode> {
    set_pcs(0);
    set_enable(true);
    let cmd = (cmd as u32) | 0x55AACC00;
    unsafe { QSPI_TXD1.write_volatile(cmd as u32) };
    start_transfer();
    read_rx()?;
    set_pcs(1);
    set_enable(false);
    Ok(())
}

/// Send a four-byte command and ignore the result.
fn send_command_and_address(cmd: u8, addr: u32) -> Result<(), ErrorCode> {
    set_pcs(0);
    set_enable(true);
    unsafe { QSPI_TXD0.write_volatile((addr << 8) | (cmd as u32)) };
    start_transfer();
    read_rx()?;
    set_pcs(1);
    set_enable(false);
    Ok(())
}

/// Send write-enable command.
fn write_enable() -> Result<(), ErrorCode> {
    send_command(0x06)
}

/// Read status register.
fn read_status() -> Result<u8, ErrorCode> {
    set_pcs(0);
    set_enable(true);
    unsafe { QSPI_TXD2.write_volatile(0x05) };
    start_transfer();
    let rx = read_rx()?;
    set_pcs(1);
    set_enable(false);
    Ok((rx >> 24) as u8)
}

/// Wait until status register says not busy.
fn wait_not_busy() -> Result<(), ErrorCode> {
    loop {
        let status = read_status()?;
        if status & 1 == 0 {
            return Ok(());
        }
    }
}

impl FlashAlgorithm for Algorithm {
    fn new(_address: u32, _clock: u32, _function: Function) -> Result<Self, ErrorCode> {
        unsafe {
            // Enable access to SLCR.
            dmb();
            SLCR_UNLOCK.write_volatile(0xdf0d);
            dmb();

            // Reset and stop QSPI.
            SLCR_QSPI_RST_CTRL.write_volatile(0b11);
            SLCR_QSPI_CLK_CTRL.write_volatile(0);
            dmb();
            SLCR_QSPI_RST_CTRL.write_volatile(0);
            dmb();

            // Read IO_PLL to guess its frequency, assuming standard 33.333MHz external clock.
            let iopll_fdiv = SLCR_IO_PLL_CTRL.read_volatile() >> 12 & 0b111_1111;
            let iopll_freq = (iopll_fdiv * 100) / 3;

            // Enable QSPI reference clock at 200MHz from detected IO_PLL frequency.
            let qspi_ref_clk_div = iopll_freq.div_ceil(200) & 0b11_1111;
            SLCR_QSPI_CLK_CTRL.write_volatile((qspi_ref_clk_div << 8) | 1);

            // Set up MIO for single QSPI device with feedback clock.
            MIO_PIN_01.write_volatile((0b011 << 9) | (1 << 1));
            MIO_PIN_02.write_volatile((0b011 << 9) | (1 << 1));
            MIO_PIN_03.write_volatile((0b011 << 9) | (1 << 1));
            MIO_PIN_04.write_volatile((0b011 << 9) | (1 << 1));
            MIO_PIN_05.write_volatile((0b011 << 9) | (1 << 1));
            MIO_PIN_06.write_volatile((0b011 << 9) | (1 << 1));
            MIO_PIN_08.write_volatile((0b011 << 9) | (1 << 1));

            // Re-lock SLCR.
            SLCR_LOCK.write_volatile(0x767b);
            dmb();

            // Enable loopback clock (TODO: only after bumping clock to 100MHz)
            //QSPI_LPBK_DLY_ADJ.write_volatile(1<<5);

            // Ensure all interrupts are disabled.
            QSPI_INT_DIS_REG.write_volatile(0b111_1101);

            // Enable linear mode by default so debugger can read flash memory.
            linear_mode();
        }
        Ok(Self)
    }

    fn erase_all(&mut self) -> Result<(), ErrorCode> {
        io_mode();

        write_enable()?;
        send_command(0xc7)?;
        wait_not_busy()?;

        linear_mode();
        Ok(())
    }

    fn erase_sector(&mut self, addr: u32) -> Result<(), ErrorCode> {
        io_mode();

        write_enable()?;
        send_command_and_address(0x20, addr - FLASH_ADDR)?;
        wait_not_busy()?;

        linear_mode();
        Ok(())
    }

    fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), ErrorCode> {
        let mut buf = [0u32; 64];
        for (idx, bytes) in data.chunks(4).enumerate() {
            let word = match bytes.len() {
                4 => u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
                3 => u32::from_le_bytes([bytes[0], bytes[1], bytes[2], 0xff]),
                2 => u32::from_le_bytes([bytes[0], bytes[1], 0xff, 0xff]),
                1 => u32::from_le_bytes([bytes[0], 0xff, 0xff, 0xff]),
                _ => return Err(ErrorCode::new(2).unwrap()),
            };
            buf[idx] = word;
        }

        io_mode();

        write_enable()?;

        set_pcs(0);
        set_enable(true);
        let mut buf_idx = 0;
        unsafe { QSPI_TXD0.write_volatile((addr << 8) | 0x02) };
        while buf_idx < buf.len() && !txf() {
            unsafe { QSPI_TXD0.write_volatile(buf[buf_idx]) };
            buf_idx += 1;
        }
        start_transfer();
        let mut rxcount = 0;
        while rxcount < 1 + buf.len() {
            if rxne() {
                unsafe { QSPI_RXD.read_volatile() };
                rxcount += 1;
            }
            if buf_idx < buf.len() && !txf() {
                unsafe { QSPI_TXD0.write_volatile(buf[buf_idx]) };
                buf_idx += 1;
            }
        }
        set_pcs(1);
        set_enable(false);

        wait_not_busy()?;

        linear_mode();
        Ok(())
    }

    fn read_flash(&mut self, addr: u32, data: &mut [u8]) -> Result<(), ErrorCode> {
        if addr < FLASH_ADDR || data.len() > FLASH_SIZE as usize {
            return Err(ErrorCode::new(1).unwrap());
        }
        let flash = unsafe { core::slice::from_raw_parts(addr as *const u8, data.len()) };
        data.copy_from_slice(&flash[..]);
        Ok(())
    }

    fn verify(&mut self, addr: u32, size: u32, data: Option<&[u8]>) -> Result<(), u32> {
        let Some(data) = data else {
            return Err(0);
        };
        if data.len() != size as usize {
            return Err(addr.wrapping_add(data.len() as u32));
        }
        if addr < FLASH_ADDR || size > FLASH_SIZE {
            return Err(0xFFFFFFFF);
        }
        for (idx, byte) in data.iter().enumerate() {
            if unsafe { (addr as *const u8).offset(idx as isize).read() } != *byte {
                return Err(addr.wrapping_add(idx as u32));
            }
        }
        Ok(())
    }

    fn blank_check(&mut self, addr: u32, size: u32, pattern: u8) -> Result<(), ErrorCode> {
        for idx in 0..size {
            if unsafe { (addr as *const u8).offset(idx as isize).read() } != pattern {
                return Err(ErrorCode::new(1).unwrap());
            }
        }
        Ok(())
    }
}

impl Drop for Algorithm {
    /// Disable and reset the QSPI peripheral. Leave MIO pins configured.
    fn drop(&mut self) {
        unsafe {
            set_pcs(1);
            QSPI_EN_REG.write_volatile(0);
            SLCR_QSPI_RST_CTRL.write_volatile(0b11);
            SLCR_QSPI_CLK_CTRL.write_volatile(0);
            dmb();
            SLCR_QSPI_RST_CTRL.write_volatile(0);
            dmb();
        }
    }
}
