---
name: stm32-build-flash-debug
description: Automated compilation, SWD flashing, GDB debugging, CLI programming, and automated hardware-in-the-loop (HIL) testing for STM32 microcontrollers (STM32G4/F4/H7).
---

# STM32 Firmware Build, Flash, Debug & Hardware-in-the-Loop Skill

This skill provides a standard runbook and reference for compiling, flashing via SWD/JTAG/DFU, headless GDB debugging, serial port arbitration, and automated test execution on STM32 microcontrollers.

---

## 1. Toolchain & Compiler Discovery

STM32CubeIDE bundles standalone GNU ARM GCC toolchains in its plugin directory:

- **ARM GCC Toolchain:**
  `/home/du/st/stm32cubeide_1.19.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.linux64_1.0.0.202410170706/tools/bin`
- **Compiler:** `arm-none-eabi-gcc`
- **GDB Debugger:** `arm-none-eabi-gdb`
- **Size Utility:** `arm-none-eabi-size`
- **Objdump:** `arm-none-eabi-objdump`

### Standard Makefile Build Command
```bash
export PATH=/home/du/st/stm32cubeide_1.19.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.linux64_1.0.0.202410170706/tools/bin:$PATH
cd firmware/joint_driver/joint-driver-8115/Debug && make -j4 all
```

---

## 2. STM32 Programmer CLI Flashing (Primary SWD Tool)

- **CLI Binary:**
  `/home/du/st/stm32cubeide_1.19.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.200.202503041107/tools/bin/STM32_Programmer_CLI`

### Probe Discovery & Device Listing
```bash
STM32_Programmer_CLI -l
```

### Full Flash & Reset Command (with Probe SN)
```bash
STM32_Programmer_CLI -c port=SWD freq=1000 sn=E1007200D0D2139393740544 -w firmware/joint_driver/joint-driver-8115/Debug/joint-driver-8115.elf -v -rst
```

> [!IMPORTANT]
> **Serial Port Arbitration Rule:** If a daemon, dashboard, or serial script is actively reading `/dev/ttyACM0` (or `/dev/ttyUSB0`), you **MUST** disconnect or stop serial polling BEFORE issuing SWD reset/programming, otherwise the host USB CDC driver will wedge or report "Device is not connected".

---

## 3. Alternative Flashing & Debugging Toolchains

Besides `STM32_Programmer_CLI`, the following modern tools can be used for STM32 firmware workflows:

### A. PyOCD (Python-native SWD Flashing & GDB Server)
PyOCD is a fast, Python-based tool that works with ST-LINK, CMSIS-DAP, and J-Link:
```bash
# Flash ELF or HEX directly:
pyocd flash firmware.elf --target stm32g473re

# Start headless GDB Server on port 3333:
pyocd gdbserver --target stm32g473re --port 3333
```

### B. OpenOCD (Open On-Chip Debugger)
```bash
# Flash and exit:
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program firmware.elf verify reset exit"

# GDB Server mode:
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg
```

### C. `st-flash` / `st-link` CLI (Linux open-source ST-Link utilities)
```bash
# Flash raw binary to 0x08000000:
st-flash --reset write firmware.bin 0x08000000
```

### D. USB DFU Bootloader (`dfu-util`)
When booting in system bootloader mode (BOOT0=1 or via software jump to system memory):
```bash
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin
```

### E. UART / USB System Bootloader via `STM32_Programmer_CLI`
```bash
# Over UART:
STM32_Programmer_CLI -c port=/dev/ttyUSB0 br=115200 -w firmware.elf -v -g 0x08000000

# Over USB DFU:
STM32_Programmer_CLI -c port=USB1 -w firmware.elf -v -rst
```

---

## 4. Headless Automated GDB Debugging

To run automated assertions, read registers, or capture stack traces on crash without a GUI:

```bash
arm-none-eabi-gdb firmware/joint_driver/joint-driver-8115/Debug/joint-driver-8115.elf \
  -ex "target extended-remote localhost:3333" \
  -ex "monitor reset halt" \
  -ex "break HardFault_Handler" \
  -ex "continue" \
  -ex "bt" \
  -ex "info registers" \
  -ex "quit"
```

---

## 5. Automated HIL Test Sequence Workflow (Motor Driver Standard)

When automating code-build-flash-test cycles with serial telemetry backends:

```bash
# 1. Stop motor cleanly
curl -sS -X POST http://127.0.0.1:1111/api/command -H 'Content-Type: application/json' -d '{"command":"STOP"}'

# 2. Disconnect serial to release USB device
curl -sS -X POST http://127.0.0.1:1111/api/disconnect

# 3. Flash firmware via SWD
/home/du/st/stm32cubeide_1.19.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.200.202503041107/tools/bin/STM32_Programmer_CLI -c port=SWD freq=1000 sn=E1007200D0D2139393740544 -w firmware/joint_driver/joint-driver-8115/Debug/joint-driver-8115.elf -v -rst

# 4. Reconnect serial
curl -sS -X POST http://127.0.0.1:1111/api/connect -H 'Content-Type: application/json' -d '{"port":"/dev/ttyACM0","baudrate":115200}'

# 5. Run Alignment / Calibration
curl -sS -X POST http://127.0.0.1:1111/api/command -H 'Content-Type: application/json' -d '{"command":"ALIGN"}'

# 6. Start Recording & Launch Closed-Loop Test
curl -sS -X POST http://127.0.0.1:1111/api/record/start
curl -sS -X POST http://127.0.0.1:1111/api/command -H 'Content-Type: application/json' -d '{"command":"SPEED 50"}'

# 7. In case of stall: Always issue immediate STOP to prevent stator heating
curl -sS -X POST http://127.0.0.1:1111/api/command -H 'Content-Type: application/json' -d '{"command":"STOP"}'
```
