# UWB Two-Way Ranging — IIT Delhi Internship

> **Intern:** Syed Abdul Musavvir & Affan Danish — B.Tech Electronics & Communication Engineering, 3rd Year  
> **Home Institution:** Jamia Millia Islamia, New Delhi  
> **Host Lab:** MDIT Lab, Centre for Sensors, Instrumentation and Cyber Physical System Engineering (SeNSE), IIT Delhi  
> **Supervisor:** Prof. Shahid Malik — SeNSE, IIT Delhi  
> **Focus Area:** Ultra-Wideband (UWB) Ranging · Embedded Systems · Sensor Fusion  
> **Hardware Platform:** NUCLEO-G070RB (STM32G070RB) · DWM1000 UWB Transceiver Module  
> **Toolchain:** STM32CubeIDE · STM32 HAL · Python (serial logging & plotting)

---

## Overview

This repository documents the complete work carried out during a research internship at IIT Delhi on **Ultra-Wideband (UWB) based precision ranging for drone and robotics applications**.

The primary deliverable is a working **Single-Sided Two-Way Ranging (SS-TWR)** system implemented on bare-metal STM32 microcontrollers using Decawave DWM1000 UWB modules.  The system achieves sub-metre range measurement between an **Anchor** node and a **Tag** node over a custom SPI driver stack written from scratch against the DW1000 register-level specification.

Secondary work covered BNO055 IMU integration and a literature survey on fusing UWB ranging with Magnetic Induction (MI) positioning for underground/GPS-denied drone navigation.

---

## What Was Achieved

| Milestone | Status |
|-----------|--------|
| DW1000 SPI driver (register + sub-register addressing) | ✅ Complete |
| DW1000 full RF init sequence (AGC / DRX / LDE / FS tuning) | ✅ Complete |
| LDE microcode load via PMSC/OTP sequence | ✅ Complete |
| CPLOCK-gated dual-speed SPI (2 MHz init → 8 MHz post-lock) | ✅ Complete |
| SS-TWR POLL → RESPONSE → FINAL protocol | ✅ Complete |
| Piggyback t\_round scheme (avoids delayed-TX hardware requirement) | ✅ Complete |
| Real-time distance output over UART (115200 baud) | ✅ Complete |
| Python serial logger & plotter for range data | ✅ Complete |
| BNO055 IMU bring-up on STM32G070CB | ✅ Complete |
| Serial data logs (20+ sessions) | ✅ Captured |
| Literature survey: UWB + MI fusion for drones | ✅ Complete |

---

## Hardware Setup

```
DWM1000 Module  ──────────────────────────────  NUCLEO-G070RB
─────────────────                               ────────────────
Pin 20  SPICLK   ────────────────────────────►  PA5  (D13 / SPI1_SCK)
Pin 19  SPIMISO  ◄───────────────────────────   PA6  (D12 / SPI1_MISO)
Pin 18  SPIMOSI  ────────────────────────────►  PA7  (D11 / SPI1_MOSI)
Pin 17  SPICSn   ◄───── (manual GPIO) ────────  PB0  (D10)
Pin 22  IRQ      ────────────────────────────►  PA9  (D8  / input)
Pin  3  RSTn     ◄───── (open-drain)  ────────  PA8  (D7)
Pin  5  VDDAON   ── 3.3 V
Pins 6,7 VDD3V3  ── 3.3 V
Pin  2  WAKEUP   ── GND (unused)
All VSS / GND    ── GND
```

**Two identical setups** are used — one flashed as **Anchor**, one as **Tag**. Only the firmware role differs; hardware is identical.

---

## Repository Structure

```
Internship_IITD/
│
├── Codes/
│   └── UWB_Two_Way_Ranging/
│       ├── Anchor/                     # STM32CubeIDE project — ANCHOR role
│       │   ├── Core/
│       │   │   ├── Inc/
│       │   │   │   ├── dw1000.h        # DW1000 register map, sub-reg constants,
│       │   │   │   │                   # AGC/DRX/LDE/FS tuning values, prototypes
│       │   │   │   ├── dw1000_stm32.h  # SPI HAL backend: standard + extended
│       │   │   │   │                   # sub-register addressing (2/3-byte header)
│       │   │   │   ├── dw1000_time.h   # DW1000 timestamp ↔ µs / metres helpers
│       │   │   │   └── main.h          # CubeMX-generated peripheral config
│       │   │   └── Src/
│       │   │       ├── dw1000.c        # Full driver: Init(), WaitForCPLOCK(),
│       │   │       │                   # AGC/DRX/LDE tune, sub-reg R/W helpers
│       │   │       ├── dw1000_time.c   # Timestamp conversion implementations
│       │   │       └── main.c          # ANCHOR application:
│       │   │                           #   hard reset → 2 MHz SPI init →
│       │   │                           #   RF tune → CPLOCK wait →
│       │   │                           #   8 MHz SPI → POLL/RESPONSE/FINAL loop
│       │   └── Drivers/                # STM32G0xx HAL + CMSIS (CubeMX generated)
│       │
│       └── Tag/                        # STM32CubeIDE project — TAG role
│           ├── Core/
│           │   ├── Inc/                # Same headers as Anchor
│           │   └── Src/
│           │       ├── dw1000.c        # Shared driver (identical to Anchor)
│           │       ├── dw1000_time.c
│           │       └── main.c          # TAG application:
│           │                           #   init → POLL TX → RESPONSE RX →
│           │                           #   FINAL TX (piggyback t_round) → repeat
│           └── Drivers/
│
├── BNO055/                             # STM32CubeIDE project — IMU bringup
│   └── Core/Src/main.c                # BNO055 I2C init, orientation readout
│
├── serial_data/                        # Python tools & captured range data
│   ├── uwb_serial_plotter_logger.py    # Main tool: live serial read + log + plot
│   ├── my_Putty_v2.py                  # Custom serial terminal replacement
│   ├── distance_v3.py                  # Distance extraction from UART stream
│   ├── plotting_dist2.py               # Matplotlib distance-vs-time plotter
│   ├── plot_txt_file.py                # Offline plotter for saved log files
│   ├── log_001.txt … log_020.txt       # 20 captured ranging sessions
│   ├── anchor data putty.txt           # Raw anchor UART capture
│   └── Figure_1.png                    # Sample distance plot
│
├── UWB and MI into Drones/            # Literature survey & research documents
│   ├── UWB_and_MagInduction_for drones.pdf
│   ├── Research on IMU-Assisted UWB-Based Positioning Algorithm…pdf
│   ├── GUIDANCE_OF_UNMANNED_AERIAL_VEHICLES.pdf
│   ├── positioning.pdf
│   ├── small-unmanned-aircraft-theory-and-practice…pdf
│   └── APPLICATIONS FOR TACTICAL MEMS + ABSOLUTE ENCODER FUSION_.docx
│
├── Datasheet/                          # Reference datasheets
│   ├── DWM1000 (1).PDF                 # Decawave DWM1000 module datasheet
│   ├── stm32g070cb (1).pdf             # STM32G070CB datasheet
│   └── um2324-stm32-nucleo64-boards…pdf # NUCLEO-64 user manual
│
├── ADIS16505 (Rev. C)/                 # ADIS16505 tactical IMU reference material
├── Executive Summary.docx              # Internship executive summary report
├── LICENSE
└── README.md
```

---

## Ranging Protocol

The system implements **Single-Sided Two-Way Ranging (SS-TWR)** with a **piggyback t\_round** scheme that avoids the need for delayed-transmit hardware support:

```
TAG                                     ANCHOR
 │                                         │
 │──── FC_POLL (0x20) ───────────────────► │  t_poll_rx recorded
 │                                         │
 │ ◄─── FC_RESPONSE (0x21) ───────────────│  t_resp_tx recorded
 │                                         │
 │  t_final_tx recorded after TX           │
 │──── FC_FINAL (0x22) ──────────────────► │
 │     [payload: t_round from prev cycle]  │
 │                                         │  ToF = (t_round_prev - t_reply_prev) / 2
 │                                         │  Distance = ToF × speed_of_light
 │         (repeat every ~200 ms)          │
```

`t_round = t_final_tx − t_poll_tx` is computed by the Tag *after* transmission and sent in the *next* cycle's FINAL frame. The Anchor pairs it with `t_reply = t_resp_tx − t_poll_rx` from the same prior cycle. This is mathematically equivalent to standard SS-TWR and requires no hardware delayed-send.

---

## DW1000 Driver Architecture

The driver stack is split across three files to match the existing project layout:

```
dw1000_stm32.h   ← SPI transport layer
    dw1000_WriteData()          standard 1-byte header (reg read/write)
    dw1000_ReadData()
    dw1000_WriteSubData()       extended 2/3-byte header (sub-register access)
    dw1000_ReadSubData()        per DW1000 User Manual SPI protocol
    dw1000_WriteSubReg{8,16,32}()   convenience typed wrappers
    dw1000_ReadSubReg{8,16,32}()

dw1000.h / dw1000.c  ← register map + logic layer
    Sub-register offsets:  AGC_TUNE1/2/3, DRX_TUNE0b/1a/1b/2/4H,
                           LDE_CFG1/2, RF_RXCTRLH/TXCTRL, TC_PGDELAY,
                           FS_PLLTUNE/PLLCFG/XTALT, PMSC_CTRL0/1,
                           OTP_CTRL  (all with Channel-5 tuning values)
    dw1000_Init()           PMSC soft-reset → LDE load → AGC/DRX/RF/FS tune
    dw1000_WaitForCPLOCK()  HAL_GetTick()-based PLL lock poll
    dw1000_{Start,Clear}Transmit/Receive()
    dw1000_GetTx/RxTimestamp()  40-bit LE timestamp read from TX_TIME/RX_TIME
    dw1000_GetSystemStatus()
    dw1000_GetSystemState()

dw1000_time.h / dw1000_time.c  ← timestamp conversion
    DW1000_Time_TimestampToMicroseconds()
    DW1000_Time_TimestampToMeters()
```

### Key Initialisation Sequence

```
1. Hard reset RSTn (drive LOW → release to Hi-Z, never drive HIGH)
2. SPI at 2 MHz (≤ 3 MHz limit before CLKPLL lock — DWM1000 datasheet Table 2)
3. Verify DEV_ID ridtag == 0xDECA over SPI
4. dw1000_Init(): PMSC soft-reset → OTP LDE microcode load → write all
   required tuning registers (AGC, DRX, LDE, RF_RXCTRLH/TXCTRL,
   TC_PGDELAY, FS_PLLTUNE/PLLCFG/XTALT, TX_POWER, TX_FCTRL, SYS_CFG)
5. dw1000_WaitForCPLOCK() with 200 ms timeout — HALT if not locked
6. Switch SPI to 8 MHz (≤ 20 MHz post-lock limit — datasheet Table 2)
7. Clear all status flags
8. Enter ranging loop
```

### RF Configuration (Channel 5 / PRF 16 MHz / 110 kbps)

| Register | Value | Purpose |
|----------|-------|---------|
| AGC\_TUNE1 | 0x8870 | AGC threshold PRF16 |
| AGC\_TUNE2 | 0x2502A907 | AGC config |
| AGC\_TUNE3 | 0x0035 | AGC config |
| DRX\_TUNE0b | 0x000A | 110 kbps std SFD |
| DRX\_TUNE1a | 0x0087 | PRF16 correlator |
| DRX\_TUNE1b | 0x0064 | 1024-symbol preamble |
| DRX\_TUNE2 | 0x311A002D | PAC=8 PRF16 |
| DRX\_TUNE4H | 0x0028 | preamble > 64 |
| LDE\_CFG1 | 0x6D | LDE NTM / PMULT |
| LDE\_CFG2 | 0x1607 | LDE PRF16 |
| RF\_RXCTRLH | 0xD8 | Ch5 RX analog |
| RF\_TXCTRL | 0x001E3FE0 | Ch5 TX analog |
| TC\_PGDELAY | 0xC0 | Ch5 PG delay |
| FS\_PLLTUNE | 0xA6 | Ch5 PLL tune |
| FS\_PLLCFG | 0x0800041D | Ch5 PLL config |

---

## Serial Data Tools

Located in `serial_data/`, these Python scripts were developed during the internship to capture and visualise live ranging data:

| Script | Purpose |
|--------|---------|
| `uwb_serial_plotter_logger.py` | Live serial read, auto-log to `log_NNN.txt`, real-time distance plot |
| `my_Putty_v2.py` | Custom terminal: timestamps each line, saves session |
| `distance_v3.py` | Regex-extracts distance values from anchor UART stream |
| `plotting_dist2.py` | Matplotlib plots distance-vs-time from live or file data |
| `plot_txt_file.py` | Offline plot from any saved log file |

**Serial port:** 115200 baud, 8N1.  
**Anchor UART output format:**
```
[ANCHOR] Distance=1.234 m
```

---

## Build & Flash

### Requirements

- STM32CubeIDE 1.x
- STM32G0xx HAL drivers (bundled in project `Drivers/`)
- ST-Link V2 or NUCLEO on-board debugger

### Steps

1. Clone the repo and open either `Codes/UWB_Two_Way_Ranging/Anchor` or `.../Tag` in STM32CubeIDE as an existing project.
2. Build → Flash to the respective NUCLEO board.
3. Connect both boards to PC USB (for power + UART).
4. Open two serial terminals at 115200 baud, one per board.
5. Power on — both boards will print `CPLOCK LOCKED` within ~50 ms if wiring is correct, then begin ranging.

### Expected UART Output

**Anchor:**
```
[ANCHOR] DEV_ID model=0x01 ver=3 rev=0 ridtag=0xDECA (OK)
[ANCHOR] dw1000_Init() OK
[ANCHOR] AGC_TUNE1=0x8870(exp 0x8870) LDE_CFG1=0x6D(exp 0x6D) DRX_TUNE2=0x311A002D(exp 0x311A002D)
[ANCHOR] CPLOCK LOCKED
[ANCHOR] SPI switched to 8 MHz (PLL locked).
[ANCHOR] Ready. Listening for POLL frames...
[ANCHOR] RX ok len=1 fc=0x20
[ANCHOR] RESPONSE sent. t_reply_this=...
[ANCHOR] RX ok len=6 fc=0x22
[ANCHOR] Distance=1.234 m
```

**Tag:**
```
[TAG] DEV_ID model=0x01 ver=3 rev=0 ridtag=0xDECA (OK)
[TAG] CPLOCK LOCKED
[TAG] Ready. Starting ranging cycles...
[TAG] POLL sent. t_poll_tx=...
[TAG] RESPONSE received. t_resp_rx=...
[TAG] FINAL sent. t_final_tx=... t_round(next)=...
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `ridtag=0x0000` or `0xFFFF` | SPI not working | Check PA5/6/7/PB0 wiring; confirm 3.3V supply |
| `CPLOCK TIMEOUT` | RF section not starting | Check 3.3V on DWM1000 pins 5,6,7; add 100 nF decoupling |
| `CPLOCK TIMEOUT` on one board only | Damaged module or bad solder joint | Reseat/replace module |
| AGC/LDE readback mismatch | SPI speed too high during init, or CS glitch | Confirm `/32` prescaler is reached before any register write |
| `RX timeout` after CPLOCK | Channel/PRF mismatch between TAG and ANCHOR | Ensure both use Ch5, PRF16, 1024 preamble |
| `RXFCE=0, RXPHE=1` | PHY header error | Usually a channel or SFD config mismatch |
| Distance wildly wrong | Antenna delay not calibrated | Expected until calibration; see next steps |

---

## Next Steps / Future Work

- [ ] Antenna delay calibration (TX\_ANTD / RX\_ANTD registers) for accurate absolute ranging
- [ ] DS-TWR (Double-Sided TWR) for clock-offset immunity — removes systematic error from inter-device frequency offset
- [ ] Multiple anchors → trilateration for 2D/3D position fix
- [ ] Fusion with BNO055 IMU data (complementary filter or EKF) for position + orientation
- [ ] ROS 2 integration for drone localisation use-case

---

## References

- Decawave DWM1000 Module Datasheet v1.8 — `Datasheet/DWM1000 (1).PDF`
- Decawave DW1000 User Manual — register-level SPI protocol, tuning values, LDE load sequence
- STM32G070 Datasheet — `Datasheet/stm32g070cb (1).pdf`
- NUCLEO-64 User Manual — `Datasheet/um2324-stm32-nucleo64-boards…pdf`
- thotro/arduino-dw1000 (open source) — reference for tuning register values
- `UWB and MI into Drones/` — curated papers on UWB + IMU + MI fusion for drone navigation

---

## License

See [LICENSE](LICENSE).

---

*Internship carried out at the **MDIT Lab, SeNSE Centre, IIT Delhi**, under the supervision of **Prof. Shahid Malik**. All firmware, Python tools, and documentation were developed by **Syed Abdul Musavvir & Affan Danish** (Jamia Millia Islamia, B.Tech ECE) during the internship period.*