# ACT43750 I²C Driver for STM32

Firmware driver for the Qorvo ACT43750 RF POL Drain-Switch Driver / Gate-Bias Regulator

## 🎯 Features Implemented

- ✅ Full I²C communication using STM32 HAL
- ✅ Register read/write operations
- ✅ State machine monitoring
- ✅ Gate voltage control (DACG programming)
- ✅ Drain voltage control (DSW50)
- ✅ Auto-calibration support
- ✅ Interrupt handling
- ✅ Safety sequencing (gate before drain)
- ✅ Comprehensive error handling

## 📁 File Structure

```
ACT43750/
├── ACT43750.h          # Header file with API definitions
├── ACT43750.c          # Implementation file
├── README.md           # This file
└── main.c              # Example integration code
```
## 🔧 Hardware Requirements

### Pull-up Resistors (Critical!)

I²C requires pull-up resistors on SDA and SCL lines:
- **Typical values:** 3.3kΩ to 10kΩ
- **Why needed:** I²C uses open-drain outputs, pull-ups provide the HIGH state
- **Voltage:** Connect to appropriate logic level (typically 3.3V or 5V)

**Why pull-ups are required:**
1. I²C is a bidirectional protocol using open-drain/open-collector drivers
2. Devices can only pull the line LOW, not drive it HIGH
3. Pull-up resistors provide the return path to HIGH state when all devices release the line
4. Without pull-ups, the bus will float and communication will fail

### I²C Address

- **Default 8-bit address:** 0x40 
- **Configurable via CONF pin resistors** (see datasheet Table 5)

### External Components

Per ACT43750 datasheet requirements:
- V12: 10µF ceramic capacitor to GND
- V5: 2.2µF ceramic capacitor to GND
- GV5: 2.2µF capacitor to VG
- L1: 6.8µH inductor (REGG output filter)
- CGATE: 22µF capacitor (gate output)
- RT: Resistor to set switching frequency (typically 120kΩ)

## 📖 Conceptual Questions - Answers

### 1. I²C Slave vs I²C Master Interface

**I²C Slave Interface (SDA/SCL):**
- ACT43750 acts as a **slave device**
- Controlled by external microcontroller (STM32, ESP32, etc.)
- Used for configuration, monitoring, and control
- Default address: 0x40 (7-bit)

**I²C Master Interface (SDA50/SCL50):**
- ACT43750 acts as a **master device**
- Controls companion devices (ACT43850, external EEPROM)
- Can enable/disable ACT43850
- Reads lookup tables from external memory
- Automatically scans for devices on startup

**Key Difference:** The slave interface receives commands from system MCU, while the master interface sends commands to companion ICs.

### 2. State[3:0] Field in Register 0x00

The **State[3:0]** field represents the finite state machine's current state:

**Purpose:**
- Indicates device operational status
- Ensures proper sequencing (gate voltage before drain voltage)
- Enables safe startup/shutdown procedures
- Allows monitoring of calibration progress

**Key States:**
- `0x00`: No Power - waiting for V12 bias
- `0x02`: Bias Ready - ready to enable REGG
- `0x05`: TX Ready - ready for transmission (power good)
- `0x06`: RSCAL CAL - calibration in progress
- `0x09`: DSW50 ON - drain voltage applied to RF PA
- `0x0D`: Bias Error - fault detected, requires power cycle

**Why it matters:** The state machine enforces GaN FET safety requirements. GaN devices are depletion-mode, so negative gate voltage must be applied BEFORE drain voltage to prevent damage.

### 3. Negative Gate Regulator (REGG)

**Purpose:**
The REGG block generates the negative gate voltage required by GaN RF power amplifiers.

**How it works:**
- Inverting buck DC-DC converter topology
- Input: +12V (V12 pin)
- Output: -6V to -1.5V (VG pin), adjustable
- Switching frequency: 1-2 MHz (set by RT resistor)
- Ultra-low noise: <200µVRMS output noise

**Why negative voltage is needed:**
- GaN FETs are **depletion-mode** devices (normally ON)
- Require negative gate voltage to turn OFF
- Typical operating point: -2V to -3V
- Must be applied BEFORE drain voltage for safety

**Technical details:**
- 13-bit DACG controls reference voltage (0.75V to 3.0V)
- Gate voltage VG = -2 × VDACG
- Integrated switching FETs (no external switches needed)
- Supports 300mA source, 100mA sink current

### 4. Why Pull-ups are Required on I²C

**Technical Reason:**
I²C uses **open-drain** (or open-collector) outputs:
- Devices can only **pull the line LOW** (sink current)
- Devices **cannot drive the line HIGH** (no push capability)
- Pull-up resistors provide the **HIGH state** when no device is pulling LOW

**Without pull-ups:**
- Bus lines would float when released
- No defined HIGH state
- Communication fails completely

**Typical Values:**
- **Fast mode (400 kHz):** 2.2kΩ to 4.7kΩ
- **Standard mode (100 kHz):** 4.7kΩ to 10kΩ
- **ACT43750 recommended:** 3.3kΩ to 10kΩ

**Calculation factors:**
```
Rmin = (VDD - VOL(max)) / IOL
Rmax = tr / (0.8473 × Cb)

Where:
- VDD: Supply voltage (3.3V or 5V)
- VOL: LOW level output voltage
- IOL: LOW level output current
- tr: Rise time requirement
- Cb: Bus capacitance
```

For ACT43750:
- Bus capacitance: <400pF (per datasheet)
- Recommended: 3.3kΩ when using 5V pull-up
- Recommended: 4.7kΩ when using 3.3V pull-up

## 🚀 Quick Start Guide

### 1. Hardware Setup

```c
// In your STM32 project, configure I2C peripheral
// Example for I2C1 (for nucleo-L476RG):
// - SCL: PB6
// - SDA: PB7
// - Speed: 100 kHz (standard mode)
// - Pull-ups: External 4.7kΩ resistors to 3.3V
```

### 2. Include Driver

```c
#include "ACT43750.h"

// Declare device structure
act43750_t act_device;
I2C_HandleTypeDef hi2c1;  // Your I2C handle
```

### 3. Initialize Device

```c
HAL_StatusTypeDef status;

// Initialize the driver
status = ACT43750_Init(&act_device, &hi2c1);
if (status != HAL_OK) {
    // Handle error - check I2C connection, pull-ups
    Error_Handler();
}
```

### 4. Typical Startup Sequence

```c
// 1. Enable gate voltage (REGG)
status = ACT43750_EnableREGG(&act_device);
if (status != HAL_OK) {
    Error_Handler();
}

HAL_Delay(50);  // Wait for REGG ramp

// 2. Run calibration to find optimal gate voltage
status = ACT43750_StartCalibration(&act_device);
if (status != HAL_OK) {
    Error_Handler();
}

// 3. Wait for calibration to complete
bool cal_complete = false;
uint32_t timeout = HAL_GetTick() + 100;  // 100ms timeout
while (!cal_complete && HAL_GetTick() < timeout) {
    status = ACT43750_IsCalibrationComplete(&act_device, &cal_complete);
    HAL_Delay(5);
}

if (!cal_complete) {
    // Calibration timeout or error
    Error_Handler();
}

// 4. Enable drain voltage
status = ACT43750_SetDrainVoltage(&act_device);
if (status != HAL_OK) {
    Error_Handler();
}

// RF PA is now operational!
```

## 📚 API Reference

### Initialization

```c
HAL_StatusTypeDef ACT43750_Init(act43750_t *dev, I2C_HandleTypeDef *hi2c);
```
Initializes the ACT43750 device, validates I²C communication, reads initial state.

### Register Access

```c
HAL_StatusTypeDef ACT43750_ReadReg(act43750_t *dev, uint8_t reg, uint8_t *val);
HAL_StatusTypeDef ACT43750_WriteReg(act43750_t *dev, uint8_t reg, uint8_t val);
```
Low-level register read/write operations.

### State Management

```c
HAL_StatusTypeDef ACT43750_GetState(act43750_t *dev, act43750_state_info_t *state);
```
Reads and decodes current device state.

### Voltage Control

```c
HAL_StatusTypeDef ACT43750_SetGateVoltage(act43750_t *dev, int16_t vg_mv);
HAL_StatusTypeDef ACT43750_GetGateVoltage(act43750_t *dev, int16_t *vg_mv);
```
Set/get gate voltage (-6000mV to -1500mV range).

### Power Control

```c
HAL_StatusTypeDef ACT43750_EnableREGG(act43750_t *dev);
HAL_StatusTypeDef ACT43750_DisableREGG(act43750_t *dev);
HAL_StatusTypeDef ACT43750_SetDrainVoltage(act43750_t *dev);
HAL_StatusTypeDef ACT43750_DisableDrainVoltage(act43750_t *dev);
```
Enable/disable REGG and drain voltage.

### Calibration

```c
HAL_StatusTypeDef ACT43750_StartCalibration(act43750_t *dev);
HAL_StatusTypeDef ACT43750_IsCalibrationComplete(act43750_t *dev, bool *is_complete);
```
Auto-calibration for optimal Idq bias point.
Calibration Resistor Calculation
```
For Idq = 50mA:
RSCAL = 1.5V / 0.05A = 30Ω
```
## 🧮 Gate Voltage Calculation

The ACT43750 uses a 13-bit DAC to control gate voltage:

```
VG = -2 × VDACG
VDACG = 0.75V + (DACG_code × 366.2µV)
```

**Example:**
- To set VG = -2.5V:
  - VDACG = 2.5V / 2 = 1.25V
  - DACG_code = (1.25V - 0.75V) / 366.2µV = 1365
  - Binary: 0b0101_0101_0001_0101

**Helper functions provided:**
```c
uint16_t code = ACT43750_VoltageToCode(-2500);  // -2.5V
int16_t voltage = ACT43750_CodeToVoltage(1365);  // Returns -2500
```
