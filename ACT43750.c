/*
 * ACT43750.c
 *
 *  Created on: Feb 8, 2026
 *      Author: Admin
 */
#include "ACT43750.h"
#include <string.h>


static HAL_StatusTypeDef ACT43750_ReadDACG_Value(act43750_t *dev, uint16_t *dacg_val);
static HAL_StatusTypeDef ACT43750_WriteDACG_Value(act43750_t *dev, uint16_t dacg_val);
static int16_t ACT43750_DACGValToVoltage(uint16_t dacg_val);
static uint16_t ACT43750_VoltageToDACGVal(int16_t vg);
static void ACT43750_UpdateStateInfo(act43750_t *dev, uint8_t state_reg);
static HAL_StatusTypeDef ACT43750_CheckCommunication(act43750_t *dev);

/* Initialize the ACT43750 device */
HAL_StatusTypeDef ACT43750_Init(act43750_t *dev, I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t reg_val;

    if (dev == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }

    memset(dev, 0, sizeof(act43750_t));

    dev->hi2c = hi2c;
    dev->i2c_address = ACT43750_I2C_ADDR;

    status = ACT43750_CheckCommunication(dev);
    if (status != HAL_OK) {
        return status;
    }

    status = ACT43750_ReadReg(dev, ACT43750_REG_STATE, &reg_val);
    if (status != HAL_OK) {
        return status;
    }

    ACT43750_UpdateStateInfo(dev, reg_val);

    status = ACT43750_ReadReg(dev, ACT43750_REG_DACG_MIN, &dev->dacg_min);
    if (status != HAL_OK) {
        return status;
    }

    status = ACT43750_ReadReg(dev, ACT43750_REG_DACG_MAX, &dev->dacg_max);
    if (status != HAL_OK) {
        return status;
    }

    status = ACT43750_ReadDACG_Value(dev, &dev->dacg_val);
    if (status != HAL_OK) {
        return status;
    }

    dev->gate_voltage = ACT43750_DACGValToVoltage(dev->dacg_val);

    return HAL_OK;
}

/* Read a register from ACT43750 */
HAL_StatusTypeDef ACT43750_ReadReg(act43750_t *dev, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef status;

    if (dev == NULL || val == NULL || dev->hi2c == NULL) {
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Read(dev->hi2c,
                              dev->i2c_address,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              val,
                              1,
							  I2C_TIMEOUT);
    return status;
}

/* Write a register to ACT43750 */
HAL_StatusTypeDef ACT43750_WriteReg(act43750_t *dev, uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef status;

    if (dev == NULL || dev->hi2c == NULL) {
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Write(dev->hi2c,
                               dev->i2c_address,
                               reg,
                               I2C_MEMADD_SIZE_8BIT,
                               &val,
                               1,
							   I2C_TIMEOUT);
    return status;
}

/* Get current device state
 * Reads state register (0x00) and decodes:
 * Bits [3:0]: State machine state
 * Bit [4]: Calibration comparator status
 * Bits [6:5]: RT detection status
 */
HAL_StatusTypeDef ACT43750_GetState(act43750_t *dev, act43750_state_info_t *state)
{
    HAL_StatusTypeDef status;
    uint8_t reg_val;

    if (dev == NULL || state == NULL) {
        return HAL_ERROR;
    }

    status = ACT43750_ReadReg(dev, ACT43750_REG_STATE, &reg_val);
    if (status != HAL_OK) {
        return status;
    }

    ACT43750_UpdateStateInfo(dev, reg_val);

    memcpy(state, &dev->state, sizeof(act43750_state_info_t));

    return HAL_OK;
}

/* Set gate voltage (REGG output)
 * Gate voltage calculation:
 * VG = -2 × VDACG (gate voltage is negative 2x the DAC voltage)
 * VDACG = 0.75V + (DACG_val × 366.2µV)
 * DACG_val = (|VG|/2 - 0.75V) / 366.2µV
 * Valid range: -1500mV to -6000mV
 */
HAL_StatusTypeDef ACT43750_SetGateVoltage(act43750_t *dev, int16_t vg)
{
    HAL_StatusTypeDef status;
    uint16_t dacg_val;

    if (dev == NULL) {
        return HAL_ERROR;
    }

    if (vg > ACT43750_VG_MIN || vg < ACT43750_VG_MAX) {
        return HAL_ERROR;
    }

    dacg_val = ACT43750_VoltageToDACGVal(vg);

    status = ACT43750_WriteDACG_Value(dev, dacg_val);
    if (status != HAL_OK) {
        return status;
    }

    dev->dacg_val = dacg_val;
    dev->gate_voltage = vg;

    return HAL_OK;
}

/* Enable drain voltage (DSW50 block)
 * The IC uses the logic OR of the ENTX pin and the I2C registers to enable DSW50S
 * Writing 1 to EN_DSW50 bit enables the drain voltage.
 * Safety check: REGG should be enabled first (gate voltage before drain voltage)
 */
HAL_StatusTypeDef ACT43750_SetDrainVoltage(act43750_t *dev)
{
    HAL_StatusTypeDef status;
    uint8_t state_reg;

    if (dev == NULL) {
        return HAL_ERROR;
    }

    status = ACT43750_ReadReg(dev, ACT43750_REG_STATE, &state_reg);
    if (status != HAL_OK) {
        return status;
    }

    act43750_state_t current_state = (act43750_state_t)(state_reg & 0x0F);
    if (current_state < ACT43750_STATE_TX_READY) {
        return HAL_ERROR;
    }

    status = ACT43750_WriteReg(dev, ACT43750_REG_CONTROL, ACT43750_CTRL_EN_DSW50);
    if (status != HAL_OK) {
        return status;
    }

    dev->state.dsw50_enabled = true;

    return HAL_OK;
}

/* Enable REGG block (negative gate voltage regulator)
 * Enable the REGG block by writing a 1 into the EN_REGG bit
 * This starts the gate voltage ramp sequence.
 */
HAL_StatusTypeDef ACT43750_EnableREGG(act43750_t *dev)
{
    HAL_StatusTypeDef status;

    if (dev == NULL) {
        return HAL_ERROR;
    }

    status = ACT43750_WriteReg(dev, ACT43750_REG_CONTROL, ACT43750_CTRL_EN_REGG);
    if (status != HAL_OK) {
        return status;
    }

    dev->state.regg_enabled = true;

    return HAL_OK;
}

/* Disable REGG block */
HAL_StatusTypeDef ACT43750_DisableREGG(act43750_t *dev)
{
    HAL_StatusTypeDef status;

    if (dev == NULL) {
        return HAL_ERROR;
    }

    if (dev->state.dsw50_enabled) {
        status = ACT43750_DisableDrainVoltage(dev);
        if (status != HAL_OK) {
            return status;
        }
    }

    status = ACT43750_WriteReg(dev, ACT43750_REG_CONTROL, ACT43750_CTRL_DIS_REGG);
    if (status != HAL_OK) {
        return status;
    }

    dev->state.regg_enabled = false;

    return HAL_OK;
}

/* Disable drain voltage (DSW50 block) */
HAL_StatusTypeDef ACT43750_DisableDrainVoltage(act43750_t *dev)
{
    HAL_StatusTypeDef status;

    if (dev == NULL) {
        return HAL_ERROR;
    }

    status = ACT43750_WriteReg(dev, ACT43750_REG_CONTROL, ACT43750_CTRL_DIS_DSW50);
    if (status != HAL_OK) {
        return status;
    }

    dev->state.dsw50_enabled = false;

    return HAL_OK;
}

/* Start calibration routine
 * The calibration routine can be started by writing a 1 into the RunCal bit
 * The IC automatically finds optimal gate voltage for target Idq.
 */
HAL_StatusTypeDef ACT43750_StartCalibration(act43750_t *dev)
{
    HAL_StatusTypeDef status;

    /* Parameter validation */
    if (dev == NULL) {
        return HAL_ERROR;
    }

    status = ACT43750_WriteReg(dev, ACT43750_REG_CONTROL, ACT43750_CTRL_RUN_CAL);
    if (status != HAL_OK) {
        return status;
    }

    dev->state.calibration_active = true;

    return HAL_OK;
}

/* Check if calibration is complete
 * Calibration is complete when state machine exits RSCAL_CAL state
 */
HAL_StatusTypeDef ACT43750_IsCalibrationComplete(act43750_t *dev, uint8_t *is_complete)
{
    HAL_StatusTypeDef status;
    act43750_state_info_t state;

    if (dev == NULL || is_complete == NULL) {
        return HAL_ERROR;
    }

    status = ACT43750_GetState(dev, &state);
    if (status != HAL_OK) {
        return status;
    }

    *is_complete = (state.current_state != ACT43750_STATE_RSCAL_CAL);

    if (*is_complete) {
        dev->state.calibration_active = false;

        uint16_t new_val;
        status = ACT43750_ReadDACG_Value(dev, &new_val);
        if (status == HAL_OK) {
            dev->dacg_val = new_val;
            dev->gate_voltage = ACT43750_DACGValToVoltage(new_val);
        }
    }

    return HAL_OK;
}

/* Get current gate voltage */
HAL_StatusTypeDef ACT43750_GetGateVoltage(act43750_t *dev, int16_t *vg)
{
    HAL_StatusTypeDef status;
    uint16_t dacg_val;

    if (dev == NULL || vg == NULL) {
        return HAL_ERROR;
    }

    status = ACT43750_ReadDACG_Value(dev, &dacg_val);
    if (status != HAL_OK) {
        return status;
    }

    *vg = ACT43750_DACGValToVoltage(dacg_val);

    dev->dacg_val = dacg_val;
    dev->gate_voltage = *vg;

    return HAL_OK;
}

/* Read interrupt status registers */
HAL_StatusTypeDef ACT43750_ReadInterrupts(act43750_t *dev,
                                          uint8_t *int_status_1,
                                          uint8_t *int_status_2,
                                          uint8_t *int_status_3)
{
    HAL_StatusTypeDef status;

    if (dev == NULL) {
        return HAL_ERROR;
    }

    if (int_status_1 != NULL) {
        status = ACT43750_ReadReg(dev, ACT43750_REG_INTERRUPT_STATUS_1, int_status_1);
        if (status != HAL_OK) {
            return status;
        }
    }

    if (int_status_2 != NULL) {
        status = ACT43750_ReadReg(dev, ACT43750_REG_INTERRUPT_STATUS_2, int_status_2);
        if (status != HAL_OK) {
            return status;
        }
    }

    if (int_status_3 != NULL) {
        status = ACT43750_ReadReg(dev, ACT43750_REG_INTERRUPT_STATUS_3, int_status_3);
        if (status != HAL_OK) {
            return status;
        }
    }

    return HAL_OK;
}

/* Clear interrupt flags */
HAL_StatusTypeDef ACT43750_ClearInterrupts(act43750_t *dev)
{
    uint8_t dummy;
    return ACT43750_ReadInterrupts(dev, &dummy, &dummy, &dummy);
}

/* Set DACG minimum clamp */
HAL_StatusTypeDef ACT43750_SetDACGMin(act43750_t *dev, uint8_t dacg_min_8bit)
{
    HAL_StatusTypeDef status;

    if (dev == NULL) {
        return HAL_ERROR;
    }

    status = ACT43750_WriteReg(dev, ACT43750_REG_DACG_MIN, dacg_min_8bit);
    if (status == HAL_OK) {
        dev->dacg_min = dacg_min_8bit;
    }

    return status;
}

/* Set DACG maximum clamp */
HAL_StatusTypeDef ACT43750_SetDACGMax(act43750_t *dev, uint8_t dacg_max_8bit)
{
    HAL_StatusTypeDef status;

    if (dev == NULL) {
        return HAL_ERROR;
    }

    status = ACT43750_WriteReg(dev, ACT43750_REG_DACG_MAX, dacg_max_8bit);
    if (status == HAL_OK) {
        dev->dacg_max = dacg_max_8bit;
    }

    return status;
}

/* Check I2C communication */
static HAL_StatusTypeDef ACT43750_CheckCommunication(act43750_t *dev)
{
    if (dev == NULL || dev->hi2c == NULL) {
        return HAL_ERROR;
    }

    return HAL_I2C_IsDeviceReady(dev->hi2c,
                                 dev->i2c_address,
                                 3,
								 I2C_TIMEOUT);
}

/* Perform software reset sequence */
HAL_StatusTypeDef ACT43750_SoftReset(act43750_t *dev)
{
    HAL_StatusTypeDef status;

    if (dev == NULL) {
        return HAL_ERROR;
    }

    status = ACT43750_DisableDrainVoltage(dev);
    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(10);

    status = ACT43750_DisableREGG(dev);
    if (status != HAL_OK) {
        return status;
    }

    dev->state.regg_enabled = false;
    dev->state.dsw50_enabled = false;
    dev->state.calibration_active = false;

    return HAL_OK;
}

/* Convert gate voltage (mV) to DACG 13-bit val
 * VG = -2 × VDACG
 * DACG_val = (VDACG - 0.75V) / 366.2µV
 */
static uint16_t ACT43750_VoltageToDACGVal(int16_t vg)
{
    int32_t vdacg;
    int16_t dacg_val;

    vdacg = ((-vg) * 1000) / 2;

    dacg_val = (vdacg - 750000) / ACT43750_DACG_STEP;

    if (dacg_val < ACT43750_DACG_MIN_REG_VAL) {
        dacg_val = ACT43750_DACG_MIN_REG_VAL;
    }
    if (dacg_val > ACT43750_DACG_MAX_REG_VAL) {
        dacg_val = ACT43750_DACG_MAX_REG_VAL;
    }

    return (uint16_t)dacg_val;
}

/* Convert DACG 13-bit val to gate voltage (mV)
 * VDACG = 0.75V + (val × 366.2µV)
 * VG = -2 × VDACG
 */
static int16_t ACT43750_DACGValToVoltage(uint16_t dacg_val)
{
    int32_t vdacg;
    int32_t vg;

    if (dacg_val < ACT43750_DACG_MIN_REG_VAL) {
		dacg_val = ACT43750_DACG_MIN_REG_VAL;
	}
	if (dacg_val > ACT43750_DACG_MAX_REG_VAL) {
		dacg_val = ACT43750_DACG_MAX_REG_VAL;
	}

    vdacg = 750000 + ((int32_t)dacg_val * ACT43750_DACG_STEP);

    vg = -(vdacg * 2) / 1000;

    return (int16_t)vg;
}

/* Read DACG 13-bit val from registers
 * Register 0x07h [7:0]: DACG upper 8 bits (UPPER MSB)
 * Register 0x06h [4:0]: DACG lower 5 bits (LOWER MSB)
 * 13-bit val = (LOWER MSB << 5) | UPPER MSB
 */
static HAL_StatusTypeDef ACT43750_ReadDACG_Value(act43750_t *dev, uint16_t *dacg_val)
{
    HAL_StatusTypeDef status;
    uint8_t upper_msb, lower_msb;

    status = ACT43750_ReadReg(dev, ACT43750_REG_DACG_UPPER_MSB, &upper_msb);
    if (status != HAL_OK) {
        return status;
    }

    status = ACT43750_ReadReg(dev, ACT43750_REG_DACG_LOWER_MSB, &lower_msb);
    if (status != HAL_OK) {
        return status;
    }

    *dacg_val = (((uint16_t)lower_msb << 8) & 0xFF00) | (upper_msb & 0xFF);

    return HAL_OK;
}

/* Write DACG 13-bit val to registers
 * The register values are only transferred to the DAC when a 1 is written
 * into the DACG_Load bit. The IC automatically writes a 1 into DACG_Load
 * every time a new value is written into register 0x07h.
 * Sequence:
 * 1. Write LSB to register 0x06h
 * 2. Write MSB to register 0x07h (automatically triggers load)
 */
static HAL_StatusTypeDef ACT43750_WriteDACG_Value(act43750_t *dev, uint16_t dacg_val)
{
    HAL_StatusTypeDef status;
    uint8_t upper_msb, lower_msb;

    if (dacg_val < ACT43750_DACG_MIN_REG_VAL) {
		dacg_val = ACT43750_DACG_MIN_REG_VAL;
	}
	if (dacg_val > ACT43750_DACG_MAX_REG_VAL) {
		dacg_val = ACT43750_DACG_MAX_REG_VAL;
	}

	upper_msb = (uint8_t)(dacg_val & 0xFF);
	lower_msb = (uint8_t)((dacg_val>>8) & 0x1F);

    status = ACT43750_WriteReg(dev, ACT43750_REG_DACG_UPPER_MSB, upper_msb);
    if (status != HAL_OK) {
        return status;
    }

    status = ACT43750_WriteReg(dev, ACT43750_REG_DACG_LOWER_MSB, lower_msb);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

/* Update device state information from state register
 * Decodes state register (0x00):
 * Bits [3:0]: State machine state
 * Bit [4]: Calibration comparator (1 = threshold reached)
 * Bits [6:5]: RT detection (00 = detected, others = error)
 */
static void ACT43750_UpdateStateInfo(act43750_t *dev, uint8_t state_reg)
{
    dev->state.current_state = (act43750_state_t)(state_reg & 0x0F);

    dev->state.regg_enabled = (dev->state.current_state >= ACT43750_STATE_REGG_RAMP);
    dev->state.dsw50_enabled = (dev->state.current_state == ACT43750_STATE_DSW50_ON);
    dev->state.calibration_active = (dev->state.current_state == ACT43750_STATE_RSCAL_CAL);
    dev->state.power_good = (dev->state.current_state == ACT43750_STATE_TX_READY ||
                             dev->state.current_state == ACT43750_STATE_DSW50_ON);

    dev->state.fault_flags = 0;
    if (dev->state.current_state == ACT43750_STATE_BIAS_ERROR ||
        dev->state.current_state == ACT43750_STATE_ERROR_REGG_MIN) {
        dev->state.fault_flags = 1;
    }
}

