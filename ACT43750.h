/*
 * ACT43750.h
 *
 *  Created on: Feb 7, 2026
 *      Author: Admin
 */

#ifndef INC_ACT43750_H_
#define INC_ACT43750_H_

#include "main.h"

#define ACT43750_I2C_ADDR           0x40  // 8-bit address

#define I2C_TIMEOUT             	1

/* Based on datasheet register map */
#define ACT43750_REG_STATE                  0x00  // State register
#define ACT43750_REG_INTERRUPT_STATUS_1     0x01  // Interrupt status 1
#define ACT43750_REG_INTERRUPT_STATUS_2     0x02  // Interrupt status 2
#define ACT43750_REG_INTERRUPT_STATUS_3     0x03  // Interrupt status 3
#define ACT43750_REG_DACG_LOAD              0x04  // DACG load control
#define ACT43750_REG_DACG_ADJUST            0x05  // DACG adjustment
#define ACT43750_REG_DACG_UPPER_MSB         0x06  // DACG[4:0] lower 5 bits
#define ACT43750_REG_DACG_LOWER_MSB         0x07  // DACG[12:5] upper 8 bits
#define ACT43750_REG_CONTROL                0x08  // Control register
#define ACT43750_REG_INTERRUPT_MASK_1       0x09  // Interrupt mask 1
#define ACT43750_REG_INTERRUPT_MASK_2       0x0A  // Interrupt mask 2
#define ACT43750_REG_GREF_CONTROL           0x0E  // GREF control
#define ACT43750_REG_LSR_VOLTAGE            0x0F  // LSR voltage setting
#define ACT43750_REG_CONFIG_1               0x10  // Configuration 1
#define ACT43750_REG_CONFIG_2               0x11  // Configuration 2
#define ACT43750_REG_DEADTIME               0x12  // Dead-time control
#define ACT43750_REG_IDQ_ADJUST             0x13  // IDQ calibration adjustment
#define ACT43750_REG_DACG_MIN               0x14  // DACG minimum clamp
#define ACT43750_REG_DACG_MAX               0x15  // DACG maximum clamp
#define ACT43750_REG_I2C_MASTER_CONFIG      0x17  // I2C master configuration

/* Control Register  */
#define ACT43750_CTRL_DIS_DSW50             (1 << 0)  // Disable DSW50
#define ACT43750_CTRL_EN_DSW50              (1 << 1)  // Enable DSW50
#define ACT43750_CTRL_RUN_CAL               (1 << 3)  // Run calibration
#define ACT43750_CTRL_DIS_REGG              (1 << 6)  // Disable REGG
#define ACT43750_CTRL_EN_REGG               (1 << 7)  // Enable REGG

/*DACG Reference Voltage Constants */
#define ACT43750_DACG_MIN_VOLTAGE        750   // Minimum DACG voltage (0.75V)
#define ACT43750_DACG_MAX_VOLTAGE        3000  // Maximum DACG voltage (3.0V)
#define ACT43750_DACG_STEP               366.2   // Step size in microvolts (366.2µV)
#define ACT43750_DACG_MIN_REG_VAL        0x0400 // Minimum 13-bit val
#define ACT43750_DACG_MAX_REG_VAL        0x1FFF // Maximum 13-bit val

/*Gate Voltage Range (VG = -2 × VDACG)*/
#define ACT43750_VG_MIN                 (-ACT43750_DACG_MIN_VOLTAGE*2) // -1.5V
#define ACT43750_VG_MAX                 (-ACT43750_DACG_MAX_VOLTAGE*2) // -6.0V

/*State Machine States (Register 0x00 bits [3:0])*/
typedef enum {
    ACT43750_STATE_NO_POWER         = 0x00,  // No power state
    ACT43750_STATE_BIAS_RAMP        = 0x01,  // Bias ramping
    ACT43750_STATE_BIAS_READY       = 0x02,  // Bias ready
    ACT43750_STATE_REGG_RAMP        = 0x03,  // REGG ramping
    ACT43750_STATE_REG50_RAMP       = 0x04,  // REG50 ramping
    ACT43750_STATE_TX_READY         = 0x05,  // Ready for transmission
    ACT43750_STATE_RSCAL_CAL        = 0x06,  // Calibration in progress
    ACT43750_STATE_REGG_TO_MIN      = 0x07,  // REGG going to minimum
    ACT43750_STATE_REGG_PROG        = 0x08,  // REGG programming
    ACT43750_STATE_DSW50_ON         = 0x09,  // DSW50 is ON
    ACT43750_STATE_ERROR_REGG_MIN   = 0x0A,  // Error - REGG at minimum
    ACT43750_STATE_V50_DISCHARGE    = 0x0B,  // V50 discharging
    ACT43750_STATE_REGG_SHUTDOWN    = 0x0C,  // REGG shutting down
    ACT43750_STATE_BIAS_ERROR       = 0x0D,  // Bias error state
    ACT43750_STATE_UNKNOWN          = 0xFF   // Unknown/invalid state
} act43750_state_t;

/*Device State Information Structure contains decoded state machine status and flags*/
typedef struct {
    act43750_state_t current_state;         // Current state machine state
    uint8_t regg_enabled;                   // REGG block enabled
    uint8_t dsw50_enabled;                  // DSW50 block enabled
    uint8_t power_good;                     // Power good status
    uint8_t calibration_active;             // Calibration in progress
    uint8_t fault_flags;                    // Fault status flags
} act43750_state_info_t;

/* ACT43750 Device Structure Contains all runtime data for the device */
typedef struct {
    I2C_HandleTypeDef *hi2c;             // I2C handle pointer
    uint8_t i2c_address;                 // Device I2C address (8-bit)

    uint16_t dacg_val;                   // Current DACG 13-bit val
    int16_t gate_voltage;                // Current gate voltage in mV
    uint8_t dacg_min;                    // DACG minimum clamp
    uint8_t dacg_max;                    // DACG maximum clamp

    act43750_state_info_t state;         // Device state information
} act43750_t;


/* Initializes and verify I2C communication of ACT43750 */
HAL_StatusTypeDef ACT43750_Init(act43750_t *dev, I2C_HandleTypeDef *hi2c);

/* Read a register from ACT43750 */
HAL_StatusTypeDef ACT43750_ReadReg(act43750_t *dev, uint8_t reg, uint8_t *val);

/* Write a register to ACT43750 */
HAL_StatusTypeDef ACT43750_WriteReg(act43750_t *dev, uint8_t reg, uint8_t val);

/* Get current device state */
HAL_StatusTypeDef ACT43750_GetState(act43750_t *dev, act43750_state_info_t *state);

/* Set gate voltage (REGG output) */
HAL_StatusTypeDef ACT43750_SetGateVoltage(act43750_t *dev, int16_t vg);

/* Enable drain voltage (DSW50 block) */
HAL_StatusTypeDef ACT43750_SetDrainVoltage(act43750_t *dev);

/* Enable REGG block (negative gate voltage regulator) */
HAL_StatusTypeDef ACT43750_EnableREGG(act43750_t *dev);

/* Disable REGG block */
HAL_StatusTypeDef ACT43750_DisableREGG(act43750_t *dev);

/* Disable drain voltage (DSW50 block) */
HAL_StatusTypeDef ACT43750_DisableDrainVoltage(act43750_t *dev);

/* Start calibration routine for perfect gate voltage req for idq*/
HAL_StatusTypeDef ACT43750_StartCalibration(act43750_t *dev);

/* To check ACT43750 exits calibration routine */
HAL_StatusTypeDef ACT43750_IsCalibrationComplete(act43750_t *dev, uint8_t *is_complete);

/* Get current gate voltage */
HAL_StatusTypeDef ACT43750_GetGateVoltage(act43750_t *dev, int16_t *vg);

/* Read interrupt status if power good is not valid */
HAL_StatusTypeDef ACT43750_ReadInterrupts(act43750_t *dev,
                                          uint8_t *int_status_1,
                                          uint8_t *int_status_2,
                                          uint8_t *int_status_3);

/* Clear interrupt flags */
HAL_StatusTypeDef ACT43750_ClearInterrupts(act43750_t *dev);

/* Reset ACT43750 */
HAL_StatusTypeDef ACT43750_SoftwareReset(act43750_t *dev);

/* Set DACG minimum clamp */
HAL_StatusTypeDef ACT43750_SetDACGMin(act43750_t *dev, uint8_t dacg_min_8bit);

/* Set DACG maximum clamp */
HAL_StatusTypeDef ACT43750_SetDACGMax(act43750_t *dev, uint8_t dacg_max_8bit);
#endif /* INC_ACT43750_H_ */
