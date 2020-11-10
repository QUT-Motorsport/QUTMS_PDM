/** @file module.h
 *
 * @brief A description of the module’s purpose.
 *
 * @par
 */

#ifndef BTS7XX_H
#define BTS7XX_H

#include "spi.h"

/** Diagnostic Registers
 * WRNDIAG - Warning diagnosis
 * STDDIAG - Standard diagnosis
 * ERRDIAG - Error diagnosis
 */

/** Configuration Registers
 * OUT - Output configuration
 * RCS - Restart counter status (read only)
 * OCR - Over-current threshold configuration
 * RCD - Restart counter disable
 * KRC - KILIS range control
 * SRC - Slew rate control register (read only)
 * HWCR - Hardware configuration
 * ICS - Input status & checksum input
 * PCS - Parallel channel and slew rate control
 * DCR - Diagnostic configuration and swap bit
 */

// Configuration Registers Read Commands
#define BTS7XX_READ_OUT_COMMAND 		0x00
#define BTS7XX_READ_RCS_COMMAND 		0x08
#define BTS7XX_READ_SRC_COMMAND 		0x09
#define BTS7XX_READ_OCR_COMMAND 		0x04
#define BTS7XX_READ_RCD_COMMAND 		0x0C
#define BTS7XX_READ_KRC_COMMAND 		0x05
#define BTS7XX_READ_PCS_COMMAND 		0x0D
#define BTS7XX_READ_HWCR_COMMAND 		0x06
#define BTS7XX_READ_ICS_COMMAND 		0x0E
#define BTS7XX_READ_DCR_COMMAND 		0x07

// Configuration Registers Write Commands
#define BTS7XX_WRITE_OUT_COMMAND		0x80
#define BTS7XX_WRITE_OCR_COMMAND		0xC0
#define BTS7XX_WRITE_RCD_COMMAND		0xC0
#define BTS7XX_WRITE_KRC_COMMAND		0xD0
#define BTS7XX_WRITE_PCS_COMMAND		0xD0
#define BTS7XX_WRITE_HWCR_COMMAND		0xE0
#define BTS7XX_WRITE_ICS_COMMAND		0xE0
#define BTS7XX_WRITE_DCR_COMMAND 		0xF0

#define BTS7XX_WRITE_OUT_CH1			0x01
#define BTS7XX_WRITE_OUT_CH2			0x02
#define BTS7XX_WRITE_OUT_CH3			0x04
#define BTS7XX_WRITE_OUT_CH4			0x08

typedef enum {
	BTS7XX_OUT_CH1 = 0x01,
	BTS7XX_OUT_CH2 = 0x02,
	BTS7XX_OUT_CH3 = 0x04,
	BTS7XX_OUT_CH4 = 0x08
} BTS7XX_OUT_CHANNELS;

// Diagnostic Register Read Commands
#define BTS7XX_REG_READ_WRNDIAG 		0x01
#define BTS7XX_REG_READ_STDDIAG 		0x02
#define BTS7XX_REG_READ_ERRDIAG 		0x03

typedef struct _BTS7XX_OCR_OBJ {
	uint8_t OCR_OCT0 : 1;
	uint8_t OCR_OCT1 : 1;
	uint8_t OCR_OCT2 : 1;
	uint8_t OCR_OCT3 : 1;
} BTS7XX_OCR_OBJ;

// Decode diagnostic responses


// Register definitions
// #define BTS7XX_REG_ADDR0_OUT
#define BTS7XX_REG_ADDR1_OUT      0x00

//#define BTS7XX_REG_ADDR0_RCS
#define BTS7XX_REG_ADDR1_RCS      0x08 // 1000

//#define BTS7XX_REG_ADDR0_SRC
#define BTS7XX_REG_ADDR1_SRC      0x09 // 1001

#define BTS7XX_REG_ADDR0_OCR      0x00
#define BTS7XX_REG_ADDR1_OCR      0x04 // 0100

#define BTS7XX_REG_ADDR0_RCD      0x00
#define BTS7XX_REG_ADDR1_RCD      0x0C // 1100

#define BTS7XX_REG_ADDR0_KRC      0x01
#define BTS7XX_REG_ADDR1_KRC      0x05 // 0101

#define BTS7XX_REG_ADDR0_PCS      0x01
#define BTS7XX_REG_ADDR1_PCS      0x0D // 1101

#define BTS7XX_REG_ADDR0_HWCR     0x02
#define BTS7XX_REG_ADDR1_HWCR     0x06 // 0110

#define BTS7XX_REG_ADDR0_ICS      0x02
#define BTS7XX_REG_ADDR1_ICS      0x0E // 1110

#define BTS7XX_REG_ADDR0_DCR      0x03
#define BTS7XX_REG_ADDR1_DCR      0x07 // x111

// OPEN LOAD DETECTION
// LIMP HOME MODE ACTIVATED

//0x02
//0x40

int8_t max8(int8_t num1, int8_t num2);

uint8_t BTS7XX_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t command, uint8_t *data, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin);

uint8_t BTS7XX_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t command, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin);

#endif /* BTS7XX_H */

/***end of file***/
