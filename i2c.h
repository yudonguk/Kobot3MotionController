/*
 * i2c.h
 *
 *  Created on: 2012. 8. 7.
 *      Author: East
 */

#ifndef I2C_H_
#define I2C_H_

#include "type.h"
#include "inttypes.h"

void I2cRccConfig();
void I2cNvicConfig();
void I2cInit(uint32_t clockSpeed);

BOOL I2c2WriteRequest(uint16_t address, uint8_t* data, uint32_t size);
BOOL I2c2ReadRequest(uint16_t address, uint8_t* data, uint32_t availableSize);

BOOL IsI2c2OnGoing();
BOOL IsI2c2Completed();
BOOL IsI2c2Error();

#endif /* I2C_H_ */
