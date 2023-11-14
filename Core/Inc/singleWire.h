/*
 * singleWire.h
 *
 *  Created on: 15-Oct-2023
 *      Author: AutoWires
 */

#ifndef INC_SINGLEWIRE_H_
#define INC_SINGLEWIRE_H_

// Define a structure to hold your data
typedef struct {
    uint8_t *dataArrPtr;
    uint8_t cnt;
    // Add more fields if needed
} singleWireData_t;

void StartSingleWireTask(void *argument);

#endif /* INC_SINGLEWIRE_H_ */
