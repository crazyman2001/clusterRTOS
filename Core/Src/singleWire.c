/*
 * singleWire.c
 *
 *  Created on: 15-Oct-2023
 *      Author: AutoWires
 */
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "Display.h"
#include "singleWire.h"
#include "event_groups.h"

//  declare a event grounp handler variable
extern EventGroupHandle_t  xEventGroupDisplay;

// declare queue to get data from single wire communication
extern QueueHandle_t xQueueSingleWire;

extern uint32_t divDis;
volatile uint16_t adcSpeed = 0, swSpeed = 0, speedVarF = 0;
extern uint32_t tLimit, tcnt;
volatile uint8_t gGear, parking, gr, gInfo, wnSymbol, bitMotor, bitECU;
extern _Bool runTime;

/* USER CODE BEGIN Header_StartSingleWireTask */
/**
  * @brief  Function implementing the Single wire communication thread.
  * @param  argument: array pointer with recived data
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartSingleWireTask(void *argument)
{
	singleWireData_t myData;
	EventBits_t bitsToSet;
	uint8_t bit0, bit1, bit2, bit3, bit4, bit5, bit6, bit7, bit8, bit9, bitMotor;
  /* Infinite loop */
  for(;;)
  {
	  if (xQueueReceive(xQueueSingleWire, &myData, portMAX_DELAY) == pdTRUE) {
		  // Data received successfully, process it
		  // receivedData now contains the data sent from the ISR
			bit0= *myData.dataArrPtr++;//-dataVar[10];
			//if (dataVar[0]<dataVar[10])
			//	bit0+=0xfe;
			bit1= *myData.dataArrPtr++;;//-dataVar[10];
			//if (dataVar[1]<dataVar[10])
			//	bit1+=0xfe;
			bit2= *myData.dataArrPtr++;//-dataVar[10];
			//if (dataVar[2]<dataVar[10])
			//	bit2+=0xfe;
			bit3= *myData.dataArrPtr++;
			uint8_t vers = *(myData.dataArrPtr+6);
			if (bit3 < vers)
				bit3+=0xfe;
			bit4= *myData.dataArrPtr++ - vers;
			if (bit4 < vers)
				bit4+=0xfe;
			bit5= *myData.dataArrPtr++ - vers;
			if (bit5 < vers)
				bit5+=0xfe;
			bit6= *myData.dataArrPtr++;//-dataVar[10];
			//if (dataVar[6]<dataVar[10])
			//	bit6+=0xfe;
			bit7= *myData.dataArrPtr++ - vers;
			if (bit7 < vers)
				bit7+=0xfe;
			bit8= *myData.dataArrPtr++ - vers;
			if (bit8 < vers)
				bit8+=0xfe;
			bit9= *myData.dataArrPtr++ - vers;
			if (bit9 < vers)
				bit9+=0xfe;
			bitsToSet = 0;
			//speed calculation
			adcSpeed = ((bit7<<8)|bit8);
			swSpeed	= adcSpeed/17;
			speedVarF = swSpeed;//(swSpeed*grm)/50;
			if(speedVarF >= 2)
			{
				if(speedVarF>199)
					speedVarF = 199;
				tLimit=(divDis*20)/speedVarF;      //3600 actual
			}
			else
			{
				speedVarF = 0;
				tLimit = (divDis*20);
				runTime=1;
				tcnt=0;
			}

			//read parking data
			parking = (((bit3)&0x80)||((bit2)&0x0B));
			if(parking)
			{
				bitsToSet |= PARK_BIT;
			}
			else
			{
				bitsToSet |= SPEED_BIT;
			}

			gr = (bit4)&0x03;
			if ((bit4)&0x80)
				gGear=4;
			else
				gGear=gr;
			if((bit5)&0x04)  // Reverse Gear
			{
				bitsToSet |= REV_BIT;
			}
			else if((bit3)&0x04)
			{
				bitsToSet |= CURISE_BIT;
			}
			else if((bit5)&0x01)	//Gear 4
			{
				bitsToSet |= ECHO_BIT;
			}
			else if ((gGear >= 1)&&(gGear <= 4))
			{
				bitsToSet |= GEAR_BIT;
			}
			if(((bit5)&0x02)|((bit4)&0x20))    // Brake
			{
				bitsToSet |= WARN_BIT;
			}
			if(bit3&0x20)
			{
				bitsToSet |= THROTLE_BIT;
			}
			if(bit3&0x41)									//0x40
			{
				bitsToSet |= MOTOR_BIT;
			}
			if((bit5)&0x42||(bit4)&0x20)  //Side Stand
			{
				bitsToSet |= SIDE_BIT;
			}
			if((bit3)&0x08)
			{
				bitsToSet |= BAT_DEAD_BIT;
				bitECU = 1;
			}
			else
			{
				if(!(bit3&0x10))
					bitECU = 0;
			}
			if((bit3)&0x10)
			{
				bitsToSet |= ECU_BIT;
				bitECU = 1;
			}
			else
			{
				if (!(bit3&0x08))
					bitECU = 0;
			}
			if((bit5)&0x40)    // Side Stand
				wnSymbol|=0x04;

			xEventGroupSetBits(xEventGroupDisplay, bitsToSet);
	  }
  }
}
