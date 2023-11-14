#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "Display.h"
#include "singleWire.h"
#include "event_groups.h"
#include "can.h"

extern osSemaphoreId_t xSemCanDetectHandle;
extern EventGroupHandle_t  xEventGroupDisplay;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];
extern FDCAN_TxHeaderTypeDef TxHeader;
extern uint8_t TxData[8];

extern _Bool tmpH1,tmpH2, tmpH3, tmpH4, tmpH5, tmpH6, mTemp;
extern unsigned char CANbuf[7][7];
extern volatile unsigned int lineCount, CANindex, CANtype;
extern volatile uint8_t direction, spd, spdLH, emp1, emp2, mcx1, brkMod, gear, ntcCount, tBaud;
extern volatile uint8_t canBatFlag, canControllerFlag, controllerFault, batteryFault, motorFault, motorTemp, throtelFault, throttlePos;
extern volatile uint8_t chargeState, errorState, reGen, reverseMod, parkingMod, thrmlRun, sStand, brake, cruiseMod, Warning;
extern volatile uint32_t tcnt, tLimit, divDis, ahVar;
extern volatile uint16_t flt1, flt2, flt3, batteryFault16, soh, rAh, cAh, speedVar2, speedVar;
extern volatile uint16_t ntc1, ntc2, ntc3, ntc4, ntc5, ntc6, tempLimit, motorRpm, ctempAlert, mtempAlert;
extern volatile uint32_t odoVar, tripVar, volt, soc, odoVarC, tripVarC;
extern volatile int32_t amp;
extern int16_t amp2;
uint16_t eeCounter = 0;
uint8_t tBuf[200], odoSwVal=0;

void FDCAN_SetBaud(unsigned char baud)
{
	unsigned int bVar;
	//for(;cnt<=4;)
	{
		if(baud == 1)
			bVar = 125;
		else if(baud == 2)
			bVar = 250;
		else if(baud == 3)
			bVar = 500;
		else if(baud == 4)
			bVar = 1000;
	  hfdcan1.Instance = FDCAN1;
	  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
	  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	  hfdcan1.Init.AutoRetransmission = ENABLE;
	  hfdcan1.Init.TransmitPause = ENABLE;
	  hfdcan1.Init.ProtocolException = DISABLE;
	  hfdcan1.Init.NominalPrescaler = 1;
	  hfdcan1.Init.NominalSyncJumpWidth = 16;
	  hfdcan1.Init.NominalTimeSeg1 = ((32000/bVar)-1);//127;
	  hfdcan1.Init.NominalTimeSeg2 = (16000/bVar);//64;
	  hfdcan1.Init.DataPrescaler = 1;
	  hfdcan1.Init.DataSyncJumpWidth = 4;
	  hfdcan1.Init.DataTimeSeg1 = 5;
	  hfdcan1.Init.DataTimeSeg2 = 4;
	  hfdcan1.Init.StdFiltersNbr = 1;
	  hfdcan1.Init.ExtFiltersNbr = 1;
	  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
	  {
		//  cnt++;
		Error_Handler();
	  }
	  /*else
	  {
		  break;
	  }*/
	}
}

void FDCAN1_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig, sFilterConfig2;

	  /* Configure Rx filter */
	  sFilterConfig.IdType = FDCAN_EXTENDED_ID;//FDCAN_STANDARD_ID;//
	  sFilterConfig.FilterIndex = 0;
	  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;//_NO_EIDM;//FDCAN_FILTER_RANGE;//FDCAN_FILTER_MASK;//
	  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	  sFilterConfig.FilterID1 = 0x01;//0x108109a
	  sFilterConfig.FilterID2 = 0x1FFFFFFF;//0x7ff;//
	  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sFilterConfig2.IdType = FDCAN_STANDARD_ID;//FDCAN_EXTENDED_ID;//
	  sFilterConfig2.FilterIndex = 0;
	  sFilterConfig2.FilterType = FDCAN_FILTER_RANGE;//FDCAN_FILTER_RANGE_NO_EIDM;//FDCAN_FILTER_MASK;
	  sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	  sFilterConfig2.FilterID1 = 0x01;//0x108109a
	  sFilterConfig2.FilterID2 = 0x7ff;//
	  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig2) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Configure global filter:
	     Filter all remote frames with STD and EXT ID
	     Reject non matching frames with STD ID and EXT ID */
	  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Start the FDCAN module */
	  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Prepare Tx Header */
	  TxHeader.Identifier = 0x321;
	  TxHeader.IdType = FDCAN_STANDARD_ID;//FDCAN_EXTENDED_ID;//
	  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	  TxHeader.MessageMarker = 0;

	//  TxHeader.Identifier = 0x321;
	  /* Prepare Rx Header */
		  /*RxHeader.IdType = FDCAN_EXTENDED_ID;
	  	  RxHeader.RxFrameType = FDCAN_DATA_FRAME;
	  	  RxHeader.DataLength = FDCAN_DLC_BYTES_8;
	  	  RxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	  	  RxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	  	  RxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	  	  //RxHeader.RxEventFifoControl = FDCAN_NO_RX_EVENTS;
	  	  //RxHeader.MessageMarker = 0;*/
}

uint64_t getMask(uint8_t maskVar, uint8_t strt, _Bool end)
{
	uint64_t retMask = 0, shift;
	if(end)
	{
		shift = 0x8000000000000000;
		shift >>= strt;
		for(uint8_t itr = 0; itr < maskVar; itr++)
		{
			retMask >>= 1;
			retMask |= shift;
		}
	}
	else
	{
		shift = 0x01;
		shift <<= strt;
		for(uint8_t itr = 0; itr < maskVar; itr++)
		{
			retMask <<= 1;
			retMask |= shift;
		}
	}
	return retMask;
}

void canParseFun(uint16_t id, uint32_t canValue)
{
	switch( id )
	{
	case SOC1:
		soc = canValue;
		xEventGroupSetBits(xEventGroupDisplay, SOC_DISP_BIT1);
		break;
	case SOC2:
		soc = canValue;
		soc /= 10;
		xEventGroupSetBits(xEventGroupDisplay, SOC_DISP_BIT2);
		break;
	case SOC3:
		soc = canValue;
		soc /= 100;
		xEventGroupSetBits(xEventGroupDisplay, SOC_DISP_BIT2);
		break;
	case SOC4:
		soc = canValue;
		xEventGroupSetBits(xEventGroupDisplay, SOC_DISP_BIT2);
		break;
	case SOC5:
		soc = canValue;
		soc /= 10;
		xEventGroupSetBits(xEventGroupDisplay, SOC_DISP_BIT3);
		break;
	case 3:
		cAh = (uint16_t)canValue;
		xEventGroupSetBits(xEventGroupDisplay, CAH_BIT);
		break;
	case VOLT1:
		volt = canValue;
		xEventGroupSetBits(xEventGroupDisplay, VOLT_BIT1);
		break;
	case VOLT2:
		volt = canValue;
		volt /= 10;
		xEventGroupSetBits(xEventGroupDisplay, VOLT_BIT1);
		break;
	case VOLT3:
		volt = canValue;
		volt /= 100;
		xEventGroupSetBits(xEventGroupDisplay, VOLT_BIT1);
		break;
	case AMP1:
		amp2 = (int16_t)canValue;
		amp2-=30000;
		xEventGroupSetBits(xEventGroupDisplay, AMP_BIT1);
		break;
	case AMP2:
		amp2 = (int16_t)canValue;
		xEventGroupSetBits(xEventGroupDisplay, AMP_BIT2);
		break;
	/*case AMP3:
		amp2 = (int16_t)canValue;
		xEventGroupSetBits(xEventGroupDisplay, AMP_BIT3);
		break;
	case AMP4:
		amp2 = (int16_t)canValue;
		xEventGroupSetBits(xEventGroupDisplay, AMP_BIT4);
		break;
	case AMP5:
		amp2 = (int16_t)canValue;
    	amp2-=5000;
		xEventGroupSetBits(xEventGroupDisplay, AMP_BIT5);
		break;
	case AMP6:
		amp2 = (int16_t)canValue;
		xEventGroupSetBits(xEventGroupDisplay, AMP_BIT6);
		break;
	case AMP7:
		amp = canValue;
		xEventGroupSetBits(xEventGroupDisplay, AMP_BIT7);
		break;
	case AMP8:
		amp = canValue;
		xEventGroupSetBits(xEventGroupDisplay, AMP_BIT8);
		break;
	case AMP9:
		amp = canValue;
		xEventGroupSetBits(xEventGroupDisplay, AMP_BIT9);
		break;
	case AMP10:
		amp = canValue;
		amp /= 10;
		xEventGroupSetBits(xEventGroupDisplay, AMP_BIT9);
		break;*/
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		osSemaphoreRelease(xSemCanDetectHandle);
	    /* Retrieve Rx messages from RX FIFO0 */
	    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	    {
	    	Error_Handler();
	    }
	    /*if ((RxHeader.Identifier == 0x1CECFF23)||(RxHeader.Identifier == 0x1CEBFF23))//0x8f40301//0x10f8109a
	    {
	    	//vbtM=0;
	    	//canBatCount=10000;
	    	if(RxData[0]==0x20)
	    	{
	    		lineCount = RxData[3];
	    		CANindex = 1;
	    		if(RxData[5]==0xB5)
	    			CANtype = 1;    //SOC, SOH
	    		else if(RxData[5]==0xB0)
					CANtype = 2;    //Current, RaH
	    		else if(RxData[5]==0xC7)
					CANtype = 3;    //voltage
	    		else if(RxData[5]==0x7E)
					CANtype = 4;    //battery temp
	    		else
	    			CANtype = 0;
	    	}
	    	else  //if(CANtype=1|CANtype=2|CANtype=3CANtype=4)
	    	{
	    		if(RxData[0]==CANindex)
	    		{
	    			for(int j=0; j<7; j++)
	    			//for(int j=0; j<6; j++)
	    			{
	    				CANbuf[CANindex-1][j] = RxData[j+1];
	    			}
	    			if(CANindex<lineCount)
	    			{
	    				CANindex++;
	    			}
	    			else
	    			{
	    				CANdone = 1;
	    			}
	    		}
	    	}
	    }
	    /*if (RxHeader.Identifier == 0x18FBF523)
	    {
	    	if(RxData[0]==0)
	    	{
	    		canIf30=1;
	    		batteryFault16 = ((RxData[1]&0xC0)||(RxData[2])||(RxData[3]&0x03));
	    		thrmlRun=(RxData[2]&0x01);
	    	}
	    }
	    if (RxHeader.Identifier == 0x100)
	    {
	  	  canIfS1=1;
	  	  volt = ((RxData[0]<<8)|(RxData[1]));
	  	  amp=((RxData[2]<<8)|(RxData[3]));
	  	  rAh = ((RxData[4]<<8)|(RxData[5]));
	  	  //canBatCount=4000;
	  	  //vbtM=0;
	    }
	    if (RxHeader.Identifier == 0x101)//0x8f40301//0x10f8109a
	    {
		  canIfS2=1;
		  cAh = (RxData[0]<<8)|(RxData[1]);
		  soc = (RxData[4]<<8)|(RxData[5]);
	  	  //canBatCount=4000;
	  	  //vbtM=0;
	    }
	    if (RxHeader.Identifier == 0x102)//0x8f40301//0x10f8109a
	    {
		  canIfS3=1;
		  flt1 = ((RxData[0]<<8)|(RxData[1]));
		  flt2 = ((RxData[2]<<8)|(RxData[3]));
		  flt3 = ((RxData[4]<<8)|(RxData[5]));
		  //canBatCount=4000;
	  	  //vbtM=0;
	    }
	    if (RxHeader.Identifier == 0x104)//0x8f40301//0x10f8109a
	    {
		  canIfS7=1;
		  ntcCount=RxData[1];
	  	  //canBatCount=4000;
	  	  //vbtM=0;
	    }
	    if (RxHeader.Identifier == 0x105)//0x8f40301//0x10f8109a
	    {
		  canIfS5=1;
		  if (ntcCount>0)
			  ntc1 = ((RxData[0]<<8)|(RxData[1]));
		  else
			  ntc1 = 0;
		  if (ntcCount>1)
			  ntc2 = ((RxData[2]<<8)|(RxData[3]));
		  else
			  ntc2 = 0;
		  if (ntcCount>2)
			  ntc3 = ((RxData[4]<<8)|(RxData[5]));
		  else
			  ntc3 = 0;
		  //canBatCount=4000;
	  	  //vbtM=0;
	    }
	    if (RxHeader.Identifier == 0x106)//0x8f40301//0x10f8109a
	    {
		  canIfS5=1;
		  if (ntcCount>3)
			  ntc4 = ((RxData[0]<<8)|(RxData[1]));
		  else
			  ntc4 = 0;
		  if (ntcCount>4)
			  ntc5 = ((RxData[2]<<8)|(RxData[3]));
		  else
			  ntc5 = 0;
		  if (ntcCount>5)
			  ntc6 = ((RxData[4]<<8)|(RxData[5]));
		  else
			  ntc6 = 0;
		  //canBatCount=4000;
	  	  //vbtM=0;
	    }
	    if (RxHeader.Identifier == 0x14520902)//0x8f40301//0x10f8109a
	    {
	  	  canIf00=1;
	  	  //spdM=0;
	  	  if(!emp1)
	  	  {
	  		  if (spdLH)
	  		  {
	  			  speedVar=((RxData[0]<<8)|(RxData[1]));    //motrola
	  		  }
	  		  else
	  		  {
	  			  speedVar=((RxData[1]<<8)|(RxData[0]));    //intel
	  		  }
	  	  }
	  	  //canControllerCount=2000;
	    }
	    if (RxHeader.Identifier == 0x18530902)//0x8f40301//0x10f8109a
	    {
	  	  canIf2=1;
	  	  //spdM=0;
	  	  throttlePos=RxData[0];
	  	  if(!emp2)
	  	  {
	  		  direction=(RxData[1]&0x08)>>3;
	  		  parkingMod=fun(RxData[1],2);
	  		  gear=(RxData[4]&0x06)>>1;
	  		  brkMod=fun(RxData[2],4);
	  		  //RxData[1]&0x03;
	  		  motorFault=fun(RxData[2],6)||fun(RxData[2],7);
	  		  motorTemp=RxData[3];
	  		  controllerFault=fun(RxData[1],5)||fun(RxData[1],4)||fun(RxData[2],6);
	  		  reGen=fun(RxData[2],2);
	  		  throtelFault=fun(RxData[2],5);
	  	  }
	  	  else
	  	  {
	  		  gear=(RxData[4]&0x03);
	  	  }
	  	  //canControllerCount=2000;
	    }
	    if (RxHeader.Identifier == 0x18660903)
	    {
			if(!mcx1)
			{
			  canIf3=1;
			  //volt=(RxData[3]<<8)|RxData[2];
			  soc=(RxData[5]<<8)|RxData[4];
			  ahVar =(RxData[7]<<8)|RxData[6];
			  //amp=(RxData[4]<<8)|RxData[5];
			  //canBatCount=5000;
			}
	    }
	    if (RxHeader.Identifier == 0x1806E5F4)	//0x0C640903								//0x93  CAN reciver address
	    {
	  	  if(!mcx1)
	  	  {
			  canIf04=1;
			  chargeState=RxData[4];
			  //canBatCount=5000;
	  	  }
	    }
	    if (RxHeader.Identifier == 0x08650903)									//0x98  CAN reciver address
	    {
			  canIf05=1;
			  errorState=(RxData[7]||RxData[5]||RxData[4]||RxData[3]||RxData[2]||RxData[1]||RxData[0]);
			  //canBatCount=5000;
	    }
	    if (RxHeader.Identifier == 0x10630903)									//0x98  CAN reciver address
	    {
	  	  if(!mcx1)
	  	  {
			  canIf06=1;
			  volt=((RxData[7]<<24)|(RxData[6]<<16)|(RxData[5]<<8)|RxData[4]);
			  amp=((RxData[3]<<24)|(RxData[2]<<16)|(RxData[1]<<8)|RxData[0])>>10;
			  //amp2=((RxData[3]<<24)|(RxData[2]<<16)|(RxData[1]<<8)|RxData[0]);
			  //amp=-amp;
			  //amp3=-amp2;
			  //amp4=amp3>>10;
			  //canBatCount=5000;
	  	  }
	    }
	  	if (RxHeader.Identifier == 0x08F40301)
	  	{
	    	  canIf4=1;
	    	  speedVar=((RxData[4]<<8)|(RxData[5]));
	    	  //canControllerCount=2000;
	  	}
	  	if (RxHeader.Identifier == 0x08F60301)
	  	{
	  		  canIf5=1;
	  		  direction=RxData[0]&0x03;
	  		  gear=RxData[0]&0x04;
	  		  throtelFault=RxData[2]&0x40;
	  		  controllerFault=RxData[2]&0x10;
	  		  mtempAlert=((RxData[4]<<8)|(RxData[5]));
	  		  mTemp=RxData[2]&0x80;
	  		  //canControllerCount=2000;
	  	}
	    if (RxHeader.Identifier == 0x18010113)									//0x98  CAN reciver address
	    {
	    	  canIf7=1;
	    	  tripVarC=((RxData[4]<<24)|(RxData[5]<<16)|(RxData[6]<<8)|RxData[7]);
	    	  odoVarC=((RxData[0]<<24)|(RxData[1]<<16)|(RxData[2]<<8)|RxData[3]);
	    	  //canControllerCount=2000;
	    }
	    if (RxHeader.Identifier == 0x18904001)
	    {
	    	  canIf8=1;
	    	  volt=(RxData[0]<<8)|RxData[1];
	    	  soc=(RxData[6]<<8)|RxData[7];
	    	  amp2=(RxData[4]<<8)|RxData[5];
	    	  //canBatCount=3000;
	    	  //vbtM=0;
	    }*/
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
		osSemaphoreRelease(xSemCanDetectHandle);
	    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK)
	    {
	    	Error_Handler();
	    }
	}
}

void StartCanTask(void *argument)
{
	uint8_t seq;
	//FDCAN1_Config();
	for(;;)
	{
		for(seq = 0; seq < 20; seq++)
		{
			TxData[0] = 0x00;
			TxData[1] = 0x00;
			TxData[2] = 0x00;
			TxData[3] = 0x00;
			TxData[4] = 0x00;
			TxData[5] = 0x00;
			TxData[6] = 0x00;
			TxData[7] = 0x00;
			switch( seq )
			{
				case 0:
	  				TxHeader.Identifier = 0x18900140;
	  				TxHeader.IdType 	= FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 1:
	  	    		TxHeader.Identifier = 0x18930140;
	  	    		TxHeader.IdType 	= FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 2:
	  	    		TxHeader.Identifier = 0x18980140;
	  	    		TxHeader.IdType = FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 3:
	  	    		TxHeader.Identifier = 0x18920140;
	  	    		TxHeader.IdType = FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 4:
	  	    		TxHeader.Identifier = 0x18940140;
	  	    		TxHeader.IdType = FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 5:
	  	    		TxHeader.Identifier = 0x18950140;
	  	    		TxHeader.IdType = FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 6:
	  	    		TxHeader.Identifier = 0x18000112;
	  	    		TxHeader.IdType = FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	  	    		TxData[0] = odoSwVal;
					break;
				case 7:
	  	    		TxHeader.Identifier = 0x18000111;
	  	    		TxHeader.IdType = FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	  	    		uint32_t adr=eeCounter*4;
	  	    		eepromRead(adr, tBuf, 4);
	  	    		//at24_HAL_ReadBytes(&hi2c1, 0xA0, adr, (uint8_t *)&tBuf, 4);
	  	    		odoVar=tBuf[0];
	  	   			odoVar<<=8;
	  	   			odoVar+=tBuf[1];
	  	   			odoVar<<=8;
	  	   			odoVar+=tBuf[2];
	  	   			odoVar<<=8;
	  	   			odoVar+=tBuf[3];
	  	   			TxData[0] = tBuf[0];
	  	    		TxData[1] = tBuf[1];
	  	    		TxData[2] = tBuf[2];
	  	    		TxData[3] = tBuf[3];
	  	    		HAL_Delay(1);
	  	    		//odoVar=(unsigned long)((tBuf[0]<<24)|(tBuf[1]<<16)|(tBuf[2]<<8)|(tBuf[3]));
	  	    		adr+=ADDR_TRIP_OFF;
	  	    		eepromRead(adr, tBuf, 4);
	  	    	//	at24_HAL_ReadBytes(&hi2c1, 0xA0, adr, (uint8_t *)&tBuf, 4);
	  	    		//read_eeprom(adr, tBuf, 4);
	  	    		tripVar=tBuf[0];
	  	    		tripVar<<=8;
	  	    		tripVar+=tBuf[1];
	  	    		tripVar<<=8;
	  	    		tripVar+=tBuf[2];
	  	    		tripVar<<=8;
	  	    		tripVar+=tBuf[3];
	  	    		TxData[4] = tBuf[0];
	  	    		TxData[5] = tBuf[1];
	  	    		TxData[6] = tBuf[2];
	  	    		TxData[7] = tBuf[3];
					break;
				case 8:
	  	    		TxHeader.Identifier=0x100;
	  	    		TxHeader.IdType = FDCAN_STANDARD_ID;//FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 9:
	  	    		TxHeader.Identifier=0x101;
	  	    		TxHeader.IdType = FDCAN_STANDARD_ID;//FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 10:
	  	    		TxHeader.Identifier=0x102;
	  	    		TxHeader.IdType = FDCAN_STANDARD_ID;//FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 11:
	  	    		TxHeader.Identifier=0x105;
	  	    		TxHeader.IdType = FDCAN_STANDARD_ID;//FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 12:
	  	    		TxHeader.Identifier=0x106;
	  	    		TxHeader.IdType = FDCAN_STANDARD_ID;//FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 13:
	  	    		TxHeader.Identifier=0x104;
	  	    		TxHeader.IdType = FDCAN_STANDARD_ID;//FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 14:
	  	    		TxHeader.Identifier=0x103;
	  	    		TxHeader.IdType = FDCAN_STANDARD_ID;//FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	  				TxData[0] = 0x01;
	  				TxData[1] = 0x03;
	  				TxData[2] = 0x00;
	  				TxData[3] = 0x1D;
	  				TxData[4] = 0x00;
	  				TxData[5] = 0x60;
	  				TxData[6] = 0xD5;
	  				TxData[7] = 0xE4;
					break;
				case 15:
	  	    		TxHeader.Identifier=0x1FA;
	  	    		TxHeader.IdType = FDCAN_EXTENDED_ID;//FDCAN_EXTENDED_ID;
	  	    		TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	  				TxData[0] = 0xAA;
					break;
				case 16:
	  	    		TxHeader.Identifier=0x3ABDCEF;
	  	    		TxHeader.IdType = FDCAN_EXTENDED_ID;//FDCAN_EXTENDED_ID;
	  	    		TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	  				TxData[0] = 0x0E;
					break;
				case 17:
	  	    		TxHeader.Identifier=0x1F3D2518;
	  	    		TxHeader.IdType = FDCAN_EXTENDED_ID;//FDCAN_EXTENDED_ID;
	  	    		TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	  				TxData[0] = 0x0E;
					break;
				case 18:
	  	    		TxHeader.Identifier=0xE64090D;
	  	    		TxHeader.IdType = FDCAN_EXTENDED_ID;
	  	    		TxHeader.DataLength = FDCAN_DLC_BYTES_0;
					break;
				case 19:
	  	    		TxHeader.Identifier=0x330;
	  	    		TxHeader.IdType = FDCAN_STANDARD_ID;//FDCAN_EXTENDED_ID;
	  				TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	  				TxData[0] = 0x01;
	  				TxData[1] = 0x04;
	  				TxData[2] = 0x01;
	  				TxData[3] = 0x30;
	  				TxData[4] = 0x00;
	  				TxData[5] = 0x00;
	  				TxData[6] = 0x01;
	  				TxData[7] = 0x30;
	  	    		TxHeader.TxFrameType = FDCAN_DATA_FRAME;
					break;
			}
			int ret=HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
			if(ret>=1)
			{
				FDCAN_SetBaud(tBaud);
				//MX_FDCAN1_Init();
				FDCAN1_Config();
			}
			osDelay(30);
		}
	}
}
