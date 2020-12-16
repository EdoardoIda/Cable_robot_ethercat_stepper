/*
 * Easycat.c
 *
 *  Created on: Dec 16, 2020
 *      Author: EdoOffice
 */

#include "Easycat.h"

void EasyCat_Setup(HandleEasyCat *hEC, SPI_HandleTypeDef *hspi, GPIO_TypeDef* SPI_CHIP_SELECT_PORT, uint16_t SPI_CHIP_SELECT) {
	hEC->spi_line = hspi;
	hEC->SCS_port = SPI_CHIP_SELECT_PORT;
	hEC->SCS_pin = SPI_CHIP_SELECT;
	hEC->LED_port = GPIOB;
	hEC->LED_pin = GPIO_PIN_7;
	hEC->BufferIn = &Buffer_In_Loc;
	hEC->BufferOut = &Buffer_Out_Loc;
	uint8_t flag = EasyCat_Init(hEC);
	while (flag != 1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_Delay(1000);
	}
}

//---------------------------------------------------

uint8_t EasyCat_Init(HandleEasyCat *hEC)
{
  #define Tout 1000

  ULONG TempLong;
  unsigned short i;

  EasyCat_SCS_High(hEC);

  HAL_Delay(100);

  EasyCat_SPIParametersCheck(hEC);

  EasyCat_SPIWriteRegisterDirect (hEC, RESET_CTL, DIGITAL_RST);

  i = 0;
  do
  {
    i++;
    TempLong.Long = EasyCat_SPIReadRegisterDirect (hEC, RESET_CTL, 4);
  }while (((TempLong.Byte[0] & 0x01) != 0x00) && (i != Tout));

  if (i == Tout)
  {
	HAL_SPI_DeInit(hEC->spi_line);
    return 0;
  }

  i = 0;
  do
  {
    i++;
    TempLong.Long = EasyCat_SPIReadRegisterDirect (hEC, BYTE_TEST, 4);
  }while ((TempLong.Long != 0x87654321) && (i != Tout));

  if (i == Tout)
  {
	HAL_SPI_DeInit(hEC->spi_line);
    return 0;
  }

  i = 0;
  do
  {
    i++;
    TempLong.Long = EasyCat_SPIReadRegisterDirect (hEC, HW_CFG, 4);
  }while (((TempLong.Byte[3] & READY) == 0) && (i != Tout));

  if (i == Tout)
  {
	HAL_SPI_DeInit(hEC->spi_line);
    return 0;
  }

  return 1;
}

//------------------------------------------------------------------------

unsigned char EasyCat_MainTask(HandleEasyCat *hEC)
{
  uint8_t WatchDog = 0;
  uint8_t Operational = 0;
  unsigned char i;
  ULONG TempLong;
  unsigned char Status;

  TempLong.Long = EasyCat_SPIReadRegisterIndirect (hEC, WDOG_STATUS, 1);
  if ((TempLong.Byte[0] & 0x01) == 0x01)
    WatchDog = 0;
  else
    WatchDog = 1;

  TempLong.Long = EasyCat_SPIReadRegisterIndirect (hEC, AL_STATUS, 1);
  Status = TempLong.Byte[0] & 0x0F;

  if (Status == ESM_OP)
    Operational = 1;
  else
    Operational = 0;

  if (WatchDog | !Operational)
  {
    for (i=0; i < TOT_BYTE_NUM_OUT ; i++)
    {
      hEC->BufferOut->Byte[i] = 0;
    }
   }
  else
  {
    EasyCat_SPIReadProcRamFifo(hEC);
  }

  EasyCat_SPIWriteProcRamFifo(hEC);

  if (WatchDog)
  {
    Status |= 0x80;
  }
  return Status;
}


//---------------------------------------------------------

unsigned long EasyCat_SPIReadRegisterDirect (HandleEasyCat *hEC, unsigned short Address, unsigned char Len)

{
  ULONG Result;
  UWORD Addr;
  Addr.Word = Address;
  unsigned char i;

  EasyCat_SCS_Low(hEC);

    Tx[0] = COMM_SPI_READ;
    Tx[1] = Addr.Byte[1];
    Tx[2] = Addr.Byte[0];

    EasyCat_SPI_TransferTx(hEC, 3);



  EasyCat_SPI_TransferRx(hEC, Len);
  EasyCat_SCS_High(hEC);
  for (i = 0;i < Len;i++) {
	  Result.Byte[i] = Rx[i];
  }



  return Result.Long;
}


//------------------------------------------------------------------------------------------

void EasyCat_SPIWriteRegisterDirect (HandleEasyCat *hEC, unsigned short Address, unsigned long DataOut)

{
  ULONG Data;
  UWORD Addr;
  Addr.Word = Address;
  Data.Long = DataOut;


  EasyCat_SCS_Low(hEC);

  Tx[0] = COMM_SPI_WRITE;
  Tx[1] = Addr.Byte[1];
  Tx[2] = Addr.Byte[0];
  Tx[3] = Data.Byte[0];
  Tx[4] = Data.Byte[1];
  Tx[5] = Data.Byte[2];
  Tx[6] = Data.Byte[3];

  EasyCat_SPI_TransferTx(hEC, 7);

  EasyCat_SCS_High(hEC);
}

//---------------------------------------------------------------------------------------------------

unsigned long EasyCat_SPIReadRegisterIndirect (HandleEasyCat *hEC, unsigned short Address, unsigned char Len)

{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;

  TempLong.Byte[0] = Addr.Byte[0];
  TempLong.Byte[1] = Addr.Byte[1];
  TempLong.Byte[2] = Len;
  TempLong.Byte[3] = ESC_READ;

  EasyCat_SPIWriteRegisterDirect (hEC, ECAT_CSR_CMD, TempLong.Long);

  do
  {
    TempLong.Long = EasyCat_SPIReadRegisterDirect(hEC, ECAT_CSR_CMD, 4);
  }
  while(TempLong.Byte[3] & ECAT_CSR_BUSY);

  TempLong.Long = EasyCat_SPIReadRegisterDirect(hEC, ECAT_CSR_DATA, Len);
  return TempLong.Long;
}


//----------------------------------------------------------------

void  EasyCat_SPIWriteRegisterIndirect (HandleEasyCat *hEC, unsigned long DataOut, unsigned short Address, unsigned char Len)

{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;


  EasyCat_SPIWriteRegisterDirect (hEC, ECAT_CSR_DATA, DataOut);



  TempLong.Byte[0] = Addr.Byte[0];
  TempLong.Byte[1] = Addr.Byte[1];
  TempLong.Byte[2] = Len;
  TempLong.Byte[3] = ESC_WRITE;

  EasyCat_SPIWriteRegisterDirect (hEC, ECAT_CSR_CMD, TempLong.Long);

  do
  {
    TempLong.Long = EasyCat_SPIReadRegisterDirect (hEC, ECAT_CSR_CMD, 4);
  }
  while (TempLong.Byte[3] & ECAT_CSR_BUSY);
}

//-----------------------------------------------------------------------

void EasyCat_SPIReadProcRamFifo(HandleEasyCat *hEC)



{
  ULONG TempLong;
  unsigned char i;

  #if TOT_BYTE_NUM_OUT > 0

    EasyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_RD_CMD, PRAM_ABORT);

	EasyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_RD_ADDR_LEN, (0x00001000 | (((uint32_t)TOT_BYTE_NUM_OUT) << 16)));

	EasyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_RD_CMD, 0x80000000);

    do
    {
      TempLong.Long = EasyCat_SPIReadRegisterDirect (hEC, ECAT_PRAM_RD_CMD,2);
    }
    while (TempLong.Byte[1] != (FST_BYTE_NUM_ROUND_OUT/4));

	EasyCat_SCS_Low(hEC);

	  Tx[0] = COMM_SPI_READ;
	  Tx[1] = 0x00;
	  Tx[2] = 0x00;
	  EasyCat_SPI_TransferTx(hEC, 3);

	  EasyCat_SPI_TransferRx(hEC, FST_BYTE_NUM_ROUND_OUT);

    for (i=0; i< FST_BYTE_NUM_ROUND_OUT; i++)
    {
		hEC->BufferOut->Byte[i] = Rx[i];
		// Buffer_Out_Loc.Byte[i] = .....
    }

	EasyCat_SCS_High(hEC);
  #endif

}


//------------------------------------------------------------------

void EasyCat_SPIWriteProcRamFifo(HandleEasyCat *hEC)
{
  ULONG TempLong;
  unsigned char i;

  #if TOT_BYTE_NUM_IN > 0

    EasyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_WR_CMD, PRAM_ABORT);

	EasyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_WR_ADDR_LEN, (0x00001200 | (((uint32_t)TOT_BYTE_NUM_IN) << 16)));

	EasyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_WR_CMD, 0x80000000);

    do
    {
      TempLong.Long = EasyCat_SPIReadRegisterDirect (hEC, ECAT_PRAM_WR_CMD,2);
    }
    while (TempLong.Byte[1] <   (FST_BYTE_NUM_ROUND_IN/4));

	EasyCat_SCS_Low(hEC);

	Tx[0] = COMM_SPI_WRITE;
	Tx[1] = 0x00;
	Tx[2] = 0x20;

    for (i=0; i<FST_BYTE_NUM_ROUND_IN; i++)
    {
		Tx[3+i]= hEC->BufferIn->Byte[i];
    }

	EasyCat_SPI_TransferTx(hEC, 3 + FST_BYTE_NUM_ROUND_IN);

	EasyCat_SCS_High(hEC);
  #endif

}

//--------------------------------------------------------------------------------

void EasyCat_SPIParametersCheck(HandleEasyCat *hEC) {


	while (((hEC->spi_line->Instance->CFG1) & 0x70000000U) != 0x30000000U) {  //check prescaler
		HAL_GPIO_WritePin(hEC->LED_port, hEC->LED_pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(hEC->LED_port, hEC->LED_pin, GPIO_PIN_RESET);
		HAL_Delay(500);
	}

	while (((hEC->spi_line->Instance->CFG1) & 0x0000001FU) != 0x00000007U) {  //check dimensione dati
		HAL_GPIO_WritePin(hEC->LED_port, hEC->LED_pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(hEC->LED_port, hEC->LED_pin, GPIO_PIN_RESET);
		HAL_Delay(500);
	}

	while (((hEC->spi_line->Instance->CFG2) & 0x00800000U) != 0x00000000U) {  //check ordine byte
		HAL_GPIO_WritePin(hEC->LED_port, hEC->LED_pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(hEC->LED_port, hEC->LED_pin, GPIO_PIN_RESET);
		HAL_Delay(500);
	}
}
