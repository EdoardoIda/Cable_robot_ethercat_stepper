/*
 * Easycat.c
 *
 *  Created on: Dec 16, 2020
 *      Author: EdoOffice
 */

#include "Easycat.h"

void easyCat_Init(Easycat *hEC, SPI_HandleTypeDef *hspi, GPIO_TypeDef* SPI_CHIP_SELECT_PORT, uint16_t SPI_CHIP_SELECT) {
	hEC->spi_line = hspi;
	hEC->SCS_port = SPI_CHIP_SELECT_PORT;
	hEC->SCS_pin = SPI_CHIP_SELECT;
	hEC->BufferIn = &Buffer_In_Loc;
	hEC->BufferOut = &Buffer_Out_Loc;
	uint8_t flag = easyCat_Setup(hEC);
	while (flag != 1) {
		(0);
	}
}

//---------------------------------------------------

uint8_t easyCat_Setup(Easycat *hEC)
{
  #define Tout 1000

  ULONG TempLong;
  unsigned short i;

  easyCat_SCS_High(hEC);

  HAL_Delay(100);

  easyCat_SPIParametersCheck(hEC);

  easyCat_SPIWriteRegisterDirect (hEC, RESET_CTL, DIGITAL_RST);

  i = 0;
  do
  {
    i++;
    TempLong.Long = easyCat_SPIReadRegisterDirect (hEC, RESET_CTL, 4);
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
    TempLong.Long = easyCat_SPIReadRegisterDirect (hEC, BYTE_TEST, 4);
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
    TempLong.Long = easyCat_SPIReadRegisterDirect (hEC, HW_CFG, 4);
  }while (((TempLong.Byte[3] & READY) == 0) && (i != Tout));

  if (i == Tout)
  {
	HAL_SPI_DeInit(hEC->spi_line);
    return 0;
  }

  return 1;
}

//------------------------------------------------------------------------

unsigned char easyCat_MainTask(Easycat *hEC)
{
  uint8_t WatchDog = 0;
  uint8_t Operational = 0;
  unsigned char i;
  ULONG TempLong;
  unsigned char Status;

  TempLong.Long = easyCat_SPIReadRegisterIndirect (hEC, WDOG_STATUS, 1);
  if ((TempLong.Byte[0] & 0x01) == 0x01)
    WatchDog = 0;
  else
    WatchDog = 1;

  TempLong.Long = easyCat_SPIReadRegisterIndirect (hEC, AL_STATUS, 1);
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
    easyCat_SPIReadProcRamFifo(hEC);
  }

  easyCat_SPIWriteProcRamFifo(hEC);

  if (WatchDog)
  {
    Status |= 0x80;
  }
  return Status;
}


//---------------------------------------------------------

unsigned long easyCat_SPIReadRegisterDirect (Easycat *hEC, unsigned short Address, unsigned char Len)

{
  ULONG Result;
  UWORD Addr;
  Addr.Word = Address;
  unsigned char i;

  easyCat_SCS_Low(hEC);

    Tx[0] = COMM_SPI_READ;
    Tx[1] = Addr.Byte[1];
    Tx[2] = Addr.Byte[0];

    easyCat_SPI_TransferTx(hEC, 3);



  easyCat_SPI_TransferRx(hEC, Len);
  easyCat_SCS_High(hEC);
  for (i = 0;i < Len;i++) {
	  Result.Byte[i] = Rx[i];
  }



  return Result.Long;
}


//------------------------------------------------------------------------------------------

void easyCat_SPIWriteRegisterDirect (Easycat *hEC, unsigned short Address, unsigned long DataOut)

{
  ULONG Data;
  UWORD Addr;
  Addr.Word = Address;
  Data.Long = DataOut;


  easyCat_SCS_Low(hEC);

  Tx[0] = COMM_SPI_WRITE;
  Tx[1] = Addr.Byte[1];
  Tx[2] = Addr.Byte[0];
  Tx[3] = Data.Byte[0];
  Tx[4] = Data.Byte[1];
  Tx[5] = Data.Byte[2];
  Tx[6] = Data.Byte[3];

  easyCat_SPI_TransferTx(hEC, 7);

  easyCat_SCS_High(hEC);
}

//---------------------------------------------------------------------------------------------------

unsigned long easyCat_SPIReadRegisterIndirect (Easycat *hEC, unsigned short Address, unsigned char Len)

{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;

  TempLong.Byte[0] = Addr.Byte[0];
  TempLong.Byte[1] = Addr.Byte[1];
  TempLong.Byte[2] = Len;
  TempLong.Byte[3] = ESC_READ;

  easyCat_SPIWriteRegisterDirect (hEC, ECAT_CSR_CMD, TempLong.Long);

  do
  {
    TempLong.Long = easyCat_SPIReadRegisterDirect(hEC, ECAT_CSR_CMD, 4);
  }
  while(TempLong.Byte[3] & ECAT_CSR_BUSY);

  TempLong.Long = easyCat_SPIReadRegisterDirect(hEC, ECAT_CSR_DATA, Len);
  return TempLong.Long;
}


//----------------------------------------------------------------

void  easyCat_SPIWriteRegisterIndirect (Easycat *hEC, unsigned long DataOut, unsigned short Address, unsigned char Len)

{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;


  easyCat_SPIWriteRegisterDirect (hEC, ECAT_CSR_DATA, DataOut);



  TempLong.Byte[0] = Addr.Byte[0];
  TempLong.Byte[1] = Addr.Byte[1];
  TempLong.Byte[2] = Len;
  TempLong.Byte[3] = ESC_WRITE;

  easyCat_SPIWriteRegisterDirect (hEC, ECAT_CSR_CMD, TempLong.Long);

  do
  {
    TempLong.Long = easyCat_SPIReadRegisterDirect (hEC, ECAT_CSR_CMD, 4);
  }
  while (TempLong.Byte[3] & ECAT_CSR_BUSY);
}

//-----------------------------------------------------------------------

void easyCat_SPIReadProcRamFifo(Easycat *hEC)



{
  ULONG TempLong;
  unsigned char i;

  #if TOT_BYTE_NUM_OUT > 0

    easyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_RD_CMD, PRAM_ABORT);

	easyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_RD_ADDR_LEN, (0x00001000 | (((uint32_t)TOT_BYTE_NUM_OUT) << 16)));

	easyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_RD_CMD, 0x80000000);

    do
    {
      TempLong.Long = easyCat_SPIReadRegisterDirect (hEC, ECAT_PRAM_RD_CMD,2);
    }
    while (TempLong.Byte[1] != (FST_BYTE_NUM_ROUND_OUT/4));

	easyCat_SCS_Low(hEC);

	  Tx[0] = COMM_SPI_READ;
	  Tx[1] = 0x00;
	  Tx[2] = 0x00;
	  easyCat_SPI_TransferTx(hEC, 3);

	  easyCat_SPI_TransferRx(hEC, FST_BYTE_NUM_ROUND_OUT);

    for (i=0; i< FST_BYTE_NUM_ROUND_OUT; i++)
    {
		hEC->BufferOut->Byte[i] = Rx[i];
		// Buffer_Out_Loc.Byte[i] = .....
    }

	easyCat_SCS_High(hEC);
  #endif

}


//------------------------------------------------------------------

void easyCat_SPIWriteProcRamFifo(Easycat *hEC)
{
  ULONG TempLong;
  unsigned char i;

  #if TOT_BYTE_NUM_IN > 0

    easyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_WR_CMD, PRAM_ABORT);

	easyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_WR_ADDR_LEN, (0x00001200 | (((uint32_t)TOT_BYTE_NUM_IN) << 16)));

	easyCat_SPIWriteRegisterDirect (hEC, ECAT_PRAM_WR_CMD, 0x80000000);

    do
    {
      TempLong.Long = easyCat_SPIReadRegisterDirect (hEC, ECAT_PRAM_WR_CMD,2);
    }
    while (TempLong.Byte[1] <   (FST_BYTE_NUM_ROUND_IN/4));

	easyCat_SCS_Low(hEC);

	Tx[0] = COMM_SPI_WRITE;
	Tx[1] = 0x00;
	Tx[2] = 0x20;

    for (i=0; i<FST_BYTE_NUM_ROUND_IN; i++)
    {
		Tx[3+i]= hEC->BufferIn->Byte[i];
    }

	easyCat_SPI_TransferTx(hEC, 3 + FST_BYTE_NUM_ROUND_IN);

	easyCat_SCS_High(hEC);
  #endif

}

//--------------------------------------------------------------------------------

void easyCat_SPIParametersCheck(Easycat *hEC) {


	while (((hEC->spi_line->Instance->CFG1) & 0x70000000U) != 0x30000000U) {  //check prescaler
		(0);
	}

	while (((hEC->spi_line->Instance->CFG1) & 0x0000001FU) != 0x00000007U) {  //check dimensione dati
		(0);
	}

	while (((hEC->spi_line->Instance->CFG2) & 0x00800000U) != 0x00000000U) {  //check ordine byte
		(0);
	}
}
