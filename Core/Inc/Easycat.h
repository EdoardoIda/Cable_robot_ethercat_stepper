/*
 * Easycat.h
 *
 *  Created on: Dec 16, 2020
 *      Author: EdoOffice
 */

#ifndef INC_EASYCAT_H_
#define INC_EASYCAT_H_

#include "main.h"
#include "Cable_robot_winch.h"
#include "Error_manager.h"
#define CUSTOM

#if (!defined BYTE_NUM && !defined CUSTOM)
  #define BYTE_NUM 32
#endif

//---- Calcolo della dimensione e del numero di trasferimenti necessari in output ---

#ifdef  BYTE_NUM
  #define TOT_BYTE_NUM_OUT  BYTE_NUM
#else
  #define TOT_BYTE_NUM_OUT  CUST_BYTE_NUM_OUT
#endif

#if TOT_BYTE_NUM_OUT > 64

  #define SEC_BYTE_NUM_OUT  (TOT_BYTE_NUM_OUT - 64)

  #if ((SEC_BYTE_NUM_OUT & 0x03) != 0x00)
    #define SEC_BYTE_NUM_ROUND_OUT  ((SEC_BYTE_NUM_OUT | 0x03) + 1)
  #else
    #define SEC_BYTE_NUM_ROUND_OUT  SEC_BYTE_NUM_OUT
  #endif

  #define FST_BYTE_NUM_OUT  64
  #define FST_BYTE_NUM_ROUND_OUT  64

#else

  #define FST_BYTE_NUM_OUT  TOT_BYTE_NUM_OUT

  #if ((FST_BYTE_NUM_OUT & 0x03) != 0x00)
    #define FST_BYTE_NUM_ROUND_OUT ((FST_BYTE_NUM_OUT | 0x03) + 1)
  #else
    #define FST_BYTE_NUM_ROUND_OUT  FST_BYTE_NUM_OUT
  #endif

  #define SEC_BYTE_NUM_OUT  0
  #define SEC_BYTE_NUM_ROUND_OUT  0

#endif

//---- Calcolo della dimensione e del numero di trasferimenti necessari in input ---

#ifdef  BYTE_NUM
  #define TOT_BYTE_NUM_IN  BYTE_NUM
#else
  #define TOT_BYTE_NUM_IN  CUST_BYTE_NUM_IN
#endif

#if TOT_BYTE_NUM_IN > 64

  #define SEC_BYTE_NUM_IN  (TOT_BYTE_NUM_IN - 64)

  #if ((SEC_BYTE_NUM_IN & 0x03) != 0x00)

    #define SEC_BYTE_NUM_ROUND_IN  ((SEC_BYTE_NUM_IN | 0x03) + 1)
  #else
    #define SEC_BYTE_NUM_ROUND_IN  SEC_BYTE_NUM_IN
  #endif

  #define FST_BYTE_NUM_IN  64
  #define FST_BYTE_NUM_ROUND_IN  64

#else

  #define FST_BYTE_NUM_IN  TOT_BYTE_NUM_IN

  #if ((FST_BYTE_NUM_IN & 0x03) != 0x00)

    #define FST_BYTE_NUM_ROUND_IN ((FST_BYTE_NUM_IN | 0x03) + 1)
  #else
    #define FST_BYTE_NUM_ROUND_IN  FST_BYTE_NUM_IN
  #endif

  #define SEC_BYTE_NUM_IN  0
  #define SEC_BYTE_NUM_ROUND_IN  0

#endif

//----------------------------- Sanity check ---------------------------------------

#ifdef BYTE_NUM

  #ifdef CUST_BYTE_NUM_OUT
    #error "BYTE_NUM and CUST_BYTE_NUM_OUT cannot be defined at the same time !!!!"
    #error "define them correctly in file EasyCAT.h"
  #endif

  #ifdef CUST_BYTE_NUM_IN
    #error "BYTE_NUM and CUST_BYTE_NUM_IN cannot be defined at the same time !!!!"
    #error "define them correctly in file EasyCAT.h"
  #endif

#endif

#ifdef BYTE_NUM

  #if ((BYTE_NUM !=16) && (BYTE_NUM !=32) && (BYTE_NUM !=64)  && (BYTE_NUM !=128))
    #error "BYTE_NUM must be 16, 32, 64 or 128 !!! define it correctly in file EasyCAT.h"
  #endif

#else

  #if (CUST_BYTE_NUM_OUT > 128)
    #error "CUST_BYTE_NUM_OUT must be max 128 !!! define it correctly in file EasyCAT.h"
  #endif

  #if (CUST_BYTE_NUM_IN > 128)
    #error "CUST_BYTE_NUM_IN must be max 128 !!! define it correctly in file EasyCAT.h"
  #endif

#endif

//*************************************************************************************************

//---- Registri LAN9252 ---------------------------------------------------------------------------

                                            //---- Accesso ai registri EtherCAT -------------------

#define ECAT_CSR_DATA           0x0300      // EtherCAT CSR Interface Data Register
#define ECAT_CSR_CMD            0x0304      // EtherCAT CSR Interface Command Register

                                            //---- Accesso alla EtherCAT process RAM --------------

#define ECAT_PRAM_RD_ADDR_LEN   0x0308      // EtherCAT Process RAM Read Address and Length Register
#define ECAT_PRAM_RD_CMD        0x030C      // EtherCAT Process RAM Read Command Register
#define ECAT_PRAM_WR_ADDR_LEN   0x0310      // EtherCAT Process RAM Write Address and Length Register
#define ECAT_PRAM_WR_CMD        0x0314      // EtherCAT Process RAM Write Command Register

#define ECAT_PRAM_RD_DATA       0x0000      // EtherCAT Process RAM Read Data FIFO
#define ECAT_PRAM_WR_DATA       0x0020      // EtherCAT Process RAM Write Data FIFO

                                            //---- Registri EtherCAT -----------------------------

#define AL_CONTROL              0x0120      // AL control
#define AL_STATUS               0x0130      // AL status
#define AL_STATUS_CODE          0x0134      // AL status code
#define AL_EVENT                0x0220      // AL event request
#define AL_EVENT_MASK           0x0204      // AL event interrupt mask

#define WDOG_STATUS             0x0440      // watch dog status

#define SM0_BASE                0x0800      // SM0 base address (output)
#define SM1_BASE                0x0808      // SM1 base address (input)

                                            //---- Registri LAN9252 ------------------------------

#define HW_CFG                  0x0074      // hardware configuration register
#define BYTE_TEST               0x0064      // byte order test register
#define RESET_CTL               0x01F8      // reset register
#define ID_REV                  0x0050      // chip ID and revision
#define IRQ_CFG                 0x0054      // interrupt configuration
#define INT_EN                  0x005C      // interrupt enable

//---- Flag LAN9252 ------------------------------------------------------------------------------

#define ECAT_CSR_BUSY     0x80
#define PRAM_ABORT        0x40000000
#define PRAM_BUSY         0x80
#define PRAM_AVAIL        0x01
#define READY             0x08
#define DIGITAL_RST       0x00000001

//---- Flag EtherCAT -----------------------------------------------------------------------------

#define ALEVENT_CONTROL         0x0001
#define ALEVENT_SM              0x0010

//----- State Machine ------------------------------------------------------------

#define ESM_INIT                  0x01          // state machine control
#define ESM_PREOP                 0x02          // (state request)
#define ESM_BOOT                  0x03          //
#define ESM_SAFEOP                0x04          // safe-operational
#define ESM_OP                    0x08          // operational

//--- Comandi ESC --------------------------------------------------------------------------------

#define ESC_WRITE 		     0x80
#define ESC_READ 		     0xC0

//---- SPI ----------------------------------------------------------------------------------------

#define COMM_SPI_READ        0x03
#define COMM_SPI_WRITE       0x02
#define COMM_SPI_FAST_READ   0x0B

#define DUMMY_BYTE           0xFF

//*************************************************************************************************

//---- Typedef ------------------------------------------------------------------------------------

typedef union
{
    unsigned short  Word;
    unsigned char   Byte[2];
} UWORD;

typedef union
{
    unsigned long   Long;
    unsigned short  Word[2];
    unsigned char   Byte[4];
} ULONG;

#ifdef BYTE_NUM

  typedef struct
  {
    uint8_t  Byte [BYTE_NUM];
  } PROCBUFFER_OUT;

  typedef struct
  {
    uint8_t  Byte [BYTE_NUM];
  } PROCBUFFER_IN;

#endif


//---- Dichiarazione variabili locali e typedef struttura -----------------------------------------

PROCBUFFER_OUT Buffer_Out_Loc;  //buffer di output EtherCAT dal master
PROCBUFFER_IN Buffer_In_Loc;    //buffer di input EtherCAT al master
uint8_t Tx[67];                 //buffer di trasmissione SPI
uint8_t Rx[64];                 //buffer di ricezione SPI

typedef struct  {
	SPI_HandleTypeDef *spi_line;
	GPIO_TypeDef* SCS_port;
	uint16_t SCS_pin;
	PROCBUFFER_OUT *BufferOut;
	PROCBUFFER_IN *BufferIn;
} Easycat;                        //definisce il tipo di struttura contenente le impostazioni di
                                        //configurazione del trasferimento SPI e i puntatori ai buffer
                                        //locali di trasmissione e ricezione EtherCAT

//-------------------------------------------------------------------------------------------------

void easyCat_Init(Easycat *hEC, SPI_HandleTypeDef *hspi, GPIO_TypeDef* SPI_CHIP_SELECT_PORT, uint16_t SPI_CHIP_SELECT, error_t *error);
                                                                       //imposta la comunicazione, richiedendo in ingresso la struttura handler
                                                                       //e le informazioni relative alla linea SPI, al chip select e alla modalità
                                                                       //di comunicazione

unsigned char easyCat_Read(Easycat *hEC);          //accede alla memoria di master input e output presente sul chip LAN,
                                                             //in modalità asincrona è da chiamare ciclicamente nel main
unsigned char easyCat_Write(Easycat *hEC);

uint8_t easyCat_Setup(Easycat *hEC, error_t *error); //inizializza la comunicazione, è chiamata all'interno del setup

void easyCat_SPIParametersCheck(Easycat *hEC, error_t *error);  //verifica i parametri di comunicazione SPI
                                                      //(prescaler, dimensione pacchetti di dati, ordine byte)

void easyCat_SPIWriteRegisterDirect(Easycat *hEC, unsigned short Address, unsigned long DataOut);  //Scrittura di registri direttamente accessibili
unsigned long easyCat_SPIReadRegisterDirect(Easycat *hEC, unsigned short Address, unsigned char Len);  //Lettura di registri direttamente accessibili

void easyCat_SPIWriteRegisterIndirect(Easycat *hEC, unsigned long  DataOut, unsigned short Address, unsigned char Len);  //Scrittura di registri non direttamente accessibili
unsigned long easyCat_SPIReadRegisterIndirect(Easycat *hEC, unsigned short Address, unsigned char Len);  //Lettura di registri non direttamente accessibili

void easyCat_SPIReadProcRamFifo(Easycat *hEC);    //Lettura dalla RAM di processo di master output
void easyCat_SPIWriteProcRamFifo(Easycat *hEC);   //Scrittura nella RAM di processo di master input

inline void easyCat_SCS_Low(Easycat *hEC) { HAL_GPIO_WritePin(hEC->SCS_port, hEC->SCS_pin, GPIO_PIN_RESET); }  //Attiva la comunicazione SPI
inline void easyCat_SCS_High(Easycat *hEC) { HAL_GPIO_WritePin(hEC->SCS_port, hEC->SCS_pin, GPIO_PIN_SET); }  //Termina la comunicazione SPI

inline void easyCat_SPI_TransferTx(Easycat *hEC, uint16_t size) { HAL_SPI_Transmit(hEC->spi_line, &Tx[0], size, 1); }; //Trasmissione SPI
inline void easyCat_SPI_TransferRx(Easycat *hEC, uint16_t size) { HAL_SPI_Receive(hEC->spi_line, &Rx[0], size, 1); };  //Ricezione SPI

#endif /* INC_EASYCAT_H_ */
