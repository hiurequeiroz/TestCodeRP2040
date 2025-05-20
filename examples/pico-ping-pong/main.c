/*!
 * \file      main.c
 *
 * \brief     Ping-Pong implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <string.h>
#include <stdio.h>
#include "board.h"
#include "gpio.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"

#include "pico/stdlib.h"
#include "tusb.h"
#include "sx126x.h"
#include "sx126x-board.h"
const uint LED_PIN = PICO_DEFAULT_LED_PIN;


#define RF_FREQUENCY                                915000000 // Hz


#define TX_OUTPUT_POWER                             5        // dBm


#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false



typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            2000
#define BUFFER_SIZE                                 64 // Define the payload size here


uint8_t TxBuffer[BUFFER_SIZE];
uint8_t RxBuffer[BUFFER_SIZE];

// start status,send a packet
States_t State = RX_ERROR;
bool tx_done = false;
bool rx_timeout = false;
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1;
extern Gpio_t Led2;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );


uint32_t wk_time;
uint16_t irq_status;

RadioError_t device_error;
RadioStatus_t device_status;

/**
 * Main application entry point.
 */
int main( void )
{
    uint8_t i;
    
    stdio_init_all();

    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    // gpio_put(LED_PIN, 1);
    // sleep_ms(250);
    gpio_put(LED_PIN, 1);
    // sleep_ms(250);

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( 865000000 );



    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.SetMaxPayloadLength( MODEM_LORA, BUFFER_SIZE );


    Radio.Rx( RX_TIMEOUT_VALUE );

 
    while( 1 )
    {
        
        TxBuffer[0] = 'P';
        SX126xAntSwOff();
        DelayMs( 1 );
        tx_done = false;
        Radio.Send( TxBuffer, 1 );
        printf("sending P\r\n");
        while( !tx_done ) {
            if( Radio.IrqProcess != NULL )
            {
                Radio.IrqProcess( );
            }
        } 
        printf("sent\r\n");


        SX126xAntSwOn();
        printf("RX SETUP\r\n");
        rx_timeout = false;
        Radio.Rx( RX_TIMEOUT_VALUE );
        while( !rx_timeout ) {
            if( Radio.IrqProcess != NULL )
            {
                Radio.IrqProcess( );
            }
        }
    }
}

void OnTxDone( void )
{
    tx_done = true;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    gpio_put(LED_PIN, 1);
    sleep_ms(25);
    gpio_put(LED_PIN, 0);

    printf("Rssi is -%ddBm\r\n",-rssi/2);
	printf("Snr is %ddB\r\n",snr/4);
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
    printf("OnTxTimeout\r\n");
    SX126xClearDeviceErrors();

}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    printf("OnRxTimeout\r\n");

    // the spi bus is ok
    device_status = SX126xGetStatus();
    printf("device status 0x%x\r\n",device_status.Fields);
    device_error = SX126xGetDeviceErrors();
    SX126xClearDeviceErrors();
    printf("device error 0x%x\r\n",device_error.Fields);

    rx_timeout = true;
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    printf("OnRxError\r\n");
}
