/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2018 Bolder Flight Systems
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
* and associated documentation files (the "Software"), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MESSAGES_H
#define MESSAGES_H

#include "checksum.h"
#include "array_macros.h"
#include "global_defs.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"

/*
* Ensure C friendly linkages in a mixed C/C++ build
*/
#ifdef __cplusplus
#define _Bool bool
extern "C" {
#endif

/*
* List of possible message ids.
*/
enum MessageIds {
  MSG_FMU_CFG,
  MSG_INT_MPU9250_CFG,
  MSG_INT_BME280_CFG,
  MSG_MPU9250_CFG,
  MSG_BME280_CFG,
  MSG_UBLOX_CFG,
  MSG_SBUS_CFG,
  MSG_SWIFT_CFG,
  MSG_AMS5915_CFG,
  MSG_ANALOG_CFG,
  MSG_DIGITAL_CFG,
  MSG_VOLTAGE_CFG,
  MSG_TIME_DATA,
  MSG_ACCEL_DATA,
  MSG_GYRO_DATA,
  MSG_MAG_DATA,
  MSG_INCEPTOR_DATA,
  MSG_TEMPERATURE_DATA,
  MSG_GNSS_DATA,
  MSG_STATIC_PRESS_DATA,
  MSG_DIFF_PRESS_DATA,
  MSG_ANALOG_DATA,
  MSG_DIGITAL_DATA,
  MSG_VOLTAGE_DATA,
  NUM_MSG_ID
};
typedef enum MessageIds MsgId_t;

/*
* Message error codes.
*/
enum MessageError {
  MSG_ERROR_SUCCESS,
  MSG_ERROR_NULL_PTR,
  MSG_ERROR_INCORRECT_ID,
  MSG_ERROR_INCOMPLETE_MSG,
  MSG_ERROR_INCORRECT_CHECKSUM
};
typedef enum MessageError MsgError_t;

/*
* Structure to store message data.
*/
struct Message {
  MsgError_t tx_status;
  unsigned char *tx_buffer;
  unsigned int tx_len;
  unsigned int tx_hash;
  MsgId_t tx_id;
  void *tx_payload;
  MsgError_t rx_status;
  unsigned char *rx_buffer;
  unsigned char *temp_rx_buffer;
  unsigned int rx_len;
  unsigned int rx_hash;
  MsgId_t rx_id;
  MsgId_t temp_rx_id;
  void *rx_payload;
  unsigned int rx_state;
};
typedef struct Message Msg_t;
/* 
* Create a new message data structure and initialize values. 
* Returns a pointer on success and NULL on failure.
*/
Msg_t *msg_init();
/* 
* Frees the message data structure passed to it. Returns
* message error codes to indicate success or failure.
*/
MsgError_t msg_free(Msg_t *self);
/*
* Builds a transmit buffer given the message data structure,
* the component hash, the message id, and a payload. Returns
* message error codes to indicate success or failure.
*/
MsgError_t msg_build_tx(Msg_t *self,unsigned int hash,MsgId_t msg,void *payload);
/*
* Returns the message error code from the last call to
* msg_build_tx.
*/
MsgError_t msg_build_status(Msg_t *self);
/*
* Returns a pointer to the transmit buffer from the last
* call to msg_build_tx.
*/
unsigned char *msg_tx_buffer(Msg_t *self);
/*
* Returns the length of the transmit buffer from the last
* call to msg_build_tx.
*/
unsigned int msg_tx_len(Msg_t *self);
/*
* Returns the component hash from the last call to msg_build_tx.
*/
unsigned int msg_tx_hash(Msg_t *self);
/*
* Returns the message id from the last call to msg_build_tx.
*/
MsgId_t msg_tx_id(Msg_t *self);
/*
* Returns a pointer to the payload from the last call to msg_build_tx.
*/
void *msg_tx_payload(Msg_t *self);
/*
* Parses a message given a received byte at a time. Message error
* codes are returned to check status of parsing.
*/
MsgError_t msg_parse_rx(Msg_t *self,unsigned char byte);
/*
* Returns the message error code from the last call to
* msg_parse_rx.
*/
MsgError_t msg_parse_status(Msg_t *self);
/*
* Returns a pointer to the receive buffer from the last
* call to msg_parse_rx.
*/
unsigned char *msg_rx_buffer(Msg_t *self);
/*
* Returns the length of the receive buffer from the last
* call to msg_parse_rx.
*/
unsigned int msg_rx_len(Msg_t *self);
/*
* Returns the component hash from the last call to msg_parse_rx.
*/
unsigned int msg_rx_hash(Msg_t *self);
/*
* Returns the message id from the last call to msg_parse_rx.
*/
MsgId_t msg_rx_id(Msg_t *self);
/*
* Returns a pointer to the payload from the last call to msg_parse_rx.
*/
void *msg_rx_payload(Msg_t *self);

/*
* Ensure C friendly linkages in a mixed C/C++ build
*/
#ifdef __cplusplus
}
#endif

#endif
