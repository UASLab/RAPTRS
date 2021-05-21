/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

/*
* Supported:
* Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC
*/
#if defined(__MK20DX128__) 	|| defined(__MK20DX256__) || defined(__MK64FX512__)	|| defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "array_macros.h"
#include "FastCRC.h"
#include "Arduino.h"

#define BUFFER_SIZE 4096
#define RETX_DELAY_US 500

class SerialLink {
	public:
		enum MsgType {
			NACK = 0,
			ACK = 1,
			NOACK = 2,
			REQACK = 3
		};
		SerialLink(HardwareSerial& bus);
		void begin(unsigned int baud);
		void beginTransmission(MsgType type = REQACK);
		unsigned int write(unsigned char data);
		unsigned int write(unsigned char *data, unsigned int len);
		void sendTransmission();
		void endTransmission(bool ackReq = true);
		void endTransmission(unsigned int timeout);
		bool checkReceived();
		unsigned int available();
		unsigned char read();
		unsigned int read(unsigned char *data, unsigned int len);
		bool getTransmissionStatus();
		void sendStatus(bool ack);
	private:
		HardwareSerial* _bus;
		FastCRC16 _send_crc_16, _recv_crc_16;
		volatile MsgType _recv_type;
		volatile MsgType _status;
		static const unsigned char _frame_byte = 0x7E;
		static const unsigned char _esc_byte = 0x7D;
		static const unsigned char _invert_byte = 0x20;
		unsigned short _send_crc;
		static const unsigned int _header_len = 2;
		static const unsigned int _footer_len = 3;
		static const unsigned int _max_payload_len = (BUFFER_SIZE - _header_len - 2 * _footer_len) / 2;
		unsigned int _payload_len = 0;
		unsigned int _send_fpos = 0, _recv_fpos = 0;
		unsigned int _read_pos = _header_len, _msg_len = 0;
		bool _escape = false;
		unsigned char _send_buf[BUFFER_SIZE];
		unsigned char _recv_buf[BUFFER_SIZE];
};

#endif
