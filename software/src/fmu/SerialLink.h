/*
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2018 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef SERIALLINK_h
#define SERIALLINK_h

/* 
* Supported:
* Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC
*/
#if defined(__MK20DX128__) 	|| defined(__MK20DX256__) || defined(__MK64FX512__)	|| defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "array_macros.h"
#include "FastCRC.h"
#include "Arduino.h"

#define BUFFER_SIZE 1024
#define RETX_DELAY_US 500

class SerialLink {
	public:
		SerialLink(HardwareSerial& bus);
		void begin(unsigned int baud);
		void beginTransmission();
		unsigned int write(unsigned char data);
		unsigned int write(unsigned char *data, unsigned int len);
		void endTransmission();
		void endTransmission(unsigned int timeout);
		void sendTransmission();
		inline void onReceive(void (*function)(unsigned int len)) { _onReceive = function; }
		bool checkReceived();
		unsigned int available();
		unsigned char read();
		unsigned int read(unsigned char *data, unsigned int len);
		bool getTransmissionStatus();
	private:
		HardwareSerial* _bus;
		FastCRC16 _send_crc_16, _recv_crc_16;
		void (*_onReceive)(unsigned int len);
		enum MsgType {
			COMMAND,
			ACK,
			NACK
		};
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
		void _sendStatus(bool ack);
};
extern SerialLink Serial1Link;
extern SerialLink Serial2Link;
extern SerialLink Serial3Link;

#endif

#endif
