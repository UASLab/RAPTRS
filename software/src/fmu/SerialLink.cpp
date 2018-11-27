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

#include "SerialLink.h"

/* interrupt chain to generate serial interrupts */
class interruptChain {
  public:
    void chainInterrupt(int intNumber, void (*interruptPtr)(void)) {
			prevInterruptPtr = _VectorsRam[intNumber];
			_VectorsRam[intNumber] = interruptPtr;
    }
    inline void callNext(void) {
      prevInterruptPtr();
    }
  protected:
    void (*prevInterruptPtr)(void);
};
interruptChain _serial1Interrupt, _serial2Interrupt, _serial3Interrupt;
/* serial1 interrupt handler */
void hdlc_serial1_int_handler(void)
{
  _serial1Interrupt.callNext();
  Serial1Link.checkReceived();
}
/* serial2 interrupt handler */
void hdlc_serial2_int_handler(void)
{
  _serial2Interrupt.callNext();
  Serial2Link.checkReceived();
}
/* serial3 interrupt handler */
void hdlc_serial3_int_handler(void)
{
  _serial3Interrupt.callNext();
  Serial3Link.checkReceived();
}
/* Assigning a hardware serial bus */
SerialLink::SerialLink(HardwareSerial& bus)
{
  _bus = &bus;
}
/* 
* Starting the bus at the specified baud rate.
*/
void SerialLink::begin(unsigned int baud)
{
  _bus->begin(baud);
  /* assigning the appropriate interrupt handler */
  if (_bus == &Serial1) {
    _serial1Interrupt.chainInterrupt(IRQ_UART0_STATUS + 16, hdlc_serial1_int_handler);
  }
  if (_bus == &Serial2) {
    _serial2Interrupt.chainInterrupt(IRQ_UART1_STATUS + 16, hdlc_serial2_int_handler);
  }
  if (_bus == &Serial3) {
    _serial3Interrupt.chainInterrupt(IRQ_UART2_STATUS + 16, hdlc_serial3_int_handler);
  } 
}
/*
* Starting to build a new packet to send.
*/
void SerialLink::beginTransmission()
{
  /* resetting payload length */
  _payload_len = 0;
  /* resetting the frame position */
  _send_fpos = 0;
  /* framing byte */
  _send_buf[_send_fpos++] = _frame_byte;
  /* control byte */
  _send_buf[_send_fpos++] = MsgType::COMMAND;
  _send_crc = _send_crc_16.x25(&_send_buf[1],_send_fpos - 1);
}
/*
* Add a byte to the send buffer.
*/
unsigned int SerialLink::write(unsigned char data)
{
  if (_payload_len < _max_payload_len) {
    _send_crc = _send_crc_16.x25_upd(&data,1);   
    if ((data == _frame_byte) || (data == _esc_byte)) {
      _send_buf[_send_fpos++] = _esc_byte;
      _send_buf[_send_fpos++] = data ^ _invert_byte;
    } else {
      _send_buf[_send_fpos++] = data;
    }
    _payload_len++;
    return 1;
  } else {
    return 0;
  }
}
/*
* Add data bytes to the send buffer.
*/
unsigned int SerialLink::write(unsigned char *data, unsigned int len)
{
  unsigned int avail = _max_payload_len - _payload_len;
  if (len > avail) {
    len = avail;
  }
  for (unsigned int i = 0; i < len; i++) {
    _send_crc = _send_crc_16.x25_upd(&data[i],1);
    if ((data[i] == _frame_byte) || (data[i] == _esc_byte)) {
      _send_buf[_send_fpos++] = _esc_byte;
      _send_buf[_send_fpos++] = data[i] ^ _invert_byte;
    } else {
      _send_buf[_send_fpos++] = data[i];
    }    
  }
  return len;
}
/*
* Send packet and wait indefinitely for ack.
*/
void SerialLink::endTransmission()
{
  unsigned char crc_bytes[2];
  /* crc */
  crc_bytes[0] = _send_crc & 0xFF;
  crc_bytes[1] = (_send_crc >> 8) & 0xFF;
  for (unsigned int i = 0; i < ARRAY_SIZE(crc_bytes); ++i) {
    if ((crc_bytes[i] == _frame_byte) || (crc_bytes[i] == _esc_byte)) {
      _send_buf[_send_fpos++] = _esc_byte;
      _send_buf[_send_fpos++] = crc_bytes[i] ^ _invert_byte;
    } else {
      _send_buf[_send_fpos++] = crc_bytes[i];    
    }
  }
  /* framing byte */
  _send_buf[_send_fpos++] = _frame_byte;
  /* update send status */
  _status = NACK;
  /* write frame */
  _bus->write(_send_buf,_send_fpos);
  elapsedMicros sendTime = 0;
  /* wait for ACK */
  while (_status != ACK) {
    if (sendTime > RETX_DELAY_US) {
      sendTime -= RETX_DELAY_US;
      _bus->write(_send_buf,_send_fpos);
    } 
  }
}
/*
* Send packet and wait for ack or timeout.
*/
void SerialLink::endTransmission(unsigned int timeout)
{
  unsigned char crc_bytes[2];
  /* crc */
  crc_bytes[0] = _send_crc & 0xFF;
  crc_bytes[1] = (_send_crc >> 8) & 0xFF;
  for (unsigned int i = 0; i < ARRAY_SIZE(crc_bytes); ++i) {
    if ((crc_bytes[i] == _frame_byte) || (crc_bytes[i] == _esc_byte)) {
      _send_buf[_send_fpos++] = _esc_byte;
      _send_buf[_send_fpos++] = crc_bytes[i] ^ _invert_byte;
    } else {
      _send_buf[_send_fpos++] = crc_bytes[i];    
    }
  }
  /* framing byte */
  _send_buf[_send_fpos++] = _frame_byte;
  /* update send status */
  _status = NACK;
  /* write frame */
  _bus->write(_send_buf,_send_fpos);
  /* wait for ACK */
  elapsedMicros t = 0, sendTime = 0;
  while ((_status != ACK) && (t < timeout)) {
    if (sendTime > RETX_DELAY_US) {
      sendTime -= RETX_DELAY_US;
      _bus->write(_send_buf,_send_fpos);
    } 
  }
}
/*
* Send packet without waiting for ack.
*/
void SerialLink::sendTransmission()
{
  unsigned char crc_bytes[2];
  /* crc */
  crc_bytes[0] = _send_crc & 0xFF;
  crc_bytes[1] = (_send_crc >> 8) & 0xFF;
  for (unsigned int i = 0; i < ARRAY_SIZE(crc_bytes); ++i) {
    if ((crc_bytes[i] == _frame_byte) || (crc_bytes[i] == _esc_byte)) {
      _send_buf[_send_fpos++] = _esc_byte;
      _send_buf[_send_fpos++] = crc_bytes[i] ^ _invert_byte;
    } else {
      _send_buf[_send_fpos++] = crc_bytes[i];    
    }
  }
  /* framing byte */
  _send_buf[_send_fpos++] = _frame_byte;
  /* update send status */
  _status = NACK;
  /* write frame */
  _bus->write(_send_buf,_send_fpos);
}
/*
* Check to see if we've received any new messages.
*/
bool SerialLink::checkReceived()
{
  int c;
  unsigned short crc;
  while (_bus->available()) {
    c = _bus->read();
    /* frame start */
    if (_recv_fpos == 0) {
      if (c == _frame_byte) {
        _recv_buf[_recv_fpos++] = c;
      }
    } else {
      if (c == _frame_byte) {
        /* frame end */
        if (_recv_fpos >= _header_len + _footer_len - 1) {
          /* passed crc check, good packet */
          crc = _recv_crc_16.x25(&_recv_buf[1],_recv_fpos - 3);
          if (crc == (((unsigned short)_recv_buf[_recv_fpos-1] << 8) | _recv_buf[_recv_fpos - 2])) {
            _msg_len = _recv_fpos - _header_len - _footer_len + 1; // +1 because we didn't step fpos
            _read_pos = _header_len;
            _recv_fpos = 0;
            _escape = false;
            _recv_type = (MsgType)_recv_buf[1];
            if ((_recv_type == ACK) || (_recv_type == NACK)) {
              _status = _recv_type;
            }
            if (((MsgType)_recv_buf[1]) == COMMAND) {
              _onReceive(_msg_len);
              _sendStatus(true);
            }
            return true;
          /* did not pass crc, bad packet */
          } else {
            _recv_fpos = 0;
            _escape = false;
            if (((MsgType)_recv_buf[1]) == COMMAND) {
              _sendStatus(false);
            }
            return false;
          }
        /* bad frame */
        } else {
          _recv_fpos = 0;
          _escape = false;
          return false;
        }
      } else if (c == _esc_byte) {
        _escape = true;
      } else if (_escape) {
        c = c ^ _invert_byte;
        _escape = false;
        /* read into buffer */
        _recv_buf[_recv_fpos++] = c;
      /* prevent buffer overflow */
      } else if (_recv_fpos >= BUFFER_SIZE) {
        _recv_fpos = 0;
        _escape = false;        
      } else {
        /* read into buffer */
        _recv_buf[_recv_fpos++] = c;
      }
    }
  }
  return false;
}
/*
* How many bytes available in our RX buffer
*/
unsigned int SerialLink::available()
{
  return _msg_len;
}
/*
* Read a byte
*/
unsigned char SerialLink::read()
{
  if (_msg_len > 0) {
    _msg_len--;
    return _recv_buf[_read_pos++];
  } else {
    return 0;
  }
}
/*
* Read several bytes
*/
unsigned int SerialLink::read(unsigned char *data, unsigned int len)
{
  if (len > _msg_len) {
    len = _msg_len;
  }
  memcpy(data,&_recv_buf[_read_pos],len);
  _read_pos += len;
  _msg_len -=len;
  return len;
}
/*
* Send ack or nack response
*/
void SerialLink::_sendStatus(bool ack)
{
  unsigned char buf[_header_len + 2 * _footer_len];
  unsigned char crc_bytes[2];
  unsigned short crc;
  unsigned int pos = 0;
  MsgType type;
  /* framing byte */
  buf[pos++] = _frame_byte;
  /* control byte */
  type = ack ? ACK : NACK;
  buf[pos++] = type;
  /* compute crc */
  crc = _recv_crc_16.x25(&buf[1],pos - 1);
  /* crc */
  crc_bytes[0] = crc & 0xFF;
  crc_bytes[1] = (crc >> 8) & 0xFF;
  for (unsigned int i = 0; i < ARRAY_SIZE(crc_bytes); ++i) {
    if ((crc_bytes[i] == _frame_byte) || (crc_bytes[i] == _esc_byte)) {
      buf[pos++] = _esc_byte;
      buf[pos++] = crc_bytes[i] ^ _invert_byte;
    } else {
      buf[pos++] = crc_bytes[i];    
    }
  }
  /* framing byte */
  buf[pos++] = _frame_byte;
  /* write frame */
  _bus->write(buf,pos);
}
/*
* Get ack or nack status
*/
bool SerialLink::getTransmissionStatus()
{
  return (_status == ACK) ? true : false;
}

SerialLink Serial1Link = SerialLink(Serial1);
SerialLink Serial2Link = SerialLink(Serial2);
SerialLink Serial3Link = SerialLink(Serial3);
