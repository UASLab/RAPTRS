/*
datalog.hxx
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

#ifndef DATALOG_HXX_
#define DATALOG_HXX_

#include "definition-tree2.h"
#include "hardware-defs.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <exception>
#include <stdexcept>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>

using std::vector;
using std::string;

class DatalogClient {
  public:
    DatalogClient();
    void RegisterGlobalData();
    void LogBinaryData();
    void End();
  private:
    enum DataType_ {
      Uint64Key,
      Uint32Key,
      Uint16Key,
      Uint8Key,
      Int64Key,
      Int32Key,
      Int16Key,
      Int8Key,
      FloatKey,
      DoubleKey,
      Uint64Desc,
      Uint32Desc,
      Uint16Desc,
      Uint8Desc,
      Int64Desc,
      Int32Desc,
      Int16Desc,
      Int8Desc,
      FloatDesc,
      DoubleDesc,
      Data
    };
    int DataLogSocket_;
    int DataLogPort_ = 8000;
    struct sockaddr_in DataLogServer_;
    vector<string> SaveAsUint64Keys_;
    vector<ElementPtr> SaveAsUint64Nodes_;
    vector<string> SaveAsUint32Keys_;
    vector<ElementPtr> SaveAsUint32Nodes_;
    vector<string> SaveAsUint16Keys_;
    vector<ElementPtr> SaveAsUint16Nodes_;
    vector<string> SaveAsUint8Keys_;
    vector<ElementPtr> SaveAsUint8Nodes_;
    vector<string> SaveAsInt64Keys_;
    vector<ElementPtr> SaveAsInt64Nodes_;
    vector<string> SaveAsInt32Keys_;
    vector<ElementPtr> SaveAsInt32Nodes_;
    vector<string> SaveAsInt16Keys_;
    vector<ElementPtr> SaveAsInt16Nodes_;
    vector<string> SaveAsInt8Keys_;
    vector<ElementPtr> SaveAsInt8Nodes_;
    vector<string> SaveAsFloatKeys_;
    vector<ElementPtr> SaveAsFloatNodes_;
    vector<string> SaveAsDoubleKeys_;
    vector<ElementPtr> SaveAsDoubleNodes_;
    vector<uint8_t> LogDataBuffer_;
    vector<uint8_t> Buffer_;
    const uint8_t header_[2] = {0x42,0x46};
    const uint8_t headerLength_ = 5;
    const uint8_t checksumLength_ = 2;
    uint8_t LengthBuffer_[2];
    uint16_t Length_ = 0;
    uint8_t Checksum_[2];
    uint16_t ParserState_ = 0;
    void SendBinary(DataType_ Type, vector<uint8_t> &Buffer);
    void CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum);
};

class DatalogServer {
  public:
    DatalogServer();
    void ReceiveBinary();
    void End();
  private:
    FILE *LogFile_;
    int DataLogSocket_;
    int DataLogPort_ = 8000;
    struct sockaddr_in DataLogServer_;
    vector<uint8_t> Buffer_;
    bool FileExists(const string &FileName);
};

#endif
