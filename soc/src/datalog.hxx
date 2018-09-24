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

#include "definition-tree.hxx"
#include "hardware-defs.hxx"
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

class DatalogClient {
  public:
    DatalogClient();
    void RegisterGlobalData(DefinitionTree &DefinitionTreeRef);
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
    std::vector<std::string> SaveAsUint64Keys_;
    std::vector<std::string> SaveAsUint64Description_;
    std::vector<uint64_t*> SaveAsUint64Values_;
    std::vector<std::string> SaveAsUint32Keys_;
    std::vector<std::string> SaveAsUint32Description_;
    std::vector<uint32_t*> SaveAsUint32Values_;
    std::vector<std::string> SaveAsUint16Keys_;
    std::vector<std::string> SaveAsUint16Description_;
    std::vector<uint16_t*> SaveAsUint16Values_;
    std::vector<std::string> SaveAsUint8Keys_;
    std::vector<std::string> SaveAsUint8Description_;
    std::vector<uint8_t*> SaveAsUint8Values_;
    std::vector<std::string> SaveAsInt64Keys_;
    std::vector<std::string> SaveAsInt64Description_;
    std::vector<int64_t*> SaveAsInt64Values_;
    std::vector<std::string> SaveAsInt32Keys_;
    std::vector<std::string> SaveAsInt32Description_;
    std::vector<int32_t*> SaveAsInt32Values_;
    std::vector<std::string> SaveAsInt16Keys_;
    std::vector<std::string> SaveAsInt16Description_;
    std::vector<int16_t*> SaveAsInt16Values_;
    std::vector<std::string> SaveAsInt8Keys_;
    std::vector<std::string> SaveAsInt8Description_;
    std::vector<int8_t*> SaveAsInt8Values_;
    std::vector<std::string> SaveAsFloatKeys_;
    std::vector<std::string> SaveAsFloatDescription_;
    std::vector<float*> SaveAsFloatValues_;
    std::vector<std::string> SaveAsDoubleKeys_;
    std::vector<std::string> SaveAsDoubleDescription_;
    std::vector<double*> SaveAsDoubleValues_;
    std::vector<uint8_t> LogDataBuffer_;
    std::vector<uint8_t> Buffer_;
    const uint8_t header_[2] = {0x42,0x46};
    const uint8_t headerLength_ = 5;
    const uint8_t checksumLength_ = 2;
    uint8_t LengthBuffer_[2];
    uint16_t Length_ = 0;
    uint8_t Checksum_[2];
    uint16_t ParserState_ = 0;
    void SendBinary(DataType_ Type, std::vector<uint8_t> &Buffer);
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
    std::vector<uint8_t> Buffer_;
    bool FileExists(const std::string &FileName);
};

#endif
