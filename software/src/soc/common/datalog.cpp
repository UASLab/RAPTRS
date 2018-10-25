/*
datalog.cxx
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

#include <iostream>
using std::cout;
using std::endl;

#include "datalog.h"

/* Initializes the datalogger states and opens a socket for datalogging */
DatalogClient::DatalogClient() {
  DataLogSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  DataLogServer_.sin_family = AF_INET;
  DataLogServer_.sin_port = htons(DataLogPort_);
  DataLogServer_.sin_addr.s_addr = inet_addr("127.0.0.1");
}

/* Registers global data with the datalogger */
void DatalogClient::RegisterGlobalData() {
  // Get all keys
  std::vector<std::string> Keys;
  deftree.GetKeys("/",&Keys);
  // Find keys that are marked to be datalogged
  for (auto const & key: Keys) {
      // store keys, description, and value pointers
      Element *ele = deftree.getElement(key);
      log_tag_t log_tag = ele->getLoggingType();
      if ( log_tag == LOG_UINT64 ) {
          SaveAsUint64Keys_.push_back(key);
          SaveAsUint64Nodes_.push_back(ele);
      } else if ( log_tag == LOG_UINT32 ) {
          SaveAsUint32Keys_.push_back(key);
          SaveAsUint32Nodes_.push_back(ele);
      } else if ( log_tag == LOG_UINT16 ) {
          SaveAsUint16Keys_.push_back(key);
          SaveAsUint16Nodes_.push_back(ele);
      } else if ( log_tag == LOG_UINT8 ) {
          SaveAsUint8Keys_.push_back(key);
          SaveAsUint8Nodes_.push_back(ele);
      } else if ( log_tag == LOG_INT64 ) {
          SaveAsInt64Keys_.push_back(key);
          SaveAsInt64Nodes_.push_back(ele);
      } else if ( log_tag == LOG_INT32 ) {
          SaveAsInt32Keys_.push_back(key);
          SaveAsInt32Nodes_.push_back(ele);
      } else if ( log_tag == LOG_INT16 ) {
          SaveAsInt16Keys_.push_back(key);
          SaveAsInt16Nodes_.push_back(ele);
      } else if ( log_tag == LOG_INT8 ) {
          SaveAsInt8Keys_.push_back(key);
          SaveAsInt8Nodes_.push_back(ele);
      } else if ( log_tag == LOG_FLOAT ) {
          SaveAsFloatKeys_.push_back(key);
          SaveAsFloatNodes_.push_back(ele);
      } else if ( log_tag == LOG_DOUBLE ) {
          SaveAsDoubleKeys_.push_back(key);
          SaveAsDoubleNodes_.push_back(ele);
      } else if ( log_tag == LOG_NONE ) {
          // skip
      } else {
          cout << "NOTICE: no valid log tag defined for: " << key << endl;
      }
  }
  // define the data buffer size
  LogDataBuffer_.resize(
    SaveAsUint64Nodes_.size()*sizeof(uint64_t) +
    SaveAsUint32Nodes_.size()*sizeof(uint32_t) +
    SaveAsUint16Nodes_.size()*sizeof(uint16_t) +
    SaveAsUint8Nodes_.size()*sizeof(uint8_t) +
    SaveAsInt64Nodes_.size()*sizeof(int64_t) +
    SaveAsInt32Nodes_.size()*sizeof(int32_t) +
    SaveAsInt16Nodes_.size()*sizeof(int16_t) +
    SaveAsInt8Nodes_.size()*sizeof(int8_t) +
    SaveAsFloatNodes_.size()*sizeof(float) +
    SaveAsDoubleNodes_.size()*sizeof(double)
  );
  // send meta data to disk
  for (size_t i=0; i < SaveAsUint64Nodes_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint64Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint64Keys_[i][j]);
    }
    SendBinary(DataType_::Uint64Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint64Nodes_[i]->description.size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint64Nodes_[i]->description[j]);
    }
    SendBinary(DataType_::Uint64Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsUint32Nodes_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint32Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint32Keys_[i][j]);
    }
    SendBinary(DataType_::Uint32Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint32Nodes_[i]->description.size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint32Nodes_[i]->description[j]);
    }
    SendBinary(DataType_::Uint32Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsUint16Nodes_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint16Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint16Keys_[i][j]);
    }
    SendBinary(DataType_::Uint16Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint16Nodes_[i]->description.size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint16Nodes_[i]->description[j]);
    }
    SendBinary(DataType_::Uint16Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsUint8Nodes_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint8Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint8Keys_[i][j]);
    }
    SendBinary(DataType_::Uint8Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint8Nodes_[i]->description.size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint8Nodes_[i]->description[j]);
    }
    SendBinary(DataType_::Uint8Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt64Nodes_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt64Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt64Keys_[i][j]);
    }
    SendBinary(DataType_::Int64Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt64Nodes_[i]->description.size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt64Nodes_[i]->description[j]);
    }
    SendBinary(DataType_::Int64Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt32Nodes_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt32Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt32Keys_[i][j]);
    }
    SendBinary(DataType_::Int32Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt32Nodes_[i]->description.size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt32Nodes_[i]->description[j]);
    }
    SendBinary(DataType_::Int32Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt16Nodes_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt16Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt16Keys_[i][j]);
    }
    SendBinary(DataType_::Int16Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt16Nodes_[i]->description.size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt16Nodes_[i]->description[j]);
    }
    SendBinary(DataType_::Int16Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt8Nodes_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt8Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt8Keys_[i][j]);
    }
    SendBinary(DataType_::Int8Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt8Nodes_[i]->description.size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt8Nodes_[i]->description[j]);
    }
    SendBinary(DataType_::Int8Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsFloatNodes_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsFloatKeys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsFloatKeys_[i][j]);
    }
    SendBinary(DataType_::FloatKey,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsFloatNodes_[i]->description.size(); j++) {
      Buffer.push_back((uint8_t)SaveAsFloatNodes_[i]->description[j]);
    }
    SendBinary(DataType_::FloatDesc,Buffer);
  }
  for (size_t i=0; i < SaveAsDoubleNodes_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsDoubleKeys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsDoubleKeys_[i][j]);
    }
    SendBinary(DataType_::DoubleKey,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsDoubleNodes_[i]->description.size(); j++) {
      Buffer.push_back((uint8_t)SaveAsDoubleNodes_[i]->description[j]);
    }
    SendBinary(DataType_::DoubleDesc,Buffer);
  }
}

/* Sends binary data to be logged */
void DatalogClient::LogBinaryData() {
  size_t BufferLocation = 0;
  // payload
  for (size_t i=0; i < SaveAsUint64Nodes_.size(); i++) {
      uint64_t tmp = SaveAsUint64Nodes_[i]->getLong();
      memcpy(LogDataBuffer_.data()+BufferLocation,&tmp,sizeof(uint64_t));
      BufferLocation += sizeof(uint64_t);
  }
  for (size_t i=0; i < SaveAsUint32Nodes_.size(); i++) {
      uint32_t tmp = SaveAsUint32Nodes_[i]->getInt();
      memcpy(LogDataBuffer_.data()+BufferLocation,&tmp,sizeof(uint32_t));
      BufferLocation += sizeof(uint32_t);
  }
  for (size_t i=0; i < SaveAsUint16Nodes_.size(); i++) {
      uint16_t tmp = SaveAsUint16Nodes_[i]->getInt();
      memcpy(LogDataBuffer_.data()+BufferLocation,&tmp,sizeof(uint16_t));
      BufferLocation += sizeof(uint16_t);
  }
  for (size_t i=0; i < SaveAsUint8Nodes_.size(); i++) {
      uint8_t tmp = SaveAsUint8Nodes_[i]->getInt();
      memcpy(LogDataBuffer_.data()+BufferLocation,&tmp,sizeof(uint8_t));
      BufferLocation += sizeof(uint8_t);
  }
  for (size_t i=0; i < SaveAsInt64Nodes_.size(); i++) {
      int64_t tmp = SaveAsInt64Nodes_[i]->getLong();
      memcpy(LogDataBuffer_.data()+BufferLocation,&tmp,sizeof(int64_t));
      BufferLocation += sizeof(int64_t);
  }
  for (size_t i=0; i < SaveAsInt32Nodes_.size(); i++) {
      int32_t tmp = SaveAsInt32Nodes_[i]->getInt();
      memcpy(LogDataBuffer_.data()+BufferLocation,&tmp,sizeof(int32_t));
      BufferLocation += sizeof(int32_t);
  }
  for (size_t i=0; i < SaveAsInt16Nodes_.size(); i++) {
      int16_t tmp = SaveAsInt16Nodes_[i]->getInt();
      memcpy(LogDataBuffer_.data()+BufferLocation,&tmp,sizeof(int16_t));
      BufferLocation += sizeof(int16_t);
  }
  for (size_t i=0; i < SaveAsInt8Nodes_.size(); i++) {
      int8_t tmp = SaveAsInt8Nodes_[i]->getInt();
      memcpy(LogDataBuffer_.data()+BufferLocation,&tmp,sizeof(int8_t));
      BufferLocation += sizeof(int8_t);
  }
  for (size_t i=0; i < SaveAsFloatNodes_.size(); i++) {
      float tmp = SaveAsFloatNodes_[i]->getFloat();
      memcpy(LogDataBuffer_.data()+BufferLocation,&tmp,sizeof(float));
      BufferLocation += sizeof(float);
  }
  for (size_t i=0; i < SaveAsDoubleNodes_.size(); i++) {
      double tmp = SaveAsDoubleNodes_[i]->getDouble();
      memcpy(LogDataBuffer_.data()+BufferLocation,&tmp,sizeof(double));
      BufferLocation += sizeof(double);
  }
  // send data
  SendBinary(DataType_::Data,LogDataBuffer_);
}

/* Closes socket and clears states */
void DatalogClient::End() {
  SaveAsUint64Keys_.clear();
  SaveAsUint64Nodes_.clear();
  SaveAsUint32Keys_.clear();
  SaveAsUint32Nodes_.clear();
  SaveAsUint16Keys_.clear();
  SaveAsUint16Nodes_.clear();
  SaveAsUint8Keys_.clear();
  SaveAsUint8Nodes_.clear();
  SaveAsInt64Keys_.clear();
  SaveAsInt64Nodes_.clear();
  SaveAsInt32Keys_.clear();
  SaveAsInt32Nodes_.clear();
  SaveAsInt16Keys_.clear();
  SaveAsInt16Nodes_.clear();
  SaveAsInt8Keys_.clear();
  SaveAsInt8Nodes_.clear();
  SaveAsFloatKeys_.clear();
  SaveAsFloatNodes_.clear();
  SaveAsDoubleKeys_.clear();
  SaveAsDoubleNodes_.clear();
  close(DataLogSocket_);
}

/* Sends byte buffer given meta data */
void DatalogClient::SendBinary(DataType_ Type, std::vector<uint8_t> &Buffer) {
  std::vector<uint8_t> LogBuffer;
  LogBuffer.resize(headerLength_ + Buffer.size() + checksumLength_);
  // header
  LogBuffer[0] = header_[0];
  LogBuffer[1] = header_[1];
  // data source
  LogBuffer[2] = (uint8_t) Type;
  // data length
  LogBuffer[3] = Buffer.size() & 0xff;
  LogBuffer[4] = Buffer.size() >> 8;
  // payload
  std::memcpy(LogBuffer.data()+headerLength_,Buffer.data(),Buffer.size());
  // checksum
  CalcChecksum((size_t)(Buffer.size()+headerLength_),LogBuffer.data(),Checksum_);
  LogBuffer[Buffer.size()+headerLength_] = Checksum_[0];
  LogBuffer[Buffer.size()+headerLength_+1] = Checksum_[1];
  // write to UDP
  sendto(DataLogSocket_,LogBuffer.data(),LogBuffer.size(),0,(struct sockaddr *)&DataLogServer_,sizeof(DataLogServer_));
}

/* Computes a two byte checksum. */
void DatalogClient::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    Checksum[0] += ByteArray[i];
    Checksum[1] += Checksum[0];
  }
}

/* Initializes the datalogger states, opens a socket for datalogging */
DatalogServer::DatalogServer() {
  size_t FileNameCounter = 0;
  std::string DataLogName;
  std::string DataLogBaseName = "data";
  std::string DataLogType = ".bin";
  DataLogName = DataLogBaseName + std::to_string(FileNameCounter) + DataLogType;
  while(FileExists(DataLogName)) {
    FileNameCounter++;
    DataLogName = DataLogBaseName + std::to_string(FileNameCounter) + DataLogType;
  }
  if ((LogFile_ = fopen(DataLogName.c_str(),"wb"))<0) {
    throw std::runtime_error("Datalog failed to open.");
  }
  DataLogSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  DataLogServer_.sin_family = AF_INET;
  DataLogServer_.sin_port = htons(DataLogPort_);
  DataLogServer_.sin_addr.s_addr = inet_addr("127.0.0.1");
  if (bind(DataLogSocket_, (struct sockaddr *) &DataLogServer_,sizeof(DataLogServer_)) < 0) {
    throw std::runtime_error("Error binding to UDP port.");
  }
  Buffer_.resize(kUartBufferMaxSize);
}

/* Write received data to log file */
void DatalogServer::ReceiveBinary() {
  ssize_t MessageSize = recv(DataLogSocket_,Buffer_.data(),Buffer_.size(),0);
  if (MessageSize > 0) {
    // write to disk
    fwrite(Buffer_.data(),MessageSize,1,LogFile_);
    fflush(LogFile_);
  }
}

/* Closes socket and clears states */
void DatalogServer::End() {
  close(DataLogSocket_);
}

/* Checks to see if a file exists, returns true if it does and false if it does not */
bool DatalogServer::FileExists(const std::string &FileName) {
  if (FILE *file = fopen(FileName.c_str(),"r")) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}
