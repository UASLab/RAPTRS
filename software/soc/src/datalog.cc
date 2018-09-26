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

#include "datalog.hxx"

/* Initializes the datalogger states and opens a socket for datalogging */
DatalogClient::DatalogClient() {
  DataLogSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  DataLogServer_.sin_family = AF_INET;
  DataLogServer_.sin_port = htons(DataLogPort_);
  DataLogServer_.sin_addr.s_addr = inet_addr("127.0.0.1");
}

/* Registers global data with the datalogger */
void DatalogClient::RegisterGlobalData(DefinitionTree &DefinitionTreeRef) {
  // Get all keys
  std::vector<std::string> Keys;
  DefinitionTreeRef.GetKeys("/",&Keys);
  // Find keys that are marked to be datalogged
  for (auto const & elem: Keys) {
    if (DefinitionTreeRef.GetDatalog(elem)) {
      // store keys, description, and value pointers
      if (DefinitionTreeRef.GetValuePtr<uint64_t*>(elem)) {
        SaveAsUint64Keys_.push_back(elem);
        SaveAsUint64Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsUint64Values_.push_back(DefinitionTreeRef.GetValuePtr<uint64_t*>(elem));
      }
      if (DefinitionTreeRef.GetValuePtr<uint32_t*>(elem)) {
        SaveAsUint32Keys_.push_back(elem);
        SaveAsUint32Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsUint32Values_.push_back(DefinitionTreeRef.GetValuePtr<uint32_t*>(elem));
      }
      if (DefinitionTreeRef.GetValuePtr<uint16_t*>(elem)) {
        SaveAsUint16Keys_.push_back(elem);
        SaveAsUint16Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsUint16Values_.push_back(DefinitionTreeRef.GetValuePtr<uint16_t*>(elem));
      }
      if (DefinitionTreeRef.GetValuePtr<uint8_t*>(elem)) {
        SaveAsUint8Keys_.push_back(elem);
        SaveAsUint8Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsUint8Values_.push_back(DefinitionTreeRef.GetValuePtr<uint8_t*>(elem));
      }
      if (DefinitionTreeRef.GetValuePtr<int64_t*>(elem)) {
        SaveAsInt64Keys_.push_back(elem);
        SaveAsInt64Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsInt64Values_.push_back(DefinitionTreeRef.GetValuePtr<int64_t*>(elem));
      }
      if (DefinitionTreeRef.GetValuePtr<int32_t*>(elem)) {
        SaveAsInt32Keys_.push_back(elem);
        SaveAsInt32Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsInt32Values_.push_back(DefinitionTreeRef.GetValuePtr<int32_t*>(elem));
      }
      if (DefinitionTreeRef.GetValuePtr<int16_t*>(elem)) {
        SaveAsInt16Keys_.push_back(elem);
        SaveAsInt16Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsInt16Values_.push_back(DefinitionTreeRef.GetValuePtr<int16_t*>(elem));
      }
      if (DefinitionTreeRef.GetValuePtr<int8_t*>(elem)) {
        SaveAsInt8Keys_.push_back(elem);
        SaveAsInt8Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsInt8Values_.push_back(DefinitionTreeRef.GetValuePtr<int8_t*>(elem));
      }
      if (DefinitionTreeRef.GetValuePtr<float*>(elem)) {
        SaveAsFloatKeys_.push_back(elem);
        SaveAsFloatDescription_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsFloatValues_.push_back(DefinitionTreeRef.GetValuePtr<float*>(elem));
      }
      if (DefinitionTreeRef.GetValuePtr<double*>(elem)) {
        SaveAsDoubleKeys_.push_back(elem);
        SaveAsDoubleDescription_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsDoubleValues_.push_back(DefinitionTreeRef.GetValuePtr<double*>(elem));
      }
    }
  }
  // define the data buffer size
  LogDataBuffer_.resize(
    SaveAsUint64Values_.size()*sizeof(uint64_t) +
    SaveAsUint32Values_.size()*sizeof(uint32_t) +
    SaveAsUint16Values_.size()*sizeof(uint16_t) +
    SaveAsUint8Values_.size()*sizeof(uint8_t) +
    SaveAsInt64Values_.size()*sizeof(int64_t) +
    SaveAsInt32Values_.size()*sizeof(int32_t) +
    SaveAsInt16Values_.size()*sizeof(int16_t) +
    SaveAsInt8Values_.size()*sizeof(int8_t) +
    SaveAsFloatValues_.size()*sizeof(float) +
    SaveAsDoubleValues_.size()*sizeof(double)
  );
  // send meta data to disk
  for (size_t i=0; i < SaveAsUint64Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint64Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint64Keys_[i][j]);
    }
    SendBinary(DataType_::Uint64Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint64Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint64Description_[i][j]);
    }
    SendBinary(DataType_::Uint64Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsUint32Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint32Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint32Keys_[i][j]);
    }
    SendBinary(DataType_::Uint32Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint32Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint32Description_[i][j]);
    }
    SendBinary(DataType_::Uint32Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsUint16Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint16Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint16Keys_[i][j]);
    }
    SendBinary(DataType_::Uint16Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint16Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint16Description_[i][j]);
    }
    SendBinary(DataType_::Uint16Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsUint8Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint8Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint8Keys_[i][j]);
    }
    SendBinary(DataType_::Uint8Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint8Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint8Description_[i][j]);
    }
    SendBinary(DataType_::Uint8Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt64Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt64Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt64Keys_[i][j]);
    }
    SendBinary(DataType_::Int64Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt64Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt64Description_[i][j]);
    }
    SendBinary(DataType_::Int64Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt32Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt32Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt32Keys_[i][j]);
    }
    SendBinary(DataType_::Int32Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt32Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt32Description_[i][j]);
    }
    SendBinary(DataType_::Int32Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt16Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt16Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt16Keys_[i][j]);
    }
    SendBinary(DataType_::Int16Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt16Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt16Description_[i][j]);
    }
    SendBinary(DataType_::Int16Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt8Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt8Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt8Keys_[i][j]);
    }
    SendBinary(DataType_::Int8Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt8Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt8Description_[i][j]);
    }
    SendBinary(DataType_::Int8Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsFloatKeys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsFloatKeys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsFloatKeys_[i][j]);
    }
    SendBinary(DataType_::FloatKey,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsFloatDescription_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsFloatDescription_[i][j]);
    }
    SendBinary(DataType_::FloatDesc,Buffer);
  }
  for (size_t i=0; i < SaveAsDoubleKeys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsDoubleKeys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsDoubleKeys_[i][j]);
    }
    SendBinary(DataType_::DoubleKey,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsDoubleDescription_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsDoubleDescription_[i][j]);
    }
    SendBinary(DataType_::DoubleDesc,Buffer);
  }
}

/* Sends binary data to be logged */
void DatalogClient::LogBinaryData() {
  size_t BufferLocation = 0;
  // payload
  for (size_t i=0; i < SaveAsUint64Values_.size(); i++) {
    memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsUint64Values_[i]),sizeof(uint64_t));
    BufferLocation += sizeof(uint64_t);
  }
  for (size_t i=0; i < SaveAsUint32Values_.size(); i++) {
    memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsUint32Values_[i]),sizeof(uint32_t));
    BufferLocation += sizeof(uint32_t);
  }
  for (size_t i=0; i < SaveAsUint16Values_.size(); i++) {
    memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsUint16Values_[i]),sizeof(uint16_t));
    BufferLocation += sizeof(uint16_t);
  }
  for (size_t i=0; i < SaveAsUint8Values_.size(); i++) {
    memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsUint8Values_[i]),sizeof(uint8_t));
    BufferLocation += sizeof(uint8_t);
  }
  for (size_t i=0; i < SaveAsInt64Values_.size(); i++) {
    memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsInt64Values_[i]),sizeof(int64_t));
    BufferLocation += sizeof(int64_t);
  }
  for (size_t i=0; i < SaveAsInt32Values_.size(); i++) {
    memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsInt32Values_[i]),sizeof(int32_t));
    BufferLocation += sizeof(int32_t);
  }
  for (size_t i=0; i < SaveAsInt16Values_.size(); i++) {
    memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsInt16Values_[i]),sizeof(int16_t));
    BufferLocation += sizeof(int16_t);
  }
  for (size_t i=0; i < SaveAsInt8Values_.size(); i++) {
    memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsInt8Values_[i]),sizeof(int8_t));
    BufferLocation += sizeof(int8_t);
  }
  for (size_t i=0; i < SaveAsFloatValues_.size(); i++) {
    memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsFloatValues_[i]),sizeof(float));
    BufferLocation += sizeof(float);
  }
  for (size_t i=0; i < SaveAsDoubleValues_.size(); i++) {
    memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsDoubleValues_[i]),sizeof(double));
    BufferLocation += sizeof(double);
  }
  // send data
  SendBinary(DataType_::Data,LogDataBuffer_);
}

/* Closes socket and clears states */
void DatalogClient::End() {
  SaveAsUint64Keys_.clear();
  SaveAsUint64Description_.clear();
  SaveAsUint64Values_.clear();
  SaveAsUint32Keys_.clear();
  SaveAsUint32Description_.clear();
  SaveAsUint32Values_.clear();
  SaveAsUint16Keys_.clear();
  SaveAsUint16Description_.clear();
  SaveAsUint16Values_.clear();
  SaveAsUint8Keys_.clear();
  SaveAsUint8Description_.clear();
  SaveAsUint8Values_.clear();
  SaveAsInt64Keys_.clear();
  SaveAsInt64Description_.clear();
  SaveAsInt64Values_.clear();
  SaveAsInt32Keys_.clear();
  SaveAsInt32Description_.clear();
  SaveAsInt32Values_.clear();
  SaveAsInt16Keys_.clear();
  SaveAsInt16Description_.clear();
  SaveAsInt16Values_.clear();
  SaveAsInt8Keys_.clear();
  SaveAsInt8Description_.clear();
  SaveAsInt8Values_.clear();
  SaveAsFloatKeys_.clear();
  SaveAsFloatDescription_.clear();
  SaveAsFloatValues_.clear();
  SaveAsDoubleKeys_.clear();
  SaveAsDoubleDescription_.clear();
  SaveAsDoubleValues_.clear();
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
