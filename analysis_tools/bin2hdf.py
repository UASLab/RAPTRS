'''
bin2hdf.py
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
'''

# import libraries
import h5py
import struct
import argparse
import sys

# class for parsing Bfs messages
class BfsMessage:
    _ParserState = 0
    _Length = 0
    _ReturnDataType = 0
    _Buffer = []
    _Payload = []
    _ReturnPayload = []
    _LengthBuffer = []
    _Checksum = [0,0]
    DataTypes = ("Uint64Key","Uint32Key","Uint16Key","Uint8Key","Int64Key","Int32Key","Int16Key","Int8Key","FloatKey","DoubleKey","Uint64Desc","Uint32Desc","Uint16Desc","Uint8Desc","Int64Desc","Int32Desc","Int16Desc","Int8Desc","FloatDesc","DoubleDesc","Data")
    def Parse(Self,ByteRead):
        Header = bytearray([0x42,0x46])
        HeaderLength = 5;
        if Self._ParserState < 2:
            if ByteRead == Header[Self._ParserState]:
                Self._Buffer.append(ByteRead)
                Self._ParserState += 1
                return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState == 2:
            Self._Buffer.append(ByteRead)
            Self._ParserState += 1
            return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState == 3:
            Self._Buffer.append(ByteRead)
            Self._LengthBuffer.append(ByteRead)
            Self._ParserState += 1
            return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState == 4:
            Self._Buffer.append(ByteRead)
            Self._LengthBuffer.append(ByteRead)
            Self._Length = struct.unpack_from('@H',bytearray(Self._LengthBuffer),0)[0]
            Self._ParserState += 1
            return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState < (Self._Length + HeaderLength):
            Self._Buffer.append(ByteRead)
            Self._Payload.append(ByteRead)
            Self._ParserState += 1
            return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState == (Self._Length + HeaderLength):
            Self._Checksum = Self.CalcChecksum(Self._Buffer)
            if ByteRead == Self._Checksum[0]:
                Self._ParserState += 1
                return False, Self._ReturnDataType, Self._ReturnPayload
            else:
                Self._ParserState = 0
                Self._Length = 0
                Self._Buffer = []
                Self._Payload = []
                Self._LengthBuffer = []
                Self._Checksum = [0,0]
                return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState == (Self._Length + HeaderLength + 1):
            if ByteRead == Self._Checksum[1]:
                Self._ReturnPayload = Self._Payload
                Self._ReturnDataType = Self._Buffer[2]
                Self._ParserState = 0
                Self._Length = 0
                Self._Buffer = []
                Self._Payload = []
                Self._LengthBuffer = []
                Self._Checksum = [0,0]
                return True, Self._ReturnDataType, Self._ReturnPayload
            else:
                Self._ParserState = 0
                Self._Length = 0
                Self._Buffer = []
                Self._Payload = []
                Self._LengthBuffer = []
                Self._Checksum = [0,0]
                return False, Self._ReturnDataType, Self._ReturnPayload
        else:
            Self._ParserState = 0
            Self._Length = 0
            Self._Buffer = []
            Self._Payload = []
            Self._LengthBuffer = []
            Self._Checksum = [0,0]
            return False, Self._ReturnDataType, Self._ReturnPayload

    def CalcChecksum(Self,ByteArray):
        Checksum = [0,0]
        for i in range(0,len(ByteArray)):
            Checksum[0] = (Checksum[0] + ByteArray[i]) % 256
            Checksum[1] = (Checksum[1] + Checksum[0]) % 256
        return Checksum

# function to see if file name exists
def FileExists(FileName):
    try:
        h5py.File(FileName,'r')
        return True
    except:
        return False

# Parse input argument, file name to open
parser = argparse.ArgumentParser()
parser.add_argument("file", help="convert flight data binary to HDF5 format, enter binary file name as argument")
parser.add_argument("--output", help="specify output file name", action="store")
args = parser.parse_args()
print "Bolder Flight Systems"
print "Flight Data Binary to HDF5 Converter"
print "Version 1.0.0 "
print ""
sys.stdout.write("Converting " + args.file + "...")
sys.stdout.flush()
# Open binary file
try:
    BinaryFile = open(args.file,'r')
except IOError:
    print "Could not read file " + args.file
    sys.exit()
# Read binary file and close
FileContents = BinaryFile.read()
BinaryFile.close()
# Create HDF5 file
if args.output:
    try:
        DataLogName = args.output
        DataLogFile = h5py.File(DataLogName,'w-',libver='earliest')
    except:
        print "Could not create output file " + args.output
        sys.exit()
else:
    FileNameCounter = 0
    DataLogBaseName = "data"
    DataLogType = ".h5";
    DataLogName = DataLogBaseName + str(FileNameCounter) + DataLogType
    while FileExists(DataLogName):
        FileNameCounter += 1
        DataLogName = DataLogBaseName + str(FileNameCounter) + DataLogType
    DataLogFile = h5py.File(DataLogName,'w-',libver='earliest')
# instance of BfsMessage class to parse file
DataLogMessage = BfsMessage()
Uint64Datasets = []
Uint64Counter=0
Uint32Datasets = []
Uint32Counter=0
Uint16Datasets = []
Uint16Counter=0
Uint8Datasets = []
Uint8Counter=0
Int64Datasets = []
Int64Counter=0
Int32Datasets = []
Int32Counter=0
Int16Datasets = []
Int16Counter=0
Int8Datasets = []
Int8Counter=0
FloatDatasets = []
FloatCounter=0
DoubleDatasets = []
DoubleCounter=0
# parse byte array
NumberDataPoints = 0
FileContentsBinary = bytearray(FileContents)
for k in range(0,len(FileContentsBinary)):
    ReadByte = FileContentsBinary[k]
    ValidMessage, DataType, Payload = DataLogMessage.Parse(ReadByte)
    if ValidMessage:
        if DataLogMessage.DataTypes[DataType] == 'Uint64Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += unichr(Payload[i])
            Uint64Datasets.append(DataLogFile.create_dataset(KeyName,(0, 1), maxshape=(None, 1), dtype='uint64'))
        if DataLogMessage.DataTypes[DataType] == 'Uint32Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += unichr(Payload[i])
            Uint32Datasets.append(DataLogFile.create_dataset(KeyName,(0, 1), maxshape=(None, 1), dtype='uint32'))
        if DataLogMessage.DataTypes[DataType] == 'Uint16Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += unichr(Payload[i])
            Uint16Datasets.append(DataLogFile.create_dataset(KeyName,(0, 1), maxshape=(None, 1), dtype='uint16'))
        if DataLogMessage.DataTypes[DataType] == 'Uint8Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += unichr(Payload[i])
            Uint8Datasets.append(DataLogFile.create_dataset(KeyName,(0, 1), maxshape=(None, 1), dtype='uint8'))
        if DataLogMessage.DataTypes[DataType] == 'Int64Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += unichr(Payload[i])
            Int64Datasets.append(DataLogFile.create_dataset(KeyName,(0, 1), maxshape=(None, 1), dtype='int64'))
        if DataLogMessage.DataTypes[DataType] == 'Int32Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += unichr(Payload[i])
            Int32Datasets.append(DataLogFile.create_dataset(KeyName,(0, 1), maxshape=(None, 1), dtype='int32'))
        if DataLogMessage.DataTypes[DataType] == 'Int16Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += unichr(Payload[i])
            Int16Datasets.append(DataLogFile.create_dataset(KeyName,(0, 1), maxshape=(None, 1), dtype='int16'))
        if DataLogMessage.DataTypes[DataType] == 'Int8Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += unichr(Payload[i])
            Int8Datasets.append(DataLogFile.create_dataset(KeyName,(0, 1), maxshape=(None, 1), dtype='int8'))
        if DataLogMessage.DataTypes[DataType] == 'FloatKey':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += unichr(Payload[i])
            FloatDatasets.append(DataLogFile.create_dataset(KeyName,(0, 1), maxshape=(None, 1), dtype='float'))
        if DataLogMessage.DataTypes[DataType] == 'DoubleKey':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += unichr(Payload[i])
            DoubleDatasets.append(DataLogFile.create_dataset(KeyName,(0, 1), maxshape=(None, 1), dtype='double'))
        if DataLogMessage.DataTypes[DataType] == 'Uint64Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += unichr(Payload[i])
            (Uint64Datasets[Uint64Counter]).attrs["Description"] = Desc
            Uint64Counter += 1
        if DataLogMessage.DataTypes[DataType] == 'Uint32Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += unichr(Payload[i])
            (Uint32Datasets[Uint32Counter]).attrs["Description"] = Desc
            Uint32Counter += 1
        if DataLogMessage.DataTypes[DataType] == 'Uint16Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += unichr(Payload[i])
            (Uint16Datasets[Uint16Counter]).attrs["Description"] = Desc
            Uint16Counter += 1
        if DataLogMessage.DataTypes[DataType] == 'Uint8Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += unichr(Payload[i])
            (Uint8Datasets[Uint8Counter]).attrs["Description"] = Desc
            Uint8Counter += 1
        if DataLogMessage.DataTypes[DataType] == 'Int64Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += unichr(Payload[i])
            (Int64Datasets[Int64Counter]).attrs["Description"] = Desc
            Int64Counter += 1
        if DataLogMessage.DataTypes[DataType] == 'Int32Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += unichr(Payload[i])
            (Int32Datasets[Int32Counter]).attrs["Description"] = Desc
            Int32Counter += 1
        if DataLogMessage.DataTypes[DataType] == 'Int16Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += unichr(Payload[i])
            (Int16Datasets[Int16Counter]).attrs["Description"] = Desc
            Int16Counter += 1
        if DataLogMessage.DataTypes[DataType] == 'Int8Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += unichr(Payload[i])
            (Int8Datasets[Int8Counter]).attrs["Description"] = Desc
            Int8Counter += 1
        if DataLogMessage.DataTypes[DataType] == 'FloatDesc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += unichr(Payload[i])
            (FloatDatasets[FloatCounter]).attrs["Description"] = Desc
            FloatCounter += 1
        if DataLogMessage.DataTypes[DataType] == 'DoubleDesc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += unichr(Payload[i])
            (DoubleDatasets[DoubleCounter]).attrs["Description"] = Desc
            DoubleCounter += 1
        if DataLogMessage.DataTypes[DataType] == 'Data':
            offset = 0
            for i in range (0,len(Uint64Datasets)):
                (Uint64Datasets[i]).resize(NumberDataPoints+1,axis=0)
                (Uint64Datasets[i])[NumberDataPoints] = struct.unpack_from('@Q',bytearray(Payload),offset)[0]
                offset += 8
            for i in range (0,len(Uint32Datasets)):
                (Uint32Datasets[i]).resize(NumberDataPoints+1,axis=0)
                (Uint32Datasets[i])[NumberDataPoints] = struct.unpack_from('@I',bytearray(Payload),offset)[0]
                offset += 4
            for i in range (0,len(Uint16Datasets)):
                (Uint16Datasets[i]).resize(NumberDataPoints+1,axis=0)
                (Uint16Datasets[i])[NumberDataPoints] = struct.unpack_from('@H',bytearray(Payload),offset)[0]
                offset += 2
            for i in range (0,len(Uint8Datasets)):
                (Uint8Datasets[i]).resize(NumberDataPoints+1,axis=0)
                (Uint8Datasets[i])[NumberDataPoints] = struct.unpack_from('@B',bytearray(Payload),offset)[0]
                offset += 1
            for i in range (0,len(Int64Datasets)):
                (Int64Datasets[i]).resize(NumberDataPoints+1,axis=0)
                (Int64Datasets[i])[NumberDataPoints] = struct.unpack_from('@q',bytearray(Payload),offset)[0]
                offset += 8
            for i in range (0,len(Int32Datasets)):
                (Int32Datasets[i]).resize(NumberDataPoints+1,axis=0)
                (Int32Datasets[i])[NumberDataPoints] = struct.unpack_from('@i',bytearray(Payload),offset)[0]
                offset += 4
            for i in range (0,len(Int16Datasets)):
                (Int16Datasets[i]).resize(NumberDataPoints+1,axis=0)
                (Int16Datasets[i])[NumberDataPoints] = struct.unpack_from('@h',bytearray(Payload),offset)[0]
                offset += 2
            for i in range (0,len(Int8Datasets)):
                (Int8Datasets[i]).resize(NumberDataPoints+1,axis=0)
                (Int8Datasets[i])[NumberDataPoints] = struct.unpack_from('@b',bytearray(Payload),offset)[0]
                offset += 1
            for i in range (0,len(FloatDatasets)):
                (FloatDatasets[i]).resize(NumberDataPoints+1,axis=0)
                (FloatDatasets[i])[NumberDataPoints] = struct.unpack_from('@f',bytearray(Payload),offset)[0]
                offset += 4
            for i in range (0,len(DoubleDatasets)):
                (DoubleDatasets[i]).resize(NumberDataPoints+1,axis=0)
                (DoubleDatasets[i])[NumberDataPoints] = struct.unpack_from('@d',bytearray(Payload),offset)[0]
                offset += 8
            DataLogFile.flush()
            NumberDataPoints += 1
DataLogFile.close()
print "done!"
print "Created data log file " + DataLogName
