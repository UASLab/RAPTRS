#!/usr/bin/python3

'''
bin2hdf.py
Brian R Taylor
brian.taylor@bolderflight.com
mod's by Curtis L. Olson <olson126@umn.edu>

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
import numpy as np
import struct
import argparse
import os
import re
import sys

# class for parsing Bfs messages
class BfsMessage:
    state = 0
    length = 0
    buf = []
    payload = []
    length_buf = []
    checksum = [0,0]
    DataTypes = ("Uint64Key", "Uint32Key", "Uint16Key", "Uint8Key", "Int64Key",
                 "Int32Key", "Int16Key", "Int8Key", "FloatKey", "DoubleKey",
                 "Uint64Desc", "Uint32Desc", "Uint16Desc", "Uint8Desc",
                 "Int64Desc", "Int32Desc", "Int16Desc", "Int8Desc",
                 "FloatDesc", "DoubleDesc", "Data")
    
    def Parse(self, ByteRead):
        Header = bytearray([0x42,0x46])
        HeaderLength = 5;
        if self.state < 2:
            if self.state == 0:
                self.length = 0
                self.buf = []
                self.payload = []
                self.length_buf = []
                self.checksum = [0,0]
            if ByteRead == Header[self.state]:
                self.buf.append(ByteRead)
                self.state += 1
            return None, None
        elif self.state == 2:
            self.buf.append(ByteRead)
            self.state += 1
            return None, None
        elif self.state == 3:
            self.buf.append(ByteRead)
            self.length_buf.append(ByteRead)
            self.state += 1
            return None, None
        elif self.state == 4:
            self.buf.append(ByteRead)
            self.length_buf.append(ByteRead)
            self.length = struct.unpack_from('@H', bytearray(self.length_buf), 0)[0]
            self.state += 1
            return None, None
        elif self.state < (self.length + HeaderLength):
            self.buf.append(ByteRead)
            self.payload.append(ByteRead)
            self.state += 1
            return None, None
        elif self.state == (self.length + HeaderLength):
            self.checksum = self.CalcChecksum(self.buf)
            if ByteRead == self.checksum[0]:
                self.state += 1
            else:
                self.state = 0
            return None, None
        elif self.state == (self.length + HeaderLength + 1):
            if ByteRead == self.checksum[1]:
                self.state = 0
                DataType = self.DataTypes[self.buf[2]]
                return DataType, self.payload
            else:
                self.state = 0
                return None, None
        else:
            self.state = 0
            return None, None

    def CalcChecksum(self, ByteArray):
        Checksum = [0,0]
        for i in range(0, len(ByteArray)):
            Checksum[0] = (Checksum[0] + ByteArray[i]) % 256
            Checksum[1] = (Checksum[1] + Checksum[0]) % 256
        return Checksum

# Parse input argument, file name to open
parser = argparse.ArgumentParser()
parser.add_argument("file", help="convert flight data binary to HDF5 format, enter binary file name as argument")
parser.add_argument("--output", help="specify output file name", action="store")
args = parser.parse_args()

print("Bolder Flight Systems")
print("Flight Data Binary to HDF5 Converter")
print("Version 1.0.1")
print("")

print("Converting:", args.file)
# Open binary file
try:
    BinaryFile = open(args.file, 'rb')
except IOError:
    print("Could not read file:", args.file)
    sys.exit()
# Read binary file and close
FileContents = BinaryFile.read()
BinaryFile.close()
print('read bytes:', len(FileContents))

# build up our in memory data structures
# storage is a top level map for each type
# types is an ordered list of type names (the order is important in places)
# for each type we store:
#   the numpy type string
#   the pack/unpack type code
#   a list of keys for each column of this data type
#   a list of descriptions for each column of this data type
#   a list of data values (another list) for each column of this data type

storage = {}
types = []
counter = 0

def init_type(type, np_type, packcode):
    global storage
    global types
    types.append(type)
    storage[type] = { 'np_type': np_type,
                      'packcode': packcode,
                      'packstr': '',
                      'sizeof': 0,
                      'keys': [],
                      'desc': [],
                      'data': [] }

def init_storage():
    global storage
    global types
    global counter
    storage = {}
    types = []
    counter = 0
    # the order of initialization is important here and must match the order
    # in the binary data file.
    init_type('Uint64', np_type='uint64', packcode='Q')
    init_type('Uint32', np_type='uint32', packcode='I')
    init_type('Uint16', np_type='uint16', packcode='H')
    init_type('Uint8', np_type='uint8', packcode='B')
    init_type('Int64', np_type='int64', packcode='q')
    init_type('Int32', np_type='int32', packcode='i')
    init_type('Int16', np_type='int16', packcode='h')
    init_type('Int8', np_type='int8', packcode='b')
    init_type('Float', np_type='float', packcode='f')
    init_type('Double', np_type='double', packcode='d')

def write_storage():
    # Create HDF5 file
    if args.output:
        DataLogName = args.output
    else:
        FileNameCounter = 0
        DataLogBaseName = "data"
        DataLogType = ".h5";
        DataLogName = DataLogBaseName + "%03d" % FileNameCounter + DataLogType
        while os.path.isfile(DataLogName):
            FileNameCounter += 1
            DataLogName = DataLogBaseName + "%03d" % FileNameCounter + DataLogType
    print("Data log file name:", DataLogName)
    try:
        DataLogFile = h5py.File(DataLogName, 'w-', libver='earliest')
    except:
        print("Could not create output file:", args.output)
        sys.exit()

    print("Writing hdf5 file...")
    for t in types:
        for i in range (len(storage[t]['keys'])):
            d = DataLogFile.create_dataset(storage[t]['keys'][i], (counter, 1),
                                           data=np.array(storage[t]['data'][i]),
                                           dtype=storage[t]['np_type'],
                                           compression="gzip",
                                           compression_opts=9)
            d.attrs["Description"] = storage[t]['desc'][i]
    DataLogFile.close()
    print("Finished writing hdf5 file:", DataLogName)
    
# instance of BfsMessage class to parse file
DataLogMessage = BfsMessage()

# create the working space
init_storage()

# parse byte array
pkey = re.compile('(.+)Key')
pdesc = re.compile('(.+)Desc')

data_latch = False

FileContentsBinary = bytearray(FileContents)
print('parsing binary file...')
for k in range(0, len(FileContentsBinary)):
    #for testing/debugging
    #if counter > 500:
    #    break
    ReadByte = FileContentsBinary[k]
    DataType, Payload = DataLogMessage.Parse(ReadByte)
    if DataType:
        # match 'Key' token
        mkey = pkey.match(DataType)
        if mkey != None:
            if data_latch:
                write_storage()
                init_storage()
                data_latch = False
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            mtype = mkey[1]
            storage[mtype]['keys'].append(KeyName)
            storage[mtype]['data'].append([])
            storage[mtype]['packstr'] += storage[mtype]['packcode']
            storage[mtype]['sizeof'] = struct.calcsize(storage[mtype]['packstr'])
        # match 'Desc' token
        mdesc = pdesc.match(DataType)
        if mdesc != None:
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            dtype = mdesc[1]
            storage[dtype]['desc'].append(Desc)
        # match 'Data' token
        if DataType == 'Data':
            data_latch = True
            offset = 0
            for t in types:
                vals = struct.unpack_from(storage[t]['packstr'],
                                          bytearray(Payload), offset)
                for i in range(len(vals)):
                    storage[t]['data'][i].append(vals[i])
                offset += storage[t]['sizeof']
            counter += 1
            if (counter % 500) == 0:
                print("Scanning:", "%.0f seconds (%d bytes)" % (counter/50, k))

write_storage()
