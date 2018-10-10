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

#include "checksum.h"

unsigned short fletcher16(unsigned char *data,unsigned int len)
{
  unsigned short sum0, sum1;
  unsigned int i;
  if (!data) {return 0;}
  for (i = 0, sum0 = 0, sum1 = 0; i < len; ++i) {
    sum0 = (sum0 + data[i]) % 0xFF;
    sum1 = (sum1 + sum0) % 0xFF;
  }
  return (sum1 << 8) | sum0;
}
