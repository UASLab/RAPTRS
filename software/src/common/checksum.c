/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
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
