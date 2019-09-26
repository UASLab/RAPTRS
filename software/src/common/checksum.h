/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <stdlib.h>

/*
* This module contains checksum functions.
*/

/*
* Ensure C friendly linkages in a mixed C/C++ build
*/
#ifdef __cplusplus
extern "C" {
#endif

/* Fletcher-16 checksum */
unsigned short fletcher16(unsigned char *data,unsigned int len);

/*
* Ensure C friendly linkages in a mixed C/C++ build
*/
#ifdef __cplusplus
}
#endif

#endif
