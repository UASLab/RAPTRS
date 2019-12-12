/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#ifndef ARRAY_MACROS_H
#define ARRAY_MACROS_H

/* Returns the number of elements in array */
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#endif
