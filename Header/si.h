#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include <time.h>
#include <iostream>
#include <stdio.h>
#ifndef _si_INCLUDED_
#define _si_INCLUDED_
// Read 2 bytes from the SI7021
// First byte will be from 'address'
// Second byte will be from 'address'+1
int readRH(int file);
int readTemp(int file);
#endif
