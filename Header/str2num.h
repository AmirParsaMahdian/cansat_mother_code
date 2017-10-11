#pragma once
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
#include <fstream>
#include "i2c.h"


long double str2num(char *data, int len)
{
	long double a = 0;
	int dec = 0;
	
	for (int i=0; i<len; i++)
    {
    	if(data[i] == 46) // locates the decimal point
    	{
    		dec = i+1;
    		continue;
		}
	    a = a * 10;
    	a = a + (data[i] - 48);
	}
	
	a = a / pow(10,len-dec);
	return a;
}
