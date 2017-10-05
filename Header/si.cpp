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
#include "si.h"
int readTemp(int file)
{
	float temperature;
	double temp_code;
	ioctl(file,I2C_SLAVE,0x40);
    
	char config[1]={0xE3};
	write(file,config,1);
	
	char data[2];
	read(file,data,2);
  	
    temp_code = (data[0]<<8) + data[1];
   	temperature = ((175.72 * temp_code)/65536)-46.85;
	 
	return (temperature);

}


int readRH(int file)
{
	float RH;
	double Y_out1, Y_out2;
	ioctl(file,I2C_SLAVE,0x40);
    
	char config[1]={0xE5};
	write(file,config,1);
	
	char data[2];
	read(file,data,2);
	
	Y_out1 = (125*(data[0]/100)*25600)>>16;
    Y_out2 = (125*((data[0]%100)*256+data[1]))>>16;
	
   	RH = Y_out1 + Y_out2 - 6;
	 
	return (RH);
}


//#######  #######
