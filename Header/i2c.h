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

using namespace std;

//############## i2c functions ##############

int i2cCreate ()
{
	// Create I2C bus
	int file;
	char *bus = "/dev/i2c-0";
	if ((file = open(bus, O_RDWR)) < 0) 
	{
		cout<< "Failed to open the bus."<< endl;;
		exit(1);
	}
	return file;
}


void registerWrite(int bus, char address, char Register_to_set, char Data_for_set_to_register)
{
	ioctl(bus, I2C_SLAVE, address);
	
	char config[2] = {Register_to_set, Data_for_set_to_register};
	write(bus, config, 2);
}



void registerRead(int bus, unsigned char address, unsigned char reg, int num, char *data)
{
 	ioctl(bus, I2C_SLAVE, address);
    
	char config[1]={reg};
	write(bus, config, 1);
	read(bus, data, num);
}


void strRead(int bus, ofstream& log, char reg, int len)
{
    ioctl(bus,I2C_SLAVE,0x08);
    // append data to the log file
    log.open ("/home/final/log.txt", fstream::app);
    
    char config[1] = {reg};
	write(bus, config, 1);
    
	char data[10] = {};
  	read(bus, data, len);
	
	//double n;
	//n=stod(data);
	//cout<<"prescious: "n<<endl;
	
  	for(int i=0; i < len; i++)
  	{
	  	if(data[0]==NULL)
		{
			cout<< "no data";
			log<< "no data";
			break;
		}
  		cout<< data[i];
  		log<< data[i];
	}
	log<< "    ";
	cout<< endl;
	
	// close log file
	log.close();
}
