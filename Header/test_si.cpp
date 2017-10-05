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
#include "si.h"
using namespace std;

// Global variables
double temp_code;
double Y_out1,Y_out2;

// Create I2C bus
int i2cCreate();
int main()
{
	int file;
	file = i2cCreate();
	ofstream log;
	//log.open ("log.txt");
	
	while(1)
	{	
		log.open ("/home/blink/log2.txt", fstream::app);
		cout<< "Temp: "<<readTemp(file)<<" C	";
		log<< "Temp: "<< readTemp(file)<< "\n";
		
		cout<< "Hum: "<<readRH(file)<<" %"<<endl;
		log<< "Hum: "<< readRH(file)<< "\n";
		
		log.close();
		usleep(1000000);
		
	}
	
}



int i2cCreate()
{
	// Create I2C bus
	int file;
	char *bus = "/dev/i2c-0";
	if ((file = open(bus, O_RDWR)) < 0) 
	{
		printf("Failed to open the bus. \n");
		exit(1);
	}
	return file;
}


//#######  #######
