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
// Creates the i2c bus
int i2cCreate ();

// Writes data to specified register
void registerWrite(int bus, char address, char Register_to_set, char Data_for_set_to_register);

// Reads Register and returns value in different ways depending on the sensor need
int registerRead(int bus, unsigned char address, unsigned char reg);

// Reads the data coming form the microcontroller as strings using dtostrf function
void strRead(int bus, ofstream& log, char address, int len); 


//############## SI7021 ##############
// Reads temperature ( mode = 't' ) and humidity ( mode = 'h' )
float si7021Read(int bus, char mode);


//############## BMP085 ##############
const unsigned char OSS = 0;  // Oversampling Setting
bool bmp = 0;                 // Determines whether calibrate sensor (0), read Temperature (0) or Pressure (1)
// Calibration values
int ac1 ,ac2, ac3, b1, b2, mb, mc, md, b5;
unsigned int ac4, ac5, ac6;

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void setup_BMP085(int bus);

// Read the uncompensated temperature value
// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
// Read the uncompensated pressure value
// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
float bmp085Read(int bus, char mode);


//############## HMC5883 ##############
// Setup sensor and set registers
void setup_HMC5883(int bus);

// read data from sensor and return what we want
// if select_output == x return raw x 
// if select_output == y return raw y 
// if select_output == z return raw z 
// if select_output == h return head angle
float hmc5883Read(int bus, char select_output);


//############## ADXL345 ##############
// Setup sensor and set registers
void setup_ADXL345(int bus);
double adxl345Read(int bus, char axis);


//############## L3G4200D ##############
// Setup sensor and set registers
void setup_L3G4200D(int bus);
int l3g4200dRead(int bus, char axis);



//############## Main ##############

int main()
{
	int bus;
	bus = i2cCreate();
	
	// Setup BMP085
	setup_BMP085(bus);
	
	// Setup HMC5883
	setup_HMC5883(bus);
	
	// Setup ADXL345
	setup_ADXL345(bus);
	
	// Setup L3G4200D
	setup_L3G4200D(bus);
	
	ofstream log;
	// append data to the log file
	log.open ("/home/final/log.txt", fstream::app);
	log<< "Sadra CanSat Team\n\n";
	log<< "Time(ms)    Temperature(C)    Humidity(%)    Available sattelites    Latitude(degree)    Longitude(degree)    Altitide    Speed(Km/h)   accX(m/s^2)    accYaccX(m/s^2)    accZaccX(m/s^2)    Pressure(Pa)    Temperature(C)    Head(degree)";
	// close log file
	log.close();
	
	while(1)
	{		
		// append data to the log file
		log.open ("/home/final/log.txt", fstream::app);
		
		log<< "\n"<< "___________________________________________________________________________________________________________________________________________________________________"<< "\n";
		
		log<< "    ";
		//cout<< time()<< " ms	";
		
		
		// Print Temperature from SL7021
		log<< si7021Read(bus, 't')<< "    ";
		cout<<"Temp: "<< si7021Read(bus, 't')<<endl;
		
		// Print Humidity from SL7021
		log<< si7021Read(bus, 'h')<< "    ";
		cout<<"Hum: "<< si7021Read(bus, 'h')<<endl;
		
		
		//cout<< "Available sattelites: ";
		strRead(bus, log, 1, 1);
		
		//cout<< "Laitude: ";
		strRead(bus, log, 2, 9);
		
		//cout<< "Longitude: ";
		strRead(bus, log, 3, 9);
		
		//cout<< "Altitide: ";
		strRead(bus, log, 4, 7);
		
		//cout<< "Speed: ";
		strRead(bus, log, 5, 6);
		
		cout<< "UV: ";
		strRead(bus, log, 6, 4);
		
		// Print heading from HMC5883
		cout<< "head = "<< hmc5883Read(bus, 'h')<< endl;
		log<< hmc5883Read(bus, 'h')<< "    ";
		
		// BMP085 local variables
		float pressure, temperature, altitude;
		// Print temperature from BMP085
		temperature = bmp085Read(bus, 't');
		cout << "temp: " << temperature << endl;
		log<< temperature << "    ";
		
		// Print pressure from BMP085
		pressure = bmp085Read(bus, 'p');
  		cout << "pre: " << pressure << endl;
  		log<< pressure << "    ";
  		
  		// Print altitude from the international barometric formula
		altitude = 44330 * ( 1 - pow((pressure / 101325) , ( 1 / 5.255 )));
		cout << "alt: " << altitude << endl;
		log<< altitude<< "    ";
		
		// Print gyroscope data from L3G4200D
		cout<< l3g4200dRead(bus, 'x')<< endl;
		cout<< l3g4200dRead(bus, 'y')<< endl;
		cout<< l3g4200dRead(bus, 'z')<< endl;
		
		// Print accelometer data from ADXL345
		cout << "Ax: " << adxl345Read(bus, 'x') << endl;
		log<< adxl345Read(bus, 'x') << "    ";	
		cout << "Ay: " << adxl345Read(bus, 'y') << endl;
		log<< adxl345Read(bus, 'y') << "    ";
		cout << "Az: " << adxl345Read(bus, 'z') << endl;
		log<< adxl345Read(bus, 'z') << "    ";
		
		// Close log file
		log.close();
		
		usleep(100000); //Just here to slow down the serial to make it more readable
		
	}
}

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


int registerRead(int bus, unsigned char address, unsigned char reg)
{
 	ioctl(bus, I2C_SLAVE, address);
    
	char config[1]={reg};
	write(bus, config, 1);
	
	// SI7021 Temperature
	if(address == 0x40 && reg == 0xE3)
	{
		char data[2]={0};
		read(bus, data, 2);
		return  (int16_t)(data[0] << 8 | data[1]);
	}
	
	// SI7021 Humidity
	if(address == 0x40 && reg == 0xE5)
	{
		char data[2]={0};
		read(bus, data, 2);
		double Y_out1, Y_out2;
		Y_out1 = (125*(data[0]/100)*25600)>>16;
    	Y_out2 = (125*((data[0]%100)*256+data[1]))>>16;
    	
		return (float)(Y_out1 + Y_out2 - 6);
	}
	
	// BMP085 Temperature
	if(address == 0x77 && bmp == 0)
	{
		char data[2]={0};
		read(bus, data, 2);
  
		return (int16_t)(data[0] << 8 | data[1]);
	}
	
	// BMP085 Pressure
	if(address == 0x77 && bmp == 1)
	{		
		char data[3]={0};
		read(bus, data, 3);
		
		return (((int16_t) data[0] << 16) | ((int16_t) data[1] << 8) | (int16_t) data[2]) >> (8-OSS);
	}
	
	// ADXL345 Accelometer
	if(address == 0x53)
	{
		char data[2]={0};
		read(bus, data, 2);
		return (int16_t)(data[1] << 8 | data[0]);
	}
	
	// L3G4200D Gyroscope
	if(address == 0x69)
	{
		char data[1]={0};
		read(bus, data, 1);
		return (int16_t)data[0];
	}
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
	
  	for(int i=0; i < len+1; i++)
  	{
	  	if(data[0]==NULL)
		{
			//cout<< "no data";
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

//############## SI7021 ##############

float si7021Read(int bus, char mode)
{
	if ( mode == 't')
	{
		double temp_code;
		temp_code = registerRead(bus, 0x40, 0xE3);
		return (float)(((175.72 * temp_code)/65536)-46.85);
	}
	if ( mode == 'h')
	{
		return (float)registerRead(bus, 0x40, 0xE5);
	}
}

//############## BMP085 ##############

void setup_BMP085(int bus)
{
	ac1 = registerRead(bus, 0x77, 0xAA) ;
	ac2 = registerRead(bus, 0x77, 0xAC) ;
	ac3 = registerRead(bus, 0x77, 0xAE) ;
	ac4 = registerRead(bus, 0x77, 0xB0) ;
	ac5 = registerRead(bus, 0x77, 0xB2) ;
	ac6 = registerRead(bus, 0x77, 0xB4) ;
	b1 = registerRead(bus, 0x77, 0xB6) ;
	b2 = registerRead(bus, 0x77, 0xB8) ;
	mb = registerRead(bus, 0x77, 0xBA) ;
	mc = registerRead(bus, 0x77, 0xBC) ;
	md = registerRead(bus, 0x77, 0xBE) ;
}


float bmp085Read(int bus, char mode)
{
	if ( mode == 't')
	{
		unsigned int ut;
  
		registerWrite(bus, 0x77, 0xF4, 0x2E);
	  
		// Wait at least 4.5ms
		usleep(5000);
	  
		// Read two bytes from registers 0xF6 and 0xF7
		bmp = 0; // Temperature mode
		ut = registerRead(bus, 0x77, 0xF6);
		
		long x1, x2;
		x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
		x2 = ((long)mc << 11)/(x1 + md);
		b5 = x1 + x2;
		
		return (float)((b5 + 8)>>4)/10;
	}
	
	if ( mode == 'p')
	{
		unsigned long up = 0;
	    
		// Write 0x34+(OSS<<6) into register 0xF4
		// Request a pressure reading w/ oversampling setting
		registerWrite(bus, 0x77, 0xF4, 0x34+(OSS<<6));
	  
		// Wait for conversion, delay time dependent on OSS
		usleep((2 + (3<<OSS))*1000);
		
		bmp = 1; // Pressure mode
		up = registerRead(bus, 0x77, 0xF6);
	
	 	long x1, x2, x3, b3, b6, p;
		unsigned long b4, b7;
		
		b6 = b5 - 4000;
		
		// Calculate B3
		x1 = (b2 * (b6 * b6)>>12)>>11;
		x2 = (ac2 * b6)>>11;
		x3 = x1 + x2;
		b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
	  
		// Calculate B4
		x1 = (ac3 * b6)>>13;
		x2 = (b1 * ((b6 * b6)>>12))>>16;
		x3 = ((x1 + x2) + 2)>>2;
		b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
	  
		b7 = ((unsigned long)(up - b3) * (50000>>OSS));
		if (b7 < 0x80000000)
			p = (b7<<1)/b4;
		else
			p = (b7/b4)<<1;
	    
		x1 = (p>>8) * (p>>8);
		x1 = (x1 * 3038)>>16;
		x2 = (-7357 * p)>>16;
		p += (x1 + x2 + 3791)>>4;
	  
		return (float)p;
	}
	
}


//############## HMC5883 ##############

void setup_HMC5883(int bus)
{
	//Register 0x00
	/*
	CRA7  CRA6    CRA5    CRA4     CRA3     CRA2     CRA1     CRA0 
	(0)   MA1(0)  MA0(0)  DO2_(1)  DO1_(0)  DO0_(0)  MS1_(0)  MS0_(0)
	_____________________________________________
	|DO2 |DO1 |DO0 |Typical Data Output Rate (Hz)|
	|____|____|____|_____________________________|
	|0   |0   |0   |0.75                         |
	|0   |0   |1   |1.5                          |
	|0   |1   |0   |3                            |
	|0   |1   |1   |7.5                          |
	|1   |0   |0   |15 (Default)                 |
	|1   |0   |1   |30                           |
	|1   |1   |0   |75                           |
	|1   |1   |1   |Reserved                     |
	|____|____|____|_____________________________|



   |MS1|MS0|Measurement Mode 
   |___|___|____________________________
   |0  |0 
   |   |   |Normal measurement configuration (Default). In normal measurement 
   |   |   |configuration the device follows normal measurement flow. The positive and 
   |   |   |negative pins of the resistive load are left floating and high impedance. 
   |___|___|__________________________________________________
   |0   1 
   |		Positive bias configuration for X, Y, and Z axes. In this configuration, a positive 
   |		current is forced across the resistive load for all three axes. 
   |______________________________________________________________________
   |1   0 
   |		Negative bias configuration for X, Y and Z axes. In this configuration, a negative 
   |		current is forced across the resistive load for all three axes.. 
   |___________________________________________________________
   |1   1 
   |		This configuration is reserved. 
   |____________________________________________________________
	*/
	
	// Select Configuration register A(0x00)
	// Normal measurement configuration, data rate o/p = 0.75 Hz(0x60)
	registerWrite(bus, 0x1E, 0x00 , 0x34);
	
	//Register 0x02
	/*
	
	MR7   MR6  MR5  MR4  MR3  MR2  MR1     MR0 
    HS(0) (0)  (0)  (0)  (0)  (0)  MD1_(0) MD0_(1) 

    HS ->> 1 => Set this pin to enable High Speed I2C, 3400kHz. 


    MD1 MD0 Operating Mode 
    
	0   0 
        Continuous-Measurement Mode. In continuous-measurement mode, 
        the device continuously performs measurements and places the 
        result in the data register. RDY goes high when new data is placed 
        in all three registers. After a power-on or a write to the mode or 
        configuration register, the first measurement set is available from all 
        three data output registers after a period of 2/fDO and subsequent 
        measurements are available at a frequency of fDO, where fDO is the 
        frequency of data output. 
    
	0   1 
        Single-Measurement Mode (Default). When single-measurement 
        mode is selected, device performs a single measurement, sets RDY 
        high and returned to idle mode. Mode register returns to idle mode 
        bit values. The measurement remains in the data output register and 
    	RDY remains high until the data output register is read or another 
        measurement is performed. 
    
    1   0 
	    Idle Mode. Device is placed in idle mode. 
	    
    1   1 
	    Idle Mode. Device is placed in idle mode. 


	*/
	// Select Mode register(0x02)
	// Continuous measurement mode(0x00)
	registerWrite(bus, 0x1E, 0x02, 0x00);
		
	//sleep for set register and reboot sensor
	//sleep 1 sec
	sleep(1);

}


float hmc5883Read(int bus, char select_output)
{			
	ioctl(bus, I2C_SLAVE, 0x1E);
	// Read 6 bytes of data from register(0x03)
	// xMag msb, xMag lsb, zMag msb, zMag lsb, yMag msb, yMag lsb
	char reg[1] = {0x03};
	write(bus, reg, 1);
	char data[6] ={0};
		
	int xMag;
	int yMag;
	int zMag;
	float heading=0;
	if(read(bus, data, 6) != 6)
	{
		cout<< "Error : Input/output Error"<< endl;
	}
	else
	{
		// Convert the data
		xMag = (data[0] * 256 + data[1]);
		if(xMag > 32767)
		{
			xMag -= 65536;
		}
	
		zMag = (data[2] * 256 + data[3]);
		if(zMag > 32767) 
		{
			zMag -= 65536;
		}
	
		yMag = (data[4] * 256 + data[5]);
		if(yMag > 32767) 
		{
			yMag -= 65536;
		}
	
		// Output raw data to screen
		//for debug uncommment it
		//printf("Magnetic field in X-Axis : %d \n", xMag);
		//printf("Magnetic field in Y-Axis : %d \n", yMag);
		//printf("Magnetic field in Z-Axis : %d \n", zMag);

	}
		
	//calculate head of sensor
	heading = 180 * atan2(yMag,xMag)/3.141592;
		
	//set heading of device
	if(heading < 0)
	{
     		heading += 360;
    }
	 	
    //for select bettwin inputs
	switch(select_output) {
		//show x raw data
		case 'x' :
    		//for debug uncommment it
			//printf("Magnetic field in X-Axis : %d \n", xMag);
			return (float) xMag;
      		
		//show y raw data
		case 'y' :
    		//for debug uncommment it
			//printf("Magnetic field in Y-Axis : %d \n", yMag);
			return (float) yMag;
      		
		//show z raw data
		case 'z' :
    		//for debug uncommment it
			//printf("Magnetic field in Z-Axis : %d \n", zMag);
			return (float) zMag;
      	
	  	//show head angel
		case 'h' :
    		//for debug uncommment it
			//printf("head x-y: %f \n", heading);
			return (float)heading;
      		
    	//anything else
    	default :
      		cout<< "Enter x, y, z or h"<< endl; 	
		  
	}
	
}


//############## ADXL345 ##############

void setup_ADXL345(int bus)
{       
	// Enable measuring    
    registerWrite(bus, 0x53, 0x2D, 0x08);
     
    // Select Bandwidth rate register(0x20)
    // normal mode ,Output Data Rate = 100Hz(0x0a)
    registerWrite(bus, 0x53, 0x2C, 0x0A);
	
    // Select Power Control register (0x2d)
    // Auto Sleep disable(0x88)
    registerWrite(bus, 0x53, 0x0D, 0x08);
	 
    // Select Data  format register(0x31)
    // Self test disabled, 4-wire interface,full resolution,range +/- 16g
    registerWrite(bus, 0x53, 0x31, 0x0A);
	 	
}


double adxl345Read(int bus, char axis)
{
    
    ioctl(bus, I2C_SLAVE, 0x53);
    //reads the raw data
    char reg[1] = {0x32};
    write(bus, reg, 1);
    char data[6] = {9};
    if (read(bus,data,6)!=6) cout<< "Problems with Accelerometer Data Readings"<< endl;
	
	if (axis == 'x')			return registerRead(bus, 0x53, 0x32)/256.0;
	else if (axis =='y')		return registerRead(bus, 0x53, 0x34)/256.0;
	else if (axis =='z')		return registerRead(bus, 0x53, 0x36)/256.0;
}

//############## L3G4200D ##############

void setup_L3G4200D(int bus)
{
	// Enable x, y, z and turn off power down:
	registerWrite(bus, 0x69, 0x20, 0b00001111);

	// If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
	registerWrite(bus, 0x69, 0x21, 0b00000000);
	
	// Configure CTRL_REG3 to generate data ready interrupt on INT2
	// No interrupts used on INT1, if you'd like to configure INT1
	// or INT2 otherwise, consult the datasheet:
	registerWrite(bus, 0x69, 0x22, 0b00001000);

	// CTRL_REG4 controls the full-scale range, among other things:
	registerWrite(bus, 0x69, 0x23, 0b00110000);

	// CTRL_REG5 controls high-pass filtering of outputs, use it
	// if you'd like:
	registerWrite(bus, 0x69, 0x24, 0b00000000);
}


int l3g4200dRead(int bus, char axis)
{
	int MSB, LSB;
	
	if (axis == 'x')
	{
		MSB = registerRead(bus, 0x69, 0x29);
		LSB = registerRead(bus, 0x69, 0x28);
		return (int16_t)((MSB << 8) | LSB);
	}
	else if (axis == 'y')
	{
		MSB = registerRead(bus, 0x69, 0x2B);
		LSB = registerRead(bus, 0x69, 0x2A);
		return (int16_t)((MSB << 8) | LSB);
	}
	else if (axis == 'z')
	{
		MSB = registerRead(bus, 0x69, 0x2D);
		LSB = registerRead(bus, 0x69, 0x2C);
		return (int16_t)((MSB << 8) | LSB);
	}
}

///////// end /////////
