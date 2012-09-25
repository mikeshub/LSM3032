#include <LSM3032.h>
//#include <Wire.h>
#include <I2C.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define MAG_ADDRESS            (0x3C >> 1)
#define ACC_ADDRESS_SA0_A_LOW  (0x30 >> 1)
#define ACC_ADDRESS_SA0_A_HIGH (0x32 >> 1)

// Constructors ////////////////////////////////////////////////////////////////

LSM3032::LSM3032(void)
{
	// These are just some values for a particular unit; it is recommended that
	// a calibration be done for your particular unit.
	m_max.x = +540; m_max.y = +500; m_max.z = 180;
	m_min.x = -520; m_min.y = -570; m_min.z = -770;

	_device = LSM3032_DEVICE_AUTO;
	acc_address = ACC_ADDRESS_SA0_A_LOW;
}

// Public Methods //////////////////////////////////////////////////////////////

void LSM3032::init(byte device, byte sa0_a)
{
	_device = device;
	switch (_device)
	{
		case LSM3032DLH_DEVICE:
		case LSM3032DLM_DEVICE:
			if (sa0_a == LSM3032_SA0_A_LOW)
				acc_address = ACC_ADDRESS_SA0_A_LOW;
			else if (sa0_a == LSM3032_SA0_A_HIGH)
				acc_address = ACC_ADDRESS_SA0_A_HIGH;
			else
				acc_address = (detectSA0_A() == LSM3032_SA0_A_HIGH) ? ACC_ADDRESS_SA0_A_HIGH : ACC_ADDRESS_SA0_A_LOW;
			break;

		case LSM3032DLHC_DEVICE:
			acc_address = ACC_ADDRESS_SA0_A_HIGH;
			break;

		default:
			// try to auto-detect device
			if (detectSA0_A() == LSM3032_SA0_A_HIGH)
			{
				// if device responds on 0011001b (SA0_A is high), assume DLHC
				acc_address = ACC_ADDRESS_SA0_A_HIGH;
				_device = LSM3032DLHC_DEVICE;
			}
			else
			{
				// otherwise, assume DLH or DLM (pulled low by default on Pololu boards); query magnetometer WHO_AM_I to differentiate these two
				acc_address = ACC_ADDRESS_SA0_A_LOW;
				_device = (readMagReg(LSM3032_WHO_AM_I_M) == 0x3C) ? LSM3032DLM_DEVICE : LSM3032DLH_DEVICE;
			}
	}
}

// Turns on the LSM3032's accelerometer and magnetometers and places them in normal
// mode.
void LSM3032::enableDefault(void)
{
	// Enable Accelerometer
	// 0x27 = 0b00100111
	// Normal power mode, all axes enabled
	writeAccReg(LSM3032_CTRL_REG1_A, 0x27);

	// Enable Magnetometer
	// 0x00 = 0b00000000
	// Continuous conversion mode
	writeMagReg(LSM3032_MR_REG_M, 0x00);
}

// Writes an accelerometer register
void LSM3032::writeAccReg(int reg, int value)
{
	/*Wire.beginTransmission(acc_address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();*/
	I2c.write((int)acc_address,reg,value);
}

// Reads an accelerometer register
byte LSM3032::readAccReg(int reg)
{
	byte value;

	/*Wire.beginTransmission(acc_address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(acc_address, (byte)1);
	value = Wire.read();
	Wire.endTransmission();*/
	I2c.read((int)acc_address,reg,1);
	value = I2c.receive();


	return value;
}

// Writes a magnetometer register
void LSM3032::writeMagReg(int reg, int value)
{
	/*Wire.beginTransmission(MAG_ADDRESS);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();*/
	I2c.write((int)MAG_ADDRESS,reg,value);
}

// Reads a magnetometer register
byte LSM3032::readMagReg(int reg)
{
	byte value;

	// if dummy register address (magnetometer Y/Z), use device type to determine actual address
	if (reg < 0)
	{
		switch (reg)
		{
			case LSM3032_OUT_Y_H_M:
				reg = (_device == LSM3032DLH_DEVICE) ? LSM3032DLH_OUT_Y_H_M : LSM3032DLM_OUT_Y_H_M;
				break;
			case LSM3032_OUT_Y_L_M:
				reg = (_device == LSM3032DLH_DEVICE) ? LSM3032DLH_OUT_Y_L_M : LSM3032DLM_OUT_Y_L_M;
				break;
			case LSM3032_OUT_Z_H_M:
				reg = (_device == LSM3032DLH_DEVICE) ? LSM3032DLH_OUT_Z_H_M : LSM3032DLM_OUT_Z_H_M;
				break;
			case LSM3032_OUT_Z_L_M:
				reg = (_device == LSM3032DLH_DEVICE) ? LSM3032DLH_OUT_Z_L_M : LSM3032DLM_OUT_Z_L_M;
				break;
		}
	}

	/*Wire.beginTransmission(MAG_ADDRESS);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(MAG_ADDRESS, 1);
	value = Wire.read();
	Wire.endTransmission();*/
	I2c.read((int)MAG_ADDRESS,reg,1);
	value = I2c.receive();

	return value;
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM3032::readAcc(void)
{
	/*Wire.beginTransmission(acc_address);
	// assert the MSB of the address to get the accelerometer
	// to do slave-transmit subaddress updating.
	Wire.write(LSM3032_OUT_X_L_A | (1 << 7));
	Wire.endTransmission();
	Wire.requestFrom(acc_address, (byte)6);

	while (Wire.available() < 6);

	byte xla = Wire.read();
	byte xha = Wire.read();
	byte yla = Wire.read();
	byte yha = Wire.read();
	byte zla = Wire.read();
	byte zha = Wire.read();*/
	I2c.read(acc_address,LSM3032_OUT_X_L_A | (1 << 7),6,inBuffer);
	/*a.x = (xha << 8 | xla) >> 4;
	a.y = (yha << 8 | yla) >> 4;
	a.z = (zha << 8 | zla) >> 4;*/
    a.x = (inBuffer[1] << 8 | inBuffer[0]) >> 4;
	a.y = (inBuffer[3] << 8 | inBuffer[2]) >> 4;
    a.z = (inBuffer[5] << 8 | inBuffer[4]) >> 4;
}

// Reads the 3 magnetometer channels and stores them in vector m
void LSM3032::readMag(void)
{
	/*Wire.beginTransmission(MAG_ADDRESS);
	Wire.write(LSM3032_OUT_X_H_M);
	Wire.endTransmission();
	Wire.requestFrom(MAG_ADDRESS, 6);

	while (Wire.available() < 6);

	byte xhm = Wire.read();
	byte xlm = Wire.read();

	byte yhm, ylm, zhm, zlm;
	*/
	if (_device == LSM3032DLH_DEVICE)
	{
		// DLH: register address for Y comes before Z
		/*yhm = Wire.read();
		ylm = Wire.read();
		zhm = Wire.read();
		zlm = Wire.read();*/
		I2c.read(MAG_ADDRESS,LSM3032_OUT_X_H_M,6,inBuffer);
		m.x = inBuffer[0] << 8 | inBuffer[1];
		m.z = inBuffer[4] << 8 | inBuffer[5];
    	m.y = inBuffer[2] << 8 | inBuffer[3];
	}
	else
	{
		// DLM, DLHC: register address for Z comes before Y
		/*zhm = Wire.read();
		zlm = Wire.read();
		yhm = Wire.read();
		ylm = Wire.read();*/
		I2c.read(MAG_ADDRESS,LSM3032_OUT_X_H_M,6,inBuffer);
		m.x = inBuffer[0] << 8 | inBuffer[1];
		m.y = inBuffer[4] << 8 | inBuffer[5];
    	m.z = inBuffer[2] << 8 | inBuffer[3];

	}
	/*
	m.x = (xhm << 8 | xlm);
	m.y = (yhm << 8 | ylm);
	m.z = (zhm << 8 | zlm);*/
	/*I2c.read(MAG_ADDRESS,LSM3032_OUT_X_H_M,6,inBuffer);
	m.x = inBuffer[0] << 8 | inBuffer[1];
	m.y = inBuffer[4] << 8 | inBuffer[5];
    m.z = inBuffer[2] << 8 | inBuffer[3];*/

}

// Reads all 6 channels of the LSM3032 and stores them in the object variables
void LSM3032::read(void)
{
	readAcc();
	readMag();
}

// Returns the number of degrees from the -Y axis that it
// is pointing.
int LSM3032::heading(void)
{
	return heading((vector){0,-1,0});
}

// Returns the number of degrees from the From vector projected into
// the horizontal plane is away from north.
//
// Description of heading algorithm:
// Shift and scale the magnetic reading based on calibration data to
// to find the North vector. Use the acceleration readings to
// determine the Down vector. The cross product of North and Down
// vectors is East. The vectors East and North form a basis for the
// horizontal plane. The From vector is projected into the horizontal
// plane and the angle between the projected vector and north is
// returned.
int LSM3032::heading(vector from)
{
    // shift and scale
    m.x = (m.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
    m.y = (m.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
    m.z = (m.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;

    vector temp_a = a;
    // normalize
    vector_normalize(&temp_a);
    //vector_normalize(&m);

    // compute E and N
    vector E;
    vector N;
    vector_cross(&m, &temp_a, &E);
    vector_normalize(&E);
    vector_cross(&temp_a, &E, &N);

    // compute heading
    int heading = round(atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI);
    if (heading < 0) heading += 360;
	return heading;
}

void LSM3032::vector_cross(const vector *a,const vector *b, vector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float LSM3032::vector_dot(const vector *a,const vector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void LSM3032::vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

byte LSM3032::detectSA0_A(void)
{
	I2c.read((int)ACC_ADDRESS_SA0_A_LOW,(int)LSM3032_CTRL_REG1_A,1);
	/*Wire.beginTransmission(ACC_ADDRESS_SA0_A_LOW);
	Wire.write(LSM3032_CTRL_REG1_A);
	Wire.endTransmission();
	Wire.requestFrom(ACC_ADDRESS_SA0_A_LOW, 1);

	if (Wire.available())
	{
		Wire.read();
		return LSM3032_SA0_A_LOW;
	}
	else
		return LSM3032_SA0_A_HIGH;*/
	if (I2c.available())
		{
			I2c.receive();
			return LSM3032_SA0_A_LOW;
		}
		else
		return LSM3032_SA0_A_HIGH;
}
