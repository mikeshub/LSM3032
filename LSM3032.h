#ifndef __LSM3032_h__
#define __LSM3032_h__

#include <Arduino.h> // for byte data type

// device types

#define LSM3032DLH_DEVICE   0
#define LSM3032DLM_DEVICE   1
#define LSM3032DLHC_DEVICE  2
#define LSM3032_DEVICE_AUTO 3

// SA0_A states

#define LSM3032_SA0_A_LOW  0
#define LSM3032_SA0_A_HIGH 1
#define LSM3032_SA0_A_AUTO 2

// register addresses

#define LSM3032_CTRL_REG1_A       0x20
#define LSM3032_CTRL_REG2_A       0x21
#define LSM3032_CTRL_REG3_A       0x22
#define LSM3032_CTRL_REG4_A       0x23
#define LSM3032_CTRL_REG5_A       0x24
#define LSM3032_CTRL_REG6_A       0x25 // DLHC only
#define LSM3032_HP_FILTER_RESET_A 0x25 // DLH, DLM only
#define LSM3032_REFERENCE_A       0x26
#define LSM3032_STATUS_REG_A      0x27

#define LSM3032_OUT_X_L_A         0x28
#define LSM3032_OUT_X_H_A         0x29
#define LSM3032_OUT_Y_L_A         0x2A
#define LSM3032_OUT_Y_H_A         0x2B
#define LSM3032_OUT_Z_L_A         0x2C
#define LSM3032_OUT_Z_H_A         0x2D

#define LSM3032_FIFO_CTRL_REG_A   0x2E // DLHC only
#define LSM3032_FIFO_SRC_REG_A    0x2F // DLHC only

#define LSM3032_INT1_CFG_A        0x30
#define LSM3032_INT1_SRC_A        0x31
#define LSM3032_INT1_THS_A        0x32
#define LSM3032_INT1_DURATION_A   0x33
#define LSM3032_INT2_CFG_A        0x34
#define LSM3032_INT2_SRC_A        0x35
#define LSM3032_INT2_THS_A        0x36
#define LSM3032_INT2_DURATION_A   0x37

#define LSM3032_CLICK_CFG_A       0x38 // DLHC only
#define LSM3032_CLICK_SRC_A       0x39 // DLHC only
#define LSM3032_CLICK_THS_A       0x3A // DLHC only
#define LSM3032_TIME_LIMIT_A      0x3B // DLHC only
#define LSM3032_TIME_LATENCY_A    0x3C // DLHC only
#define LSM3032_TIME_WINDOW_A     0x3D // DLHC only

#define LSM3032_CRA_REG_M         0x00
#define LSM3032_CRB_REG_M         0x01
#define LSM3032_MR_REG_M          0x02

#define LSM3032_OUT_X_H_M         0x03
#define LSM3032_OUT_X_L_M         0x04
#define LSM3032_OUT_Y_H_M         -1   // The addresses of the Y and Z magnetometer output registers
#define LSM3032_OUT_Y_L_M         -2   // are reversed on the DLM and DLHC relative to the DLH.
#define LSM3032_OUT_Z_H_M         -3   // These four defines have dummy values so the library can
#define LSM3032_OUT_Z_L_M         -4   // determine the correct address based on the device type.

#define LSM3032_SR_REG_M          0x09
#define LSM3032_IRA_REG_M         0x0A
#define LSM3032_IRB_REG_M         0x0B
#define LSM3032_IRC_REG_M         0x0C

#define LSM3032_WHO_AM_I_M        0x0F // DLM only

#define LSM3032_TEMP_OUT_H_M      0x31 // DLHC only
#define LSM3032_TEMP_OUT_L_M      0x32 // DLHC only

#define LSM3032DLH_OUT_Y_H_M      0x05
#define LSM3032DLH_OUT_Y_L_M      0x06
#define LSM3032DLH_OUT_Z_H_M      0x07
#define LSM3032DLH_OUT_Z_L_M      0x08

#define LSM3032DLM_OUT_Z_H_M      0x05
#define LSM3032DLM_OUT_Z_L_M      0x06
#define LSM3032DLM_OUT_Y_H_M      0x07
#define LSM3032DLM_OUT_Y_L_M      0x08

#define LSM3032DLHC_OUT_Z_H_M     0x05
#define LSM3032DLHC_OUT_Z_L_M     0x06
#define LSM3032DLHC_OUT_Y_H_M     0x07
#define LSM3032DLHC_OUT_Y_L_M     0x08

class LSM3032
{
	public:
		typedef struct vector
		{
			float x, y, z;
		} vector;
		uint8_t inBuffer[6];
		vector a; // accelerometer readings
		vector m; // magnetometer readings
		vector m_max; // maximum magnetometer values, used for calibration
		vector m_min; // minimum magnetometer values, used for calibration

		LSM3032(void);

		void init(byte device = LSM3032_DEVICE_AUTO, byte sa0_a = LSM3032_SA0_A_AUTO);

		void enableDefault(void);

		void writeAccReg(int reg, int value);
		byte readAccReg(int reg);
		void writeMagReg(int reg, int value);
		byte readMagReg(int reg);

		void readAcc(void);
		void readMag(void);
		void read(void);

		int heading(void);
		int heading(vector from);

		// vector functions
		static void vector_cross(const vector *a, const vector *b, vector *out);
		static float vector_dot(const vector *a,const vector *b);
		static void vector_normalize(vector *a);

	private:
		byte _device; // chip type (DLH, DLM, or DLHC)
		byte acc_address;

		byte detectSA0_A(void);
};

#endif



