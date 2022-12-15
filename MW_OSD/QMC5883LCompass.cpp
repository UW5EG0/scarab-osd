/*
===============================================================================================================
QMC5883LCompass.h
Library for using QMC5583L series chip boards as a compass.
Learn more at [https://github.com/mprograms/QMC5883LCompass]

Supports:

- Getting values of XYZ axis.
- Calculating Azimuth.
- Getting 16 point Azimuth bearing direction (0 - 15).
- Getting 16 point Azimuth bearing Names (N, NNE, NE, ENE, E, ESE, SE, SSE, S, SSW, SW, WSW, W, WNW, NW, NNW)
- Smoothing of XYZ readings via rolling averaging and min / max removal.
- Optional chipset modes (see below)

===============================================================================================================

v1.0 - June 13, 2019
Written by MRPrograms
Github: [https://github.com/mprograms/]

Release under the GNU General Public License v3
[https://www.gnu.org/licenses/gpl-3.0.en.html]

===============================================================================================================



FROM QST QMC5883L Datasheet [https://nettigo.pl/attachments/440]
-----------------------------------------------
 MODE CONTROL (MODE)
	Standby			0x00
	Continuous		0x01

OUTPUT DATA RATE (ODR)
	10Hz        	0x00
	50Hz        	0x04
	100Hz       	0x08
	200Hz       	0x0C

FULL SCALE (RNG)
	2G          	0x00
	8G          	0x10

OVER SAMPLE RATIO (OSR)
	512         	0x00
	256         	0x40
	128         	0x80
	64          	0xC0

*/



#include "Arduino.h"
#include "QMC5883LCompass.h"
#include <Wire.h>

QMC5883LCompass::QMC5883LCompass() {
}


/**
	INIT
	Initialize Chip - This needs to be called in the sketch setup() function.

	@since v0.1;
**/
void QMC5883LCompass::init(){
	Wire.begin();
	_writeReg(0x0B,0x01);
	setMode(0x01,0x04,0x10,0X00);
}


/**
	SET ADDRESS
	Set the I2C Address of the chip. This needs to be called in the sketch setup() function.

	@since v0.1;
**/
// Set I2C Address if different then default.
void QMC5883LCompass::setADDR(byte b){
	_ADDR = b;
}




/**
	REGISTER
	Write the register to the chip.

	@since v0.1;
**/
// Write register values to chip
void QMC5883LCompass::_writeReg(byte r, byte v){
	Wire.beginTransmission(_ADDR);
	Wire.write(r);
	Wire.write(v);
	Wire.endTransmission();
}


/**
	CHIP MODE
	Set the chip mode.

	@since v0.1;
**/
// Set chip mode
void QMC5883LCompass::setMode(byte mode, byte odr, byte rng, byte osr){
	_writeReg(0x09,mode|odr|rng|osr);
}


/**
	RESET
	Reset the chip.

	@since v0.1;
**/
// Reset the chip
void QMC5883LCompass::setReset(){
	_writeReg(0x0A,0x80);
}

// 1 = Basic 2 = Advanced
void QMC5883LCompass::setSmoothing(byte steps, bool adv){
	_smoothUse = true;
	_smoothSteps = ( steps > 10) ? 10 : steps;
	_smoothAdvanced = (adv == true) ? true : false;
}

/**
    SET CALIBRATION
	Set calibration values for more accurate readings

	@author Claus Näveke - TheNitek [https://github.com/TheNitek]

	@since v1.1.0
**/
void QMC5883LCompass::setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max){
	_calibrationUse = true;
  _autoCalibrationStatus = CALIBRATION_STATUS_MANUAL_SET;
	_vCalibration[0][0] = x_min;
	_vCalibration[0][1] = x_max;
	_vCalibration[1][0] = y_min;
	_vCalibration[1][1] = y_max;
	_vCalibration[2][0] = z_min;
	_vCalibration[2][1] = z_max;
}



/**
	READ
	Read the XYZ axis and save the values in an array.

	@since v0.1;
**/
void QMC5883LCompass::read(){
	Wire.beginTransmission(_ADDR);
	Wire.write(0x00);
	int err = Wire.endTransmission();
	if (!err) {
		Wire.requestFrom(_ADDR, (byte)6);
		_vRaw[0] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
		_vRaw[1] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
		_vRaw[2] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
		if (_autoCalibrationStatus ==CALIBRATION_STATUS_LAUNCHED)
		{
			_vCalibration[0][0] = _vRaw[0];
			_vCalibration[0][1] = _vRaw[0];
			_vCalibration[1][0] = _vRaw[1];
			_vCalibration[1][1] = _vRaw[1];
			_vCalibration[2][0] = _vRaw[2];
			_vCalibration[2][1] = _vRaw[2];
		}
		if ( _calibrationUse || _autoCalibrationStatus == CALIBRATION_STATUS_IN_PROGRESS) {
			_applyCalibration();
		}

		if ( _smoothUse ) {
			_smoothing();
		}

		//byte overflow = Wire.read() & 0x02;
		//return overflow << 2;
	}
}

/**
    APPLY CALIBRATION
	This function uses the calibration data provided via @see setCalibration() to calculate more
	accurate readings

	@author Claus Näveke - TheNitek [https://github.com/TheNitek]

	Based on this awesome article:
	https://appelsiini.net/2018/calibrate-magnetometer/

	@since v1.1.0

**/
void QMC5883LCompass::_applyCalibration(){
	int x_offset = (_vCalibration[0][0] + _vCalibration[0][1])/2;
	int y_offset = (_vCalibration[1][0] + _vCalibration[1][1])/2;
	int z_offset = (_vCalibration[2][0] + _vCalibration[2][1])/2;
	int x_avg_delta = (_vCalibration[0][1] - _vCalibration[0][0])/2;
	int y_avg_delta = (_vCalibration[1][1] - _vCalibration[1][0])/2;
	int z_avg_delta = (_vCalibration[2][1] - _vCalibration[2][0])/2;

	int avg_delta = (x_avg_delta + y_avg_delta + z_avg_delta) / 3;

	float x_scale = (float)avg_delta / x_avg_delta;
	float y_scale = (float)avg_delta / y_avg_delta;
	float z_scale = (float)avg_delta / z_avg_delta;

	_vCalibrated[0] = (_vRaw[0] - x_offset) * x_scale;
	_vCalibrated[1] = (_vRaw[1] - y_offset) * y_scale;
	_vCalibrated[2] = (_vRaw[2] - z_offset) * z_scale;
}

/**
incapsulation of
*/
int QMC5883LCompass::getCalibrationStatus(){
	return _autoCalibrationStatus;
};

void QMC5883LCompass::setCalibrationStatus(_CALIBRATION_STATUS newStatus){
	_autoCalibrationStatus = newStatus;
}


/**
 AUTO CALIBRATION

  This function makes runtime calibration for compass alter to manual hardcode
  setting of calibration parameters. For calibration issue user can make several rounds on one place;

*/
void QMC5883LCompass::launchAutoCalibration(){
	_autoCalibrationStatus = CALIBRATION_STATUS_LAUNCHED;

}
int QMC5883LCompass::getCalibrationParam(char idx){
return _vCalibration[idx>>1][idx & 0x01];
}
int QMC5883LCompass::getCalibrationProgress(){
 return	_vCheck;
}

void QMC5883LCompass::autoCalibrationTick(){
	if (_autoCalibrationStatus == CALIBRATION_STATUS_LAUNCHED &&_vCheck < 100) //100 real reads
	{
		_vCheck++;
		_vCalibration[0][0] = _vRaw[0];
		_vCalOld[0][0] = _vRaw[0];
		_vCalibration[0][1] = _vRaw[0];
		_vCalibration[1][0] = _vRaw[1];
		_vCalOld[1][0] = _vRaw[1];
		_vCalibration[1][1] = _vRaw[1];
		_vCalibration[2][0] = _vRaw[2];
		_vCalOld[2][0] = _vRaw[2];
		_vCalibration[2][1] = _vRaw[2];

	} else {

_autoCalibrationStatus = CALIBRATION_STATUS_IN_PROGRESS;
_applyCalibration();
for (int i = 0; i < 3; i++){
if (abs(_vCalOld[i][0] - _vRaw[i]) > 1 || abs(_vCalOld[i][1] - _vRaw[i]) > 1) { // motion detection
  if (abs(getAzimuth() - _vHdgOld) > 0){
    //if motion detected and vRaw[axis] too close to edge - increment to calibration constant
		_vHdgOld = getAzimuth();
    _vCheck++;
  }

  // update minmax-s
  if (_vCalibration[i][0] > _vRaw[i])
  {
    _vCalibration[i][0] = _vRaw[i];
		_vCalOld[i][0] = _vRaw[i];
    _vCheck=0;
  }
    if (_vCalibration[i][1] < _vRaw[i])
  {
    _vCalibration[i][1] = _vRaw[i];
		_vCalOld[i][1] = _vRaw[i];
    _vCheck=0;
  }
  }
} //end FOR
  if (_vCheck > 300 && _vCalibration[0][1] - _vCalibration[0][0]> 1000 && _vCalibration[1][1] - _vCalibration[1][0]> 1000) { // many checks without updates
      _autoCalibrationStatus = CALIBRATION_STATUS_COMPLETED;
      _calibrationUse = true;
  }
}
}
/**
	SMOOTH OUTPUT
	This function smooths the output for the XYZ axis. Depending on the options set in
	@see setSmoothing(), we can run multiple methods of smoothing the sensor readings.

	First we store (n) samples of sensor readings for each axis and store them in a rolling array.
	As each new sensor reading comes in we replace it with a new reading. Then we average the total
	of all (n) readings.

	Advanced Smoothing
	If you turn advanced smoothing on, we will select the min and max values from our array
	of (n) samples. We then subtract both the min and max from the total and average the total of all
	(n - 2) readings.

	NOTE: This function does several calculations and can cause your sketch to run slower.

	@since v0.3;
**/
void QMC5883LCompass::_smoothing(){
	byte max = 0;
	byte min = 0;

	if ( _vScan > _smoothSteps - 1 ) { _vScan = 0; }

	for ( int i = 0; i < 3; i++ ) {
		if ( _vTotals[i] != 0 ) {
			_vTotals[i] = _vTotals[i] - _vHistory[_vScan][i];
		}
		_vHistory[_vScan][i] = ( _calibrationUse ) ? _vCalibrated[i] : _vRaw[i];
		_vTotals[i] = _vTotals[i] + _vHistory[_vScan][i];

		if ( _smoothAdvanced ) {
			max = 0;
			for (int j = 0; j < _smoothSteps - 1; j++) {
				max = ( _vHistory[j][i] > _vHistory[max][i] ) ? j : max;
			}

			min = 0;
			for (int k = 0; k < _smoothSteps - 1; k++) {
				min = ( _vHistory[k][i] < _vHistory[min][i] ) ? k : min;
			}

			_vSmooth[i] = ( _vTotals[i] - (_vHistory[max][i] + _vHistory[min][i]) ) / (_smoothSteps - 2);
		} else {
			_vSmooth[i] = _vTotals[i]  / _smoothSteps;
		}
	}

	_vScan++;
}


/**
	GET X AXIS
	Read the X axis

	@since v0.1;
	@return int x axis
**/
int QMC5883LCompass::getX(){
	return _get(0);
}
int QMC5883LCompass::getRawX(){
	return _vRaw[0];
}

/**
	GET Y AXIS
	Read the Y axis

	@since v0.1;
	@return int y axis
**/
int QMC5883LCompass::getY(){
	return _get(1);
}
int QMC5883LCompass::getRawY(){
	return _vRaw[1];
}

/**
	GET Z AXIS
	Read the Z axis

	@since v0.1;
	@return int z axis
**/
int QMC5883LCompass::getZ(){
	return _get(2);
}
int QMC5883LCompass::getRawZ(){
	return _vRaw[2];
}
/**
	GET SENSOR AXIS READING
	Get the smoothed, calibration, or raw data from a given sensor axis

	@since v1.1.0
	@return int sensor axis value
**/
int QMC5883LCompass::_get(int i){
	if ( _smoothUse )
		return _vSmooth[i];

	if ( _calibrationUse )
		return _vCalibrated[i];

	return _vRaw[i];
}

/**
	GET AZIMUTH
	Calculate the azimuth (in degrees);

	@since v0.1;
	@return int azimuth
**/
int QMC5883LCompass::getAzimuth(){
//	int rumb =


	int a = ((int)(atan2( getY(), -getX() ) * 180.0 / PI - 270 + 720)) % 360;
	return (a < 0)?(a+360):(a);
}
