#ifndef QMC5883L_Compass
#define QMC5883L_Compass

#include "Arduino.h"
#include "Wire.h"


class QMC5883LCompass{
protected:

  public:
    QMC5883LCompass();
	void init();
  enum _CALIBRATION_STATUS {CALIBRATION_STATUS_UNDEFINED = -1, CALIBRATION_STATUS_LAUNCHED, CALIBRATION_STATUS_IN_PROGRESS, CALIBRATION_STATUS_COMPLETED, CALIBRATION_STATUS_MANUAL_SET};
    void setADDR(byte b);
    void setMode(byte mode, byte odr, byte rng, byte osr);
	void setSmoothing(byte steps, bool adv);
	void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);
	void launchAutoCalibration();
	void autoCalibrationTick();
    void setReset();
    void read();
	int getX();
	int getY();
	int getZ();
  int getRawX();
  int getRawY();
  int getRawZ();
  int getAzimuth();
  int getCalibrationStatus();
  int getCalibrationProgress();
  void setCalibrationStatus(_CALIBRATION_STATUS newStatus);
  int getCalibrationParam(char idx);
  private:
    void _writeReg(byte reg,byte val);
	int _get(int index);
  _CALIBRATION_STATUS _autoCalibrationStatus = CALIBRATION_STATUS_UNDEFINED;

	bool _smoothUse = false;
	byte _smoothSteps = 2;
	bool _smoothAdvanced = false;
    byte _ADDR = 0x0D;
	int _vRaw[3] = {0,0,0};
	int _vHistory[10][3];
  int _vCalOld[3][2];
  int _vHdgOld = 0;
	int _vScan = 0;
	int _vCheck=0;
	long _vTotals[3] = {0,0,0};
	int _vSmooth[3] = {0,0,0};
	void _smoothing();
	bool _calibrationUse = false;
	int _vCalibration[3][2];
	int _vCalibrated[3];
	void _applyCalibration();
};

#endif
