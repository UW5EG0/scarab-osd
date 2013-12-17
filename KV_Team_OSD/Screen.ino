
char *ItoaPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos)  {
  uint8_t neg = 0;
  if(val < 0) {
    neg = 1;
    val = -val;
  }

  str[bytes] = 0;
  for(;;) {
    if(bytes == decimalpos) {
      str[--bytes] = DECIMAL;
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if(neg && bytes > 0)
    str[--bytes] = '-';

  while(bytes != 0)
    str[--bytes] = ' ';
  return str;
}

char *FormatGPSCoord(int32_t val, char *str, uint8_t p, char pos, char neg) {
  if(val < 0) {
    pos = neg;
    val = -val;
  }

  uint8_t bytes = p+8;

  str[bytes] = 0;
  str[--bytes] = pos;
  for(;;) {
    if(bytes == p) {
      str[--bytes] = DECIMAL;
      continue;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
//if(bytes < 3 && val == 0)
     if(bytes == 0 || (bytes < 3 && val == 0))
       break;   }

   while(bytes != 0)
     str[--bytes] = ' ';

   return str;
}

// Take time in Seconds and format it as 'MM:SS'
// Alternately Take time in Minutes and format it as 'HH:MM'
// If hhmmss is 1, display as HH:MM:SS
char *formatTime(uint32_t val, char *str, uint8_t hhmmss) {
  int8_t bytes = 5;
  if(hhmmss)
    bytes = 8;
  str[bytes] = 0;
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    str[--bytes] = '0' + (val % 6);
    val = val / 6;
    str[--bytes] = ':';
  } while(hhmmss-- != 0);
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
  } while(val != 0 && bytes != 0);

  while(bytes != 0)
     str[--bytes] = ' ';

  return str;
}

uint8_t FindNull(void)
{
  uint8_t xx;
  for(xx=0;screenBuffer[xx]!=0;xx++)
    ;
  return xx;
}

void displayTemperature(void)        // WILL WORK ONLY WITH V1.2
{
  int xxx;
  if (Settings[S_UNITSYSTEM])
    xxx = temperature*1.8+32;       //Fahrenheit conversion for imperial system.
  else
    xxx = temperature;

  if(xxx > temperMAX)
    temperMAX = xxx;

  //shiki mod
  if(!fieldIsVisible(temperaturePosition))
    return;
  //
  itoa(xxx,screenBuffer,10);
  uint8_t xx = FindNull();   // find the NULL
  screenBuffer[xx++]=temperatureUnitAdd[Settings[S_UNITSYSTEM]];
  screenBuffer[xx]=0;  // Restore the NULL
  MAX7456_WriteString(screenBuffer,getPosition(temperaturePosition));
}

void displayMode(void)
{
  // Shiki Mod
  if ((MwSensorActive&mode_osd_switch))
  return;
  //
  
    int16_t dist;
  if(Settings[S_UNITSYSTEM])
    dist = GPS_distanceToHome * 3.2808;           // mt to feet
  else
    dist = GPS_distanceToHome;                    // Mt

  if(dist > distanceMAX)
    distanceMAX = dist;
  itoa(dist, screenBuffer+3, 10);
  uint8_t xx = FindNull();
  if(Settings[S_UNITSYSTEM]==METRIC)
    screenBuffer[xx++] =SYM_M;
  else
   screenBuffer[xx++] =SYM_FT;
   screenBuffer[xx++] =0;
  
  if(Settings[S_MODEICON]){
    if(MwSensorActive&mode_gpshome){
      screenBuffer[8]=0;
      screenBuffer[0] = SYM_GHOME;
      screenBuffer[1] = SYM_GHOME1;
      screenBuffer[2] = SYM_COLON;
    }
    else if(MwSensorActive&mode_gpshold){
      screenBuffer[2]=0;
      screenBuffer[0] = SYM_GHOLD;
      screenBuffer[1] = SYM_GHOLD1;
    }
    else if(MwSensorActive&mode_stable){
      screenBuffer[2]=0;
      screenBuffer[0]=SYM_STABLE;
      screenBuffer[1]=SYM_STABLE1;
    }
   else{
      screenBuffer[2]=0;
      screenBuffer[0]=SYM_ACRO;
      screenBuffer[1]=SYM_ACRO1;
    }
    MAX7456_WriteString(screenBuffer,getPosition(sensorPosition)+LINE);
    if(MwSensorActive&mode_camstab){
      screenBuffer[2]=0;
      screenBuffer[0]=SYM_GIMBAL;
      screenBuffer[1]=SYM_GIMBAL1;  
    MAX7456_WriteString(screenBuffer,getPosition(gimbalPosition));
    }
    
    xx = 0;
  if(MwSensorActive&mode_stable){
    screenBuffer[xx] = SYM_ACC;
    xx++;
    }
  if (MwSensorActive&mode_mag){
    screenBuffer[xx] = SYM_MAG;
    xx++;
  }
  if (MwSensorActive&mode_baro){
    screenBuffer[xx] = SYM_BAR;
    xx++;
  }
  screenBuffer[xx] = 0;
  MAX7456_WriteString(screenBuffer,getPosition(GPS_numSatPosition)+4);


/*  
 alternative bundicator
    xx=0;
    if(MwSensorActive&mode_baro){
      screenBuffer[2]=0;
      screenBuffer[0]=SYM_BARO10; 
      screenBuffer[1]=SYM_BARO11;
    xx++;
    MAX7456_WriteString(screenBuffer,getPosition(sensorPosition)+LINE+xx*LINE);
    }
    if(MwSensorActive&mode_mag){
      screenBuffer[2]=0;
      screenBuffer[0]=SYM_MAG10;
      screenBuffer[1]=SYM_MAG11;
    xx++; 
    MAX7456_WriteString(screenBuffer,getPosition(sensorPosition)+LINE+xx*LINE);
    }
*/
  }
}

void displayArmed(void)
{
  if(!armed)
    MAX7456_WriteString_P(disarmed_text, getPosition(motorArmedPosition));
  else if(Blink10hz && flyTime < 8)
    MAX7456_WriteString_P(armed_text, getPosition(motorArmedPosition));
}

void displayCallsign(void)
{
  uint16_t position = getPosition(callSignPosition);
  if(Settings[S_DISPLAY_CS]){
      for(int X=0; X<10; X++) {
          screenBuffer[X] = char(Settings[S_CS0 + X]);
     }   
       screenBuffer[10] = 0;
       MAX7456_WriteString(screenBuffer, getPosition(callSignPosition)); 
  }
}
void displayHorizon(int rollAngle, int pitchAngle)
{
  if(!fieldIsVisible(horizonPosition))
    return;

  uint16_t position = getPosition(horizonPosition);

  if(pitchAngle>200) pitchAngle=200;
  if(pitchAngle<-250) pitchAngle=-250;
  if(rollAngle>400) rollAngle=400;
  if(rollAngle<-400) rollAngle=-400;

  for(int X=0; X<=8; X++) {
    int Y = (rollAngle * (4-X)) / 64;
    Y -= pitchAngle / 8;
    Y += 41;
    if(Y >= 0 && Y <= 81) {
      uint16_t pos = position + LINE*(Y/9) + 3 - 2*LINE + X;
      screen[pos] = SYM_AH_BAR9_0+(Y%9);
      if(Y>=9 && (Y%9) == 0)
        screen[pos-LINE] = SYM_AH_BAR9_9;
    }
  }
/*
// for extra bars
  for(int X=3; X<=5; X++) {
    int Y = (rollAngle * (4-X)) / 64;
    Y -= pitchAngle / 8;
    Y += 41;
    if(Y >= 0 && Y <= 81) {
      uint16_t pos = position + LINE*(Y/9) + 3 - 2*LINE + X;
      screen[pos-LINE] = SYM_AH_BAR9_0+(Y%9);
      if(Y>=9 && (Y%9) == 0)
        screen[pos-2*LINE] = SYM_AH_BAR9_9;
    }
  }
  for(int X=3; X<=5; X++) {
    int Y = (rollAngle * (4-X)) / 64;
    Y -= pitchAngle / 8;
    Y += 41;
    if(Y >= 0 && Y <= 81) {
      uint16_t pos = position + LINE*(Y/9) + 3 - 2*LINE + X;
      screen[pos+1*LINE] = SYM_AH_BAR9_0+(Y%9);
      if(Y>=9 && (Y%9) == 0)
        screen[pos] = SYM_AH_BAR9_9;
    }

  }
*/
  if(Settings[S_DISPLAY_HORIZON_BR]){
    //Draw center screen
    screen[position+2*LINE+7-1] = SYM_AH_CENTER_LINE;
    screen[position+2*LINE+7+1] = SYM_AH_CENTER_LINE_RIGHT;
    screen[position+2*LINE+7] =   SYM_AH_CENTER;
  }
  if (Settings[S_WITHDECORATION]){
    // Draw AH sides
    screen[position+2*LINE+1] =   SYM_AH_LEFT;
    screen[position+2*LINE+13] =  SYM_AH_RIGHT;
    screen[position+0*LINE] =     SYM_AH_DECORATION_LEFT;
    screen[position+1*LINE] =     SYM_AH_DECORATION_LEFT;
    screen[position+2*LINE] =     SYM_AH_DECORATION_LEFT;
    screen[position+3*LINE] =     SYM_AH_DECORATION_LEFT;
    screen[position+4*LINE] =     SYM_AH_DECORATION_LEFT;
    screen[position+0*LINE+14] =  SYM_AH_DECORATION_RIGHT;
    screen[position+1*LINE+14] =  SYM_AH_DECORATION_RIGHT;
    screen[position+2*LINE+14] =  SYM_AH_DECORATION_RIGHT;
    screen[position+3*LINE+14] =  SYM_AH_DECORATION_RIGHT;
    screen[position+4*LINE+14] = SYM_AH_DECORATION_RIGHT;
  }
}

void displayVoltage(void)
{
  if (Settings[S_VIDVOLTAGE_VBAT]){
    vidvoltage=MwVBat;
  }
  if (Settings[S_MAINVOLTAGE_VBAT]){
    voltage=MwVBat;
  }
  ItoaPadded(voltage, screenBuffer, 4, 3);
  screenBuffer[4] = SYM_VOLT;
  screenBuffer[5] = 0;
  MAX7456_WriteString(screenBuffer,getPosition(voltagePosition));

  if (Settings[S_SHOWBATLEVELEVOLUTION]){
    // For battery evolution display
    int BATTEV1 =Settings[S_BATCELLS] * 35;
    int BATTEV2 =Settings[S_BATCELLS] * 36;
    int BATTEV3 =Settings[S_BATCELLS] * 37;
    int BATTEV4 =Settings[S_BATCELLS] * 38;
    int BATTEV5 =Settings[S_BATCELLS] * 40;
    int BATTEV6 = Settings[S_BATCELLS] * 41;

    if (voltage < BATTEV1) screenBuffer[0]=SYM_BATT_EMPTY;
    else if (voltage < BATTEV2) screenBuffer[0]=SYM_BATT_1;
    else if (voltage < BATTEV3) screenBuffer[0]=SYM_BATT_2;
    else if (voltage < BATTEV4) screenBuffer[0]=SYM_BATT_3;
    else if (voltage < BATTEV5) screenBuffer[0]=SYM_BATT_4;
    else if (voltage < BATTEV6) screenBuffer[0]=SYM_BATT_5;
    else screenBuffer[0]=SYM_BATT_FULL;                              // Max charge icon
  }
  else {
    screenBuffer[0]=SYM_MAIN_BATT;
  }
  screenBuffer[1]=0;
  MAX7456_WriteString(screenBuffer,getPosition(voltagePosition)-1);

  if (Settings[S_VIDVOLTAGE]){
    ItoaPadded(vidvoltage, screenBuffer, 4, 3);
    screenBuffer[4]=SYM_VOLT;
    screenBuffer[5]=0;
    MAX7456_WriteString(screenBuffer,getPosition(vidvoltagePosition));
    screenBuffer[0]=SYM_VID_BAT;
    screenBuffer[1]=0;
    MAX7456_WriteString(screenBuffer,getPosition(vidvoltagePosition)-1);
  }
}

void displayCurrentThrottle(void)
{
  // Shiki mod
  if(!fieldIsVisible(CurrentThrottlePosition))
    return;
  //
  if (MwRcData[THROTTLESTICK] > HighT) HighT = MwRcData[THROTTLESTICK] -5;
  if (MwRcData[THROTTLESTICK] < LowT) LowT = MwRcData[THROTTLESTICK];      // Calibrate high and low throttle settings  --defaults set in GlobalVariables.h 1100-1900
  screenBuffer[0]=SYM_THR;
  screenBuffer[1]=0;
  MAX7456_WriteString(screenBuffer,getPosition(CurrentThrottlePosition));
  if(!armed) {
    screenBuffer[0]=' ';
    screenBuffer[1]=' ';
    screenBuffer[2]='-';
    screenBuffer[3]='-';
    screenBuffer[4]=0;
    MAX7456_WriteString(screenBuffer,getPosition(CurrentThrottlePosition)+2);
  }
  else
  {
 /*
 
     uint16_t position = getPosition(CurrentThrottlePosition)-2*LINE+5;
    screen[position+1*LINE] =  0x7F;
    screen[position+2*LINE] =  0x7F;

    int CurThrottle = map(MwRcData[THROTTLESTICK],LowT,HighT,0,100);

    if     (CurThrottle > 88)  screen[position+1*LINE] =  0x8F; 
    else if(CurThrottle > 80)  screen[position+1*LINE] =  0x8E;
    else if(CurThrottle > 72)  screen[position+1*LINE] =  0x8D;
    else if(CurThrottle > 66)  screen[position+1*LINE] =  0x8C;
    else if(CurThrottle > 58)  screen[position+1*LINE] =  0x8B;
    else if(CurThrottle > 50)  screen[position+1*LINE] =  0x8A;
    else if(CurThrottle > 42)  screen[position+2*LINE] =  0x8F;
    else if(CurThrottle > 34)  screen[position+2*LINE] =  0x8E;
    else if(CurThrottle > 26)  screen[position+2*LINE] =  0x8D;
    else if(CurThrottle > 18)  screen[position+2*LINE] =  0x8C;
    else if(CurThrottle > 10)  screen[position+2*LINE] =  0x8B;
    else                       screen[position+2*LINE] =  0x8A;
*/

    int CurThrottle = map(MwRcData[THROTTLESTICK],LowT,HighT,0,100);
    ItoaPadded(CurThrottle,screenBuffer,3,0);
    screenBuffer[3]='%';
    screenBuffer[4]=0;
    MAX7456_WriteString(screenBuffer,getPosition(CurrentThrottlePosition)+2);
  }
}

void displayTime(void)
{ 
  if(flyTime < 3600) {
    screenBuffer[0] = SYM_FLY_M;
    formatTime(flyTime, screenBuffer+1, 0);
  }
  else {
    screenBuffer[0] = SYM_FLY_H;
    formatTime(flyTime/60, screenBuffer+1, 0);
  }
  MAX7456_WriteString(screenBuffer,getPosition(flyTimePosition));

  if (armed) return ;
  uint16_t position = getPosition(onTimePosition);
  if(onTime < 3600) {
    screenBuffer[0] = SYM_ON_M;
    formatTime(onTime, screenBuffer+1, 0);
  }
  else {
    screenBuffer[0] = SYM_ON_H;
    formatTime(onTime/60, screenBuffer+1, 0);
  }
  MAX7456_WriteString(screenBuffer,getPosition(onTimePosition));
}

void displayDebug(void)
{
  #if defined DEBUG
    if(!(MwSensorActive&mode_llights))
  #endif
    return;
  for(int X=0; X<4; X++) {
debug[0]=MwAltitude;
//debug[1]=X;
//debug[2]=X;
//debug[3]=X;


    ItoaPadded(debug[X], screenBuffer+2,7,0);     
    screenBuffer[0] = 0x30+X;
    screenBuffer[1] = 0X3A;
    MAX7456_WriteString(screenBuffer,getPosition(debugPosition)+(X*LINE));
  }  
}

void displayAmperage(void)
{
 // Shiki Mod
 if(!fieldIsVisible(amperagePosition))
    return;
 //
  // Real Ampere is ampere / 10
  ItoaPadded(amperage, screenBuffer, 4, 3);     // 99.9 ampere max!
  screenBuffer[4] = SYM_AMP;
  screenBuffer[5] = 0;
  MAX7456_WriteString(screenBuffer,getPosition(amperagePosition));
}

void displaypMeterSum(void)
{
  //Shiki mod
  if(!fieldIsVisible(pMeterSumPosition))
    return;
  //
  if (Settings[S_ENABLEADC]){
    pMeterSum = amperagesum;
  }

 //Shiki mod - virtual current sensor

  screenBuffer[0]=SYM_MAH;

  int xx= 0;
  if (Settings[S_AMPERAGE_VIRTUAL])
    xx= amperagesum;
  else
    xx= pMeterSum / EST_PMSum;

  itoa(xx,screenBuffer+1,10);
  MAX7456_WriteString(screenBuffer,getPosition(pMeterSumPosition));
}

void displayRSSI(void)
{
  // Shiki mod
  if(!fieldIsVisible(rssiPosition))
    return;
  //
  screenBuffer[0] = SYM_RSSI;
  // Calcul et affichage du Rssi
  itoa(rssi,screenBuffer+1,10);
  uint8_t xx = FindNull();
  screenBuffer[xx++] = '%';
  screenBuffer[xx] = 0;
  MAX7456_WriteString(screenBuffer,getPosition(rssiPosition));
}

void displayHeading(void)
{
 //Shiki mod
 if(!fieldIsVisible(MwHeadingPosition))
    return;
 //
 if (Settings[S_SHOWHEADING]) {  
      int16_t heading = MwHeading;
      if (Settings[S_HEADING360]) {
        if(heading < 0)
          heading += 360;
        ItoaPadded(heading,screenBuffer,3,0);
        screenBuffer[3]=SYM_DEGREES;
        screenBuffer[4]=0;
      }
      else {
        ItoaPadded(heading,screenBuffer,4,0);
        screenBuffer[4]=SYM_DEGREES;
        screenBuffer[5]=0;
      }
      MAX7456_WriteString(screenBuffer,getPosition(MwHeadingPosition));
  }  
}

void displayHeadingGraph(void)
{
 //Shiki Mod
 if(!fieldIsVisible(MwHeadingGraphPosition))
    return;
 //
  int xx;
  xx = MwHeading * 4;
  xx = xx + 720 + 45;
  xx = xx / 90;

  uint16_t pos = getPosition(MwHeadingGraphPosition)+1;
  memcpy_P(screen+pos, headGraph+xx+1, 9);
}

void displayIntro(void)
{

  MAX7456_WriteString_P(message0, KVTeamVersionPosition);

  if (Settings[S_VIDEOSIGNALTYPE])
    MAX7456_WriteString_P(message2, KVTeamVersionPosition+30);
  else
    MAX7456_WriteString_P(message1, KVTeamVersionPosition+30);

    
 //haydent - Time Zone & DST Setting//
  MAX7456_WriteString_P(message10, KVTeamVersionPosition+30+LINE);
  
  if(abs(GPS_tz) >= 100)ItoaPadded(GPS_tz, screenBuffer, 5, 4);
  else ItoaPadded(GPS_tz, screenBuffer, 4, 3);
  if(GPS_tz >= 0)screenBuffer[0] = '+'; 
   
  MAX7456_WriteString(screenBuffer, KVTeamVersionPosition+37+LINE); 

  MAX7456_WriteString_P(message11, KVTeamVersionPosition+43+LINE);
  MAX7456_WriteString(itoa(GPS_dst,screenBuffer,10), KVTeamVersionPosition+47+LINE);
  //haydent - Time Zone & DST Setting//

  MAX7456_WriteString_P(MultiWiiLogoL1Add, KVTeamVersionPosition+120);
  MAX7456_WriteString_P(MultiWiiLogoL2Add, KVTeamVersionPosition+120+LINE);
  MAX7456_WriteString_P(MultiWiiLogoL3Add, KVTeamVersionPosition+120+LINE+LINE);

  MAX7456_WriteString_P(message5, KVTeamVersionPosition+120+LINE+LINE+LINE);
  MAX7456_WriteString(itoa(MwVersion,screenBuffer,10),KVTeamVersionPosition+131+LINE+LINE+LINE);

  MAX7456_WriteString_P(message6, KVTeamVersionPosition+120+LINE+LINE+LINE+LINE+LINE);
  MAX7456_WriteString_P(message7, KVTeamVersionPosition+125+LINE+LINE+LINE+LINE+LINE+LINE);
  MAX7456_WriteString_P(message8, KVTeamVersionPosition+125+LINE+LINE+LINE+LINE+LINE+LINE+LINE);
  
  MAX7456_WriteString_P(message9, KVTeamVersionPosition+120+LINE+LINE+LINE+LINE+LINE+LINE+LINE+LINE);
   if(Settings[S_DISPLAY_CS]){
      for(int X=0; X<10; X++) {
          screenBuffer[X] = char(Settings[S_CS0 + X]);
      }
   if (Blink2hz)
   MAX7456_WriteString(screenBuffer, KVTeamVersionPosition+130+LINE+LINE+LINE+LINE+LINE+LINE+LINE+LINE);;     // Call Sign on the beggining of the transmission (blink 2hz)  
   }
}

void displayGPSPosition(void)
{
  if(!GPS_fix)
    return;
  // Shiki Mod
  if(!fieldIsVisible(MwGPSLatPosition)|!MwSensorActive&mode_gpshome)
    return;
//     if(MwSensorActive&mode_gpshome)

  // Shiki Mod - display LAT/LON in  mode gpshome
  if(Settings[S_COORDINATES]|MwSensorActive&mode_gpshome){
    if(fieldIsVisible(MwGPSLatPosition)|MwSensorActive&mode_gpshome) {
      screenBuffer[0] = SYM_LAT;
      FormatGPSCoord(GPS_latitude,screenBuffer+1,4,'N','S');
      if(!Settings[S_GPSCOORDTOP])
        MAX7456_WriteString(screenBuffer,getPosition(MwGPSLatPosition));
      else
        MAX7456_WriteString(screenBuffer,getPosition(MwGPSLatPositionTop));  
    }

    if(fieldIsVisible(MwGPSLatPosition)|!MwSensorActive&mode_gpshome) {
      screenBuffer[0] = SYM_LON;
      FormatGPSCoord(GPS_longitude,screenBuffer+1,4,'E','W');
      if(!Settings[S_GPSCOORDTOP])
        MAX7456_WriteString(screenBuffer,getPosition(MwGPSLonPosition));
      else
        MAX7456_WriteString(screenBuffer,getPosition(MwGPSLonPositionTop));          
    }
  }
  
  if(Settings[S_GPSALTITUDE]){
   // Shiki Mod
   if(!fieldIsVisible(MwGPSAltPosition))
    return;
   //
      screenBuffer[0] = MwGPSAltPositionAdd[Settings[S_UNITSYSTEM]];
      uint16_t xx;
      if(Settings[S_UNITSYSTEM])
        xx = GPS_altitude * 3.2808; // Mt to Feet
      else
        xx = GPS_altitude;          // Mt
      itoa(xx,screenBuffer+1,10);
      MAX7456_WriteString(screenBuffer,getPosition(MwGPSAltPosition));
      }
}

void displayNumberOfSat(void)
{
  // Shiki mod
  if(!fieldIsVisible(GPS_numSatPosition))
    return;
  //
  screenBuffer[0] = SYM_SAT_L;
  screenBuffer[1] = SYM_SAT_R;
  itoa(GPS_numSat,screenBuffer+2,10);

// Shiki Mod - display Sats alway sin same place
//  if(!Settings[S_GPSCOORDTOP])
    MAX7456_WriteString(screenBuffer,getPosition(GPS_numSatPosition));
//  else 
//    MAX7456_WriteString(screenBuffer,getPosition(GPS_numSatPositionTop)); 
}


void displayGPS_speed(void)
{

  if(!GPS_fix) return;
  if(!armed) GPS_speed=0;

  int xx;
  if(!Settings[S_UNITSYSTEM])
    xx = GPS_speed * 0.036;           // From MWii cm/sec to Km/h
  else
    xx = GPS_speed * 0.02236932;      // (0.036*0.62137)  From MWii cm/sec to mph

  if(xx > speedMAX)
    speedMAX = xx;

  //Shiki Mod
  if(!fieldIsVisible(speedPosition))
    return;
  // 
  screenBuffer[0]=speedUnitAdd[Settings[S_UNITSYSTEM]];
  itoa(xx,screenBuffer+1,10);
  MAX7456_WriteString(screenBuffer,getPosition(speedPosition));
}

void displayGPS_time(void)       //local time of coord calc - haydent
{
  if(!GPS_fix) return;
 
  uint16_t milli = GPS_time % 1000;//get milli for later
  uint32_t seconds = (GPS_time / 1000) % 86400;//remove millisonds and whole days
  
  formatTime(seconds, screenBuffer, 1);
  if(screenBuffer[0] == ' ')screenBuffer[0] = '0';//put leading zero if empty space
  screenBuffer[8] = '.';//add milli indicator
  screenBuffer[9] = '0' + (milli / 100);//only show first digit of milli due to limit of gps rate
  screenBuffer[10] = 0;//equivalent of new line or end of buffer
   
  MAX7456_WriteString(screenBuffer,getPosition(GPS_timePosition));

}     //local time of coord calc - haydent

void displayAltitude(void)
{
  int16_t altitude;
  if(Settings[S_UNITSYSTEM])
    altitude = MwAltitude*0.032808;    // cm to feet
  else
    altitude = MwAltitude/100;         // cm to mt

  if(armed && allSec>5 && altitude > altitudeMAX)
    altitudeMAX = altitude;
  //Shiki Mod
  if(!fieldIsVisible(MwAltitudePosition))
    return;
  //
  screenBuffer[0]=MwAltitudeAdd[Settings[S_UNITSYSTEM]];
  itoa(altitude,screenBuffer+1,10);
  MAX7456_WriteString(screenBuffer,getPosition(MwAltitudePosition));
}

void displayClimbRate(void)
{
  //Shiki Mod
  if(!fieldIsVisible(MwClimbRatePosition))
    return;

    uint16_t position = getPosition(horizonPosition);
    screen[position+1*LINE+15] =  0x7F;
    screen[position+2*LINE+15] =  0x7F;
    screen[position+3*LINE+15] =  0x7F;

    if     (MwVario > 120)  screen[position+1*LINE+15] =  0x8F;
    else if(MwVario > 105)  screen[position+1*LINE+15] =  0x8E;
    else if(MwVario > 90)  screen[position+1*LINE+15] =  0x8D;
    else if(MwVario > 75)  screen[position+1*LINE+15] =  0x8C;
    else if(MwVario > 60)  screen[position+1*LINE+15] =  0x8B;
    else if(MwVario > 45)  screen[position+1*LINE+15] =  0x8A;
    else if(MwVario > 30)  screen[position+2*LINE+15] =  0x8F;
    else if(MwVario > 15)  screen[position+2*LINE+15] =  0x8E;
    else if(MwVario < -135)  screen[position+3*LINE+15] =  0x8A;
    else if(MwVario < -120)  screen[position+3*LINE+15] =  0x8B;
    else if(MwVario < -105)  screen[position+3*LINE+15] =  0x8C;
    else if(MwVario < -90)  screen[position+3*LINE+15] =  0x8D;
    else if(MwVario < -75)  screen[position+3*LINE+15] =  0x8E;
    else if(MwVario < -60)  screen[position+3*LINE+15] =  0x8F;
    else if(MwVario < -45)  screen[position+2*LINE+15] =  0x8A;
    else if(MwVario < -30)  screen[position+2*LINE+15] =  0x8B;
    else if(MwVario < -15)  screen[position+2*LINE+15] =  0x8C;
    else                    screen[position+2*LINE+15] =  0x8D;

}

void displayDistanceToHome(void)
{
  if(!GPS_fix)
    return;

  int16_t dist;
  if(Settings[S_UNITSYSTEM])
    dist = GPS_distanceToHome * 3.2808;           // mt to feet
  else
    dist = GPS_distanceToHome;                    // Mt

  if(dist > distanceMAX)
    distanceMAX = dist;
  //Shiki mod
  if(!fieldIsVisible(GPS_distanceToHomePosition))
    return;
  //
  screenBuffer[0] = GPS_distanceToHomeAdd[Settings[S_UNITSYSTEM]];
  itoa(dist, screenBuffer+1, 10);
  MAX7456_WriteString(screenBuffer,getPosition(GPS_distanceToHomePosition));
}

void displayAngleToHome(void)
{
  if(!GPS_fix)
      return;
// SHiki Mod
 if(!fieldIsVisible(GPS_angleToHomePosition))
    return;
 //     
  if(Settings[S_ANGLETOHOME]){
    if(GPS_distanceToHome <= 2 && Blink2hz)
      return;
  
    ItoaPadded(GPS_directionToHome,screenBuffer,3,0);
    screenBuffer[3] = SYM_DEGREES;
    screenBuffer[4] = 0;
    MAX7456_WriteString(screenBuffer,getPosition(GPS_angleToHomePosition));
    }
}

void displayDirectionToHome(void)
{
  if(!GPS_fix)
    return;
  if(GPS_distanceToHome <= 2 && Blink2hz)
    return;

  uint16_t position;
  if ((MwSensorActive&mode_osd_switch))
    position=getPosition(GPS_directionToHomePositionBottom);
  else
    position=getPosition(GPS_directionToHomePosition);
  
  int16_t d = MwHeading + 180 + 360 - GPS_directionToHome;
  d *= 4;
  d += 45;
  d = (d/90)%16;

  screenBuffer[0] = SYM_ARROW_SOUTH + d;
  screenBuffer[1] = 0;
  MAX7456_WriteString(screenBuffer,position);
}

void displayCursor(void)
{
  int cursorpos;

  if(ROW==10){
    if(COL==3) cursorpos=SAVEP+16-1;    // page
    if(COL==1) cursorpos=SAVEP-1;       // exit
    if(COL==2) cursorpos=SAVEP+6-1;     // save/exit
  }
  if(ROW<10){
    if(configPage==1){
      if (ROW==9) ROW=7;
      if (ROW==8) ROW=10;
      if(COL==1) cursorpos=(ROW+2)*30+10;
      if(COL==2) cursorpos=(ROW+2)*30+10+6;
      if(COL==3) cursorpos=(ROW+2)*30+10+6+6;
      }
    if(configPage==2){
      COL=3;
      if (ROW==7) ROW=5;
      if (ROW==6) ROW=10;
      if (ROW==9) ROW=5;
      cursorpos=(ROW+2)*30+10+6+6;
      }
    if(configPage==3){
      COL=3;
//      if (ROW==9) ROW=7;
//      if (ROW==8) ROW=10;
      cursorpos=(ROW+2)*30+10+6+6;
     
      }
    if(configPage==4){
      COL=3;
      if (ROW==2) ROW=3;
      if (ROW==9) ROW=5;
      if (ROW==6) ROW=10;
      cursorpos=(ROW+2)*30+10+6+6;
      }
      
    if(configPage==5 || configPage==6)
      {  
      COL=3;
      if (ROW==9) ROW=7;
      if (ROW==8) ROW=10;
      cursorpos=(ROW+2)*30+10+6+6;
      }

    if(configPage==7)
      {  
      COL=3;
      if (ROW==9) ROW=5;
      if (ROW==6) ROW=10;
      if (ROW==4) cursorpos=(ROW+2)*30+10+6+6-2;  // Metric/Imperial string longer            
      else cursorpos=(ROW+2)*30+10+6+6;
      }

    if(configPage==8){
      ROW=10;
      }
  }
  if(Blink10hz)
    screen[cursorpos] = SYM_CURSOR;
}


void displayConfigScreen(void)
{
  MAX7456_WriteString_P(configMsgEXT, SAVEP);    //EXIT
  if(!previousarmedstatus) {
    MAX7456_WriteString_P(configMsgSAVE, SAVEP+6);  //SaveExit
    MAX7456_WriteString_P(configMsgPGS, SAVEP+16); //<Page>
  }

  if(configPage==1)
  {
    MAX7456_WriteString_P(configMsg10, 38);
    MAX7456_WriteString_P(configMsg11, ROLLT);
    MAX7456_WriteString(itoa(P8[0],screenBuffer,10),ROLLP);
    MAX7456_WriteString(itoa(I8[0],screenBuffer,10),ROLLI);
    MAX7456_WriteString(itoa(D8[0],screenBuffer,10),ROLLD);

    MAX7456_WriteString_P(configMsg12, PITCHT);
    MAX7456_WriteString(itoa(P8[1],screenBuffer,10), PITCHP);
    MAX7456_WriteString(itoa(I8[1],screenBuffer,10), PITCHI);
    MAX7456_WriteString(itoa(D8[1],screenBuffer,10), PITCHD);

    MAX7456_WriteString_P(configMsg13, YAWT);
    MAX7456_WriteString(itoa(P8[2],screenBuffer,10),YAWP);
    MAX7456_WriteString(itoa(I8[2],screenBuffer,10),YAWI);
    MAX7456_WriteString(itoa(D8[2],screenBuffer,10),YAWD);

    MAX7456_WriteString_P(configMsg14, ALTT);
    MAX7456_WriteString(itoa(P8[3],screenBuffer,10),ALTP);
    MAX7456_WriteString(itoa(I8[3],screenBuffer,10),ALTI);
    MAX7456_WriteString(itoa(D8[3],screenBuffer,10),ALTD);

    MAX7456_WriteString_P(configMsg15, VELT);
    MAX7456_WriteString(itoa(P8[4],screenBuffer,10),VELP);
    MAX7456_WriteString(itoa(I8[4],screenBuffer,10),VELI);
    MAX7456_WriteString(itoa(D8[4],screenBuffer,10),VELD);

    MAX7456_WriteString_P(configMsg16, LEVT);
    MAX7456_WriteString(itoa(P8[7],screenBuffer,10),LEVP);
    MAX7456_WriteString(itoa(I8[7],screenBuffer,10),LEVI);
    MAX7456_WriteString(itoa(D8[7],screenBuffer,10),LEVD);

    MAX7456_WriteString_P(configMsg17, MAGT);
    MAX7456_WriteString(itoa(P8[8],screenBuffer,10),MAGP);

    MAX7456_WriteString("P",71);
    MAX7456_WriteString("I",77);
    MAX7456_WriteString("D",83);
  }

  if(configPage==2)
  {
    MAX7456_WriteString_P(configMsg20, 38);
    MAX7456_WriteString_P(configMsg21, ROLLT);
    MAX7456_WriteString(itoa(rcRate8,screenBuffer,10),ROLLD);
    MAX7456_WriteString_P(configMsg22, PITCHT);
    MAX7456_WriteString(itoa(rcExpo8,screenBuffer,10),PITCHD);
    MAX7456_WriteString_P(configMsg23, YAWT);
    MAX7456_WriteString(itoa(rollPitchRate,screenBuffer,10),YAWD);
    MAX7456_WriteString_P(configMsg24, ALTT);
    MAX7456_WriteString(itoa(yawRate,screenBuffer,10),ALTD);
    MAX7456_WriteString_P(configMsg25, VELT);
    MAX7456_WriteString(itoa(dynThrPID,screenBuffer,10),VELD);
    MAX7456_WriteString_P(configMsg26, LEVT);
    MAX7456_WriteString(itoa(cycleTime,screenBuffer,10),LEVD);
    MAX7456_WriteString_P(configMsg27, MAGT);
    MAX7456_WriteString(itoa(I2CError,screenBuffer,10),MAGD);
  }

  if(configPage==3)
  {
    MAX7456_WriteString_P(configMsg30, 35);
    
    MAX7456_WriteString_P(configMsg31, ROLLT);
    if(Settings[S_DISPLAYVOLTAGE]){
      MAX7456_WriteString_P(configMsgON, ROLLD);
    }
    else {
      MAX7456_WriteString_P(configMsgOFF, ROLLD);
    }
    
    MAX7456_WriteString_P(configMsg32, PITCHT);
    MAX7456_WriteString(itoa(Settings[S_VOLTAGEMIN],screenBuffer,10),PITCHD);
    
    MAX7456_WriteString_P(configMsg33, YAWT);
    if(Settings[S_VIDVOLTAGE]){
      MAX7456_WriteString_P(configMsgON, YAWD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, YAWD);
    }

    MAX7456_WriteString_P(configMsg34, ALTT);
    if(Settings[S_DISPLAYTEMPERATURE] ){
      MAX7456_WriteString_P(configMsgON, ALTD);
    }
    else {
      MAX7456_WriteString_P(configMsgOFF, ALTD);
    }
    
    MAX7456_WriteString_P(configMsg35, VELT);
    MAX7456_WriteString(itoa(Settings[S_TEMPERATUREMAX],screenBuffer,10),VELD);
    

    MAX7456_WriteString_P(configMsg36, LEVT);
    if(Settings[S_AMPER_HOUR]){
      MAX7456_WriteString_P(configMsgON, LEVD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, LEVD);
    }

    MAX7456_WriteString_P(configMsg37, MAGT);
    if(Settings[S_AMPERAGE]){
      MAX7456_WriteString_P(configMsgON, MAGD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, MAGD);
    }

    MAX7456_WriteString_P(configMsg38, MAGT+30);
    if(Settings[S_AMPERAGE_VIRTUAL]){
      MAX7456_WriteString_P(configMsgON, MAGD+30);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, MAGD+30);
    }
    MAX7456_WriteString_P(configMsg39, MAGT+60);
    MAX7456_WriteString(itoa(Settings[S_AMPDIVIDERRATIO],screenBuffer,10),MAGD+60);
  }

  if(configPage==4)
  {
    MAX7456_WriteString_P(configMsg40, 39);

    MAX7456_WriteString_P(configMsg41, ROLLT);
    MAX7456_WriteString(itoa(rssiADC,screenBuffer,10),ROLLD);

    MAX7456_WriteString_P(configMsg42, PITCHT);
    MAX7456_WriteString(itoa(rssi,screenBuffer,10),PITCHD);

    MAX7456_WriteString_P(configMsg43, YAWT);
    if(rssiTimer>0) MAX7456_WriteString(itoa(rssiTimer,screenBuffer,10),YAWD-5);
    MAX7456_WriteString(itoa(Settings[S_RSSIMIN],screenBuffer,10),YAWD);

    MAX7456_WriteString_P(configMsg44, ALTT);
    MAX7456_WriteString(itoa(Settings[S_RSSIMAX],screenBuffer,10),ALTD);

    MAX7456_WriteString_P(configMsg45, VELT);
    if(Settings[S_DISPLAYRSSI]){
      MAX7456_WriteString_P(configMsgON, VELD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, VELD);
    }
  }

  if(configPage==5)
  {
    MAX7456_WriteString_P(configMsg50, 37);

    MAX7456_WriteString_P(configMsg51, ROLLT);
    if(accCalibrationTimer>0)
      MAX7456_WriteString(itoa(accCalibrationTimer,screenBuffer,10),ROLLD);
    else
      MAX7456_WriteString("-",ROLLD);

    MAX7456_WriteString_P(configMsg52, PITCHT);
    MAX7456_WriteString(itoa(MwAccSmooth[0],screenBuffer,10),PITCHD);

    MAX7456_WriteString_P(configMsg53, YAWT);
    MAX7456_WriteString(itoa(MwAccSmooth[1],screenBuffer,10),YAWD);

    MAX7456_WriteString_P(configMsg54, ALTT);
    MAX7456_WriteString(itoa(MwAccSmooth[2],screenBuffer,10),ALTD);

    MAX7456_WriteString_P(configMsg55, VELT);
    if(magCalibrationTimer>0)
      MAX7456_WriteString(itoa(magCalibrationTimer,screenBuffer,10),VELD);
    else
      MAX7456_WriteString("-",VELD);

    MAX7456_WriteString_P(configMsg56, LEVT);
    MAX7456_WriteString(itoa(MwHeading,screenBuffer,10),LEVD);

    MAX7456_WriteString_P(configMsg57, MAGT);
    if(eepromWriteTimer>0)
      MAX7456_WriteString(itoa(eepromWriteTimer,screenBuffer,10),MAGD);
    else
      MAX7456_WriteString("-",MAGD);
  }

  if(configPage==6)
  {
    MAX7456_WriteString_P(configMsg60, 39);

    MAX7456_WriteString_P(configMsg61, ROLLT);
    if(Settings[S_DISPLAYGPS]){
      MAX7456_WriteString_P(configMsgON, ROLLD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, ROLLD);
    }

    MAX7456_WriteString_P(configMsg62, PITCHT);
    if(Settings[S_COORDINATES]){
      MAX7456_WriteString_P(configMsgON, PITCHD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, PITCHD);
    }

    MAX7456_WriteString_P(configMsg63, YAWT);
    if(Settings[S_GPSCOORDTOP]){
      MAX7456_WriteString_P(configMsgON, YAWD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, YAWD);
    }

        MAX7456_WriteString_P(configMsg64, ALTT);
    if(Settings[S_GPSALTITUDE]){
      MAX7456_WriteString_P(configMsgON, ALTD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, ALTD);
    }

    MAX7456_WriteString_P(configMsg65, VELT);
    if(Settings[S_ANGLETOHOME]){
      MAX7456_WriteString_P(configMsgON, VELD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, VELD);
    }
    
    MAX7456_WriteString_P(configMsg66, LEVT);
    if(Settings[S_SHOWHEADING]){
      MAX7456_WriteString_P(configMsgON, LEVD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, LEVD);
    }
    
    MAX7456_WriteString_P(configMsg67, MAGT);
    if(Settings[S_MODEICON]){
      MAX7456_WriteString_P(configMsgON, MAGD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, MAGD);
    }    
 }

  if(configPage==7)
  {
    MAX7456_WriteString_P(configMsg70, 39);

    MAX7456_WriteString_P(configMsg71, ROLLT);
    if(Settings[S_DISPLAY_CS]){
      MAX7456_WriteString_P(configMsgON, ROLLD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, ROLLD);
    }

    MAX7456_WriteString_P(configMsg72, PITCHT);
    if(Settings[S_THROTTLEPOSITION]){
      MAX7456_WriteString_P(configMsgON, PITCHD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, PITCHD);
    }
    
    MAX7456_WriteString_P(configMsg73, YAWT);
    if(Settings[S_WITHDECORATION]){
      MAX7456_WriteString_P(configMsgON, YAWD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, YAWD);      
    }
    
    MAX7456_WriteString_P(configMsg74, ALTT);
    if(Settings[S_UNITSYSTEM]==METRIC){
      MAX7456_WriteString_P(configMsg75, ALTD-2);
    }
    else {
      MAX7456_WriteString_P(configMsg76, ALTD-2);
    }

    MAX7456_WriteString_P(configMsg77, VELT);
    if(Settings[S_VIDEOSIGNALTYPE]){
      MAX7456_WriteString_P(configMsg79, VELD);
    }
    else {
      MAX7456_WriteString_P(configMsg78, VELD);
      }

    MAX7456_WriteString_P(configMsg700, LEVT);
    if(Settings[S_VREFERENCE]){
      MAX7456_WriteString_P(configMsg701, LEVD);
    }
    else {
      MAX7456_WriteString_P(configMsg702, LEVD);
      }
   }

  if(configPage==8)
  {
    int xx;
    MAX7456_WriteString_P(configMsg80, 38);

    MAX7456_WriteString_P(configMsg81, ROLLT);
    MAX7456_WriteString(itoa(trip,screenBuffer,10),ROLLD-3);

    MAX7456_WriteString_P(configMsg82, PITCHT);
    MAX7456_WriteString(itoa(distanceMAX,screenBuffer,10),PITCHD-3);

    MAX7456_WriteString_P(configMsg83, YAWT);
    MAX7456_WriteString(itoa(altitudeMAX,screenBuffer,10),YAWD-3);

    MAX7456_WriteString_P(configMsg84, ALTT);
    MAX7456_WriteString(itoa(speedMAX,screenBuffer,10),ALTD-3);

    MAX7456_WriteString_P(configMsg85, VELT);

    formatTime(flyingTime, screenBuffer, 1);
    MAX7456_WriteString(screenBuffer,VELD-4);

    MAX7456_WriteString_P(configMsg86, LEVT);
    xx= pMeterSum / EST_PMSum;
    MAX7456_WriteString(itoa(xx,screenBuffer,10),LEVD-3);

    MAX7456_WriteString_P(configMsg87, MAGT);
    MAX7456_WriteString(itoa(temperMAX,screenBuffer,10),MAGD-3);
    }
    
  displayCursor();
}
