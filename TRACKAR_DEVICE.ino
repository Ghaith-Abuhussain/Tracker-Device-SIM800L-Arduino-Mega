#include "BigNumber.h"
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SD.h>
#include <sim800.h>
#include "timestamp32bits.h"

void(* resetFunc) (void) = 0;

timestamp32bits stamp = timestamp32bits();
#define config_file_name "config.txt"
#define log_file_name "LOG.txt"
#define M_PI       3.14159265358979323846   // pi
#define DEFAULT_BUFLEN 512  

Adafruit_GPS GPS(&Serial1);

// variables for SIM800L Code
//SoftwareSerial mySerial(3, 2); //SIM800L Tx & Rx is connected to Arduino #3 & #2
int s = 0;
int k = 0;
int iRet = 0;
boolean sendCompleted = false;
boolean timeToSend = false;
boolean gprsEnabled = false;
boolean connectedToServer = false;
int connectionTimeout = 0;
char IMEI[20] = "";
char c;
boolean connectionInteruot = false;
uint8_t recv[21];
unsigned long lastRead = millis();
unsigned long timeEnd = 0, timeBegin = 0;
unsigned long duration = 0;
double averageDuration = 0.0;
uint8_t r = 0;
#define FONA_RST 4  // no need to connect this; 
Modem modem = Modem(FONA_RST);
bool defineSerial = false;
///////////////////////////

File config_file;
File log_file;
int i = 0;
char* value_2 = new char[2];
char* value_4 = new char[4];
char* value_8 = new char[8];
unsigned int low, high;
char* value_16 = new char[16];
char* value_data = new char[4096];
String str_temp = "";
BigNumber temp = "";
BigNumber byte_8 = "18446744073709551616";
BigNumber byte_7 = "72057594037927936";
BigNumber byte_6 = "281474976710656";
BigNumber byte_5 = "1099511627776";
BigNumber byte_4 = "4294967296";
BigNumber byte_3 = "16777216";
BigNumber byte_2 = "65536";
BigNumber byte_1 = "256";
BigNumber byte_0 = "1"; 
int data_timer_counter = 0;

// input output define 
  int internal_voltage_in =0;
  int internal_voltage =0;
  
  int external_voltage_in =0;
  int external_voltage =0;
  
  int car_is_working_in =0;
  int car_is_working =0;

  int fuel_sensor_in =0;
  int fuel_sensor =0;
  
  int charge_battery_in = 13;
  int charge_battery = 0;
  
  int cutoff_command_in = 7;
  int cutoff_command = 0;
  
  int cutoff_engine = 12;

// time
  int year;
  int mon;
  int mday;
  int hour;
  int min;
  int sec;
  int number;
  unsigned int timeAcc = 0;
  BigNumber timeAccBig;
  char cstr[6] = "";
  static char bufferr[32];
  static byte idx = 0;
  boolean recvb = false;
  
// GPS_Element
  float longitude;
  float latitude;
  float altitude;
  float angle;
  float satellites;
  float speed;
  float HDOP;
  float VDOP;
  float PDOP;
  boolean fixedForFirstTime = false;
  
  // GPS_Backup_Element
  float longitude_Backup;
  float latitude_Backup;
  float altitude_Backup;
  float angle_Backup;
  float satellites_Backup;
  float speed_Backup;
  float HDOP_Backup;
  float VDOP_Backup;
  float PDOP_Backup;

// IO_Elemnet
  int Event_ID;
  int Element_Count;
  int _1b_Element_Count;
  unsigned int _1b_DATA[20][2];
  int _2b_Element_Count;
  unsigned int _2b_DATA[20][2];
  int _4b_Element_Count;
  unsigned long _4b_DATA[20][2];
  int _8b_Element_Count;
  int _8b_sensor_id[10];
  BigNumber _8b_DATA[10];

// AVL_DATA
  BigNumber now = 0;
  int priority;

// DATA
  int Codec_id;
  int AVL_DATA_COUNT;

// TCP_AVL_DATA_PACKET
  unsigned int preamble;
  unsigned long AVL_DATA_LENGTH;
  unsigned long Crc;

// tracker_type
  int socket;
  unsigned char* APN = new unsigned char[20];
  unsigned char* user = new unsigned char[20];
  unsigned char* pass = new unsigned char[20];
  unsigned char* imei = new unsigned char[20];
  unsigned char* received_message = new unsigned char[DEFAULT_BUFLEN];

void to_String_X2(unsigned int v)
{
  v &= 0xff;
  memset(value_2, 0, sizeof(value_2));
  sprintf(value_2, "%02X", v);
}

void to_String_X4(unsigned int v)
{
  v &= 0xffff;
  memset(value_4, 0, sizeof(value_4));
  sprintf(value_4, "%04X", v);
}

void to_String_X8(unsigned long v)
{
  memset(value_8, 0, sizeof(value_8));
  memset(value_4, 0, sizeof(value_4));
  
  v &= 4294967295UL;
  low = (unsigned int)(v % 65536UL);
  high = (unsigned int)(v / 65536UL);
  
  sprintf(value_4, "%04X", high);
  strcat(value_8, value_4);
  
  memset(value_4, 0, sizeof(value_4));
  sprintf(value_4, "%04X", low);
  strcat(value_8, value_4);
}

void to_String_X16(BigNumber c)
{
  memset(value_16, 0, sizeof(value_16));
  
  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_8) * byte_8;
  sprintf(value_2, "%02X", (long)(c / byte_7));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_7) * byte_7;
  sprintf(value_2, "%02X", (long)(c / byte_6));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_6) * byte_6;
  sprintf(value_2, "%02X", (long)(c / byte_5));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_5) * byte_5;
  sprintf(value_2, "%02X", (long)(c / byte_4));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_4) * byte_4;
  sprintf(value_2, "%02X", (long)(c / byte_3));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_3) * byte_3;
  sprintf(value_2, "%02X", (long)(c / byte_2));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_2) * byte_2;
  sprintf(value_2, "%02X", (long)(c / byte_1));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_1) * byte_1;
  sprintf(value_2, "%02X", (long)(c / byte_0));
  strcat(value_16, value_2);
  
}
void gen_Tracker_packet()
{
  memset(value_data, 0, sizeof(value_data));
  to_String_X8(preamble);
  strcat(value_data, value_8);
  to_String_X8(AVL_DATA_LENGTH);
  strcat(value_data, value_8);
  to_String_X2(Codec_id);
  strcat(value_data, value_2);
  to_String_X2(AVL_DATA_COUNT);
  strcat(value_data, value_2);
  // start AVL data
  to_String_X16(now);
  strcat(value_data, value_16);
  to_String_X2(priority);
  strcat(value_data, value_2);
  // GPS
  to_String_X8(longitude);
  strcat(value_data, value_8);
  to_String_X8(latitude);
  strcat(value_data, value_8);
  to_String_X4(altitude);
  strcat(value_data, value_4);
  to_String_X4(angle);
  strcat(value_data, value_4);
  to_String_X2(satellites);
  strcat(value_data, value_2);
  to_String_X4((unsigned int)speed);
  strcat(value_data, value_4);
    // I/O
  to_String_X2(Event_ID);
  strcat(value_data, value_2);
  to_String_X2(Element_Count);
  strcat(value_data, value_2);
  to_String_X2(_1b_Element_Count);
  strcat(value_data, value_2);
  for (i = 0; i < _1b_Element_Count; i++)
  {
    to_String_X2(_1b_DATA[i][0]);
    strcat(value_data, value_2);
    to_String_X2(_1b_DATA[i][1]);
    strcat(value_data,value_2);
  }
  to_String_X2(_2b_Element_Count);
  strcat(value_data, value_2);
  for (i = 0; i < _2b_Element_Count; i++)
  {
    to_String_X2(_2b_DATA[i][0]);
    strcat(value_data, value_2);
    to_String_X4(_2b_DATA[i][1]);
    strcat(value_data,value_4);
  }
  to_String_X2(_4b_Element_Count);
  strcat(value_data, value_2);
  for (i = 0; i < _4b_Element_Count; i++)
  {
    to_String_X2(_4b_DATA[i][0]);
    strcat(value_data, value_2);
    to_String_X8(_4b_DATA[i][1]);
    strcat(value_data,value_8);
  }
  to_String_X2(_8b_Element_Count);
  strcat(value_data, value_2);
  for (i = 0; i < _8b_Element_Count; i++)
  {
    to_String_X2(_8b_sensor_id[i]);
    strcat(value_data, value_2);
    to_String_X16(_8b_DATA[i]);
    strcat(value_data,value_16);
  }
  // end AVL data
  to_String_X2(AVL_DATA_COUNT);
  strcat(value_data, value_2);
  to_String_X8(Crc);
  strcat(value_data, value_8);

  strcat(value_data, "\0");
}

char convertCharToHex(char ch)
{
  char returnType;
  switch(ch)
  {
    case '0':
    returnType = 0;
    break;
    case  '1' :
    returnType = 1;
    break;
    case  '2':
    returnType = 2;
    break;
    case  '3':
    returnType = 3;
    break;
    case  '4' :
    returnType = 4;
    break;
    case  '5':
    returnType = 5;
    break;
    case  '6':
    returnType = 6;
    break;
    case  '7':
    returnType = 7;
    break;
    case  '8':
    returnType = 8;
    break;
    case  '9':
    returnType = 9;
    break;
    case  'A':
    returnType = 10;
    break;
    case  'B':
    returnType = 11;
    break;
    case  'C':
    returnType = 12;
    break;
    case  'D':
    returnType = 13;
    break;
    case  'E':
    returnType = 14;
    break;
    case  'F' :
    returnType = 15;
    break;
    default:
    returnType = 0;
    break;
  }
  return returnType;
}

void stringToCharArray(char* hex, int len)
{
    s = 0;
    for (k = 0; k < len; k += 2) {
        hex[s] = (byte) (convertCharToHex(hex[k]) << 4 | convertCharToHex(hex[k+1]));
        s++;
    }
     hex[s] = 0;
}

void setIMEI()
{
  IMEI[0] = 0x00;
  IMEI[1] = 0x0F;
  IMEI[2] = 0x33;
  IMEI[3] = 0x35;
  IMEI[4] = 0x37;
  IMEI[5] = 0x39; // 0x31
  IMEI[6] = 0x35;
  IMEI[7] = 0x34;
  IMEI[8] = 0x30;
  IMEI[9] = 0x38;
  IMEI[10] = 0x39;
  IMEI[11] = 0x32;
  IMEI[12] = 0x37;
  IMEI[13] = 0x37;
  IMEI[14] = 0x31;
  IMEI[15] = 0x34;
  IMEI[16] = 0x36;
}

void sendToServer()
{
  sendCompleted = false;
  if(gprsEnabled == false)
  {
    modem.enableGPRS(true);
    gprsEnabled = true;
  }
  TIMSK5 |= (1 << OCIE1A);
  connectionInteruot = true;
  connectionTimeout = 0;
  while(modem.TCPconnect("10.9.195.5", 5220) == false);
  connectionInteruot = false;
  connectionTimeout = 0;
  TIMSK5 &= ~(1 << OCIE1A);
  Serial.println("connected");
  iRet = modem.TCPsend(IMEI, 17);
  receivData();
  if(recv[0] == 0x31 || (int)recv[0] == 1)
  {
    Serial.println("rece IMEI");
    modem.TCPsend(value_data, 155);
    receivData();
    sendCompleted = true;
    Serial.println("Send Completed");
    //Serial3.println("AT+CIPCLOSE");
  }
}

void receivData()
{
  lastRead = millis();
  while (millis() - lastRead < 200){
    while (modem.TCPavailable()){
      r = modem.TCPread(recv, 20);
      recv[r] = 0;
      lastRead = millis();
    }
  }
   //print_receivedSig();
}

void print_receivedSig()
{
  for(i = 0; i<r; i++)
  {
    if(recv[i] == 0)
    {
      recv[i] = 0x30;
    }
  }
  Serial.println((char *) recv);
}

void simRegistration()
{
  // set SIM800L
  Serial3.begin(9600);
  //Begin serial communication with Arduino and SIM800L
  Serial.println(F("Initializing modem... (May take a few seconds)"));
  if (! modem.begin(Serial3)) {
    Serial.println(F("Couldn't find modem"));
    while(1);
  }
  Serial.print(F("Checking for Cell network..."));
  while (modem.getNetworkStatus() != 1)
  {
    Serial3.println("AT+CPIN=\"0000\"");
  }
  Serial.println(F("Registered."));
  modem.setGPRSNetworkSettings(F("source.syriatel.com"));  // set APN
  /*if(gprsEnabled == false)
  {
    modem.enableGPRS(true);
    gprsEnabled = true;
    delay(500);
  }
  Serial3.println("AT+CLTS=1");
  delay(100);
  Serial3.println("AT&W");
  delay(500);*/
  setIMEI();
  delay(3000);
  data_timer_counter = 0;
}

void updateTimeViaSIM()
{
  Serial3.println("AT+CCLK?");
  memset(bufferr,0,sizeof(bufferr));
  idx = 0;
  bufferr[idx] = '+';
  idx++;
  bufferr[idx] = 'C';
  idx++;
  bufferr[idx] = 'C';
  idx++;
  bufferr[idx] = 'L';
  idx++;
  bufferr[idx] = 'K';
  idx++;
  while(Serial3.available()) 
  {
    c = Serial3.read();
    if(c == ':')
    {
      recvb = true;
    }
    if(recvb == true)
    {
      bufferr[idx] = c;
      idx++;
      //Serial.write(c);//Forward what Software Serial received to Serial Port
    }
    if(strstr(bufferr,"+08\""))
    {
      recvb = false;
    }
    
  }
  recvb = false;
  //Serial.println(buffer);
  mday, mon, year, hour, min, sec, number = sscanf(bufferr, "%*[^\"]\"%2u%*c%2u%*c%2u,%2u:%2u:%2u+", &year, &mon, &mday, &hour, &min, &sec);
  //Serial.println(number);
  //Serial.println(String(year) + "/" + String(mon) + "/" + String(mday) + " " + String(hour) + ":" + String(min) + ":" + String(sec));
}

void setup() 
{
  // put your setup code here, to run once:
  BigNumber::begin();
  Serial.begin(115200);

  simRegistration();
  
  // input output pin defined
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(charge_battery_in, INPUT);
  pinMode(cutoff_command_in, INPUT);
  pinMode(cutoff_engine, OUTPUT);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(charge_battery_in, LOW);
  digitalWrite(cutoff_command_in, LOW);
  digitalWrite(cutoff_engine, LOW);
  
  // GPS init
  GPS.begin(9600);
  while (!SD.begin(53))
  {
    Serial.println("initialization SD Card failed!");
  }
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  connectionInteruot = false;
  connectionTimeout = 0;
  // Timer 1 init
  cli();                   //Disable interrupts while setting registers       
  TCCR1A = 0;              // Make sure it is zero
  TCCR1B = (1 << WGM12);   // Configure for CTC mode (Set it; don't OR stuff into it)
  TCCR1B |= ((1 << CS10) | (1 << CS12)); // Prescaler @ 1024
  TIMSK1 = (1 << OCIE1A);  // Enable interrupt
  OCR1A = 15624;           // compare value = 1 sec (16MHz AVR)

 // Timer 5 init
  //cli();                   //Disable interrupts while setting registers       
  TCCR5A = 0;              // Make sure it is zero
  TCCR5B = (1 << WGM12);   // Configure for CTC mode (Set it; don't OR stuff into it)
  TCCR5B |= ((1 << CS10) | (1 << CS12)); // Prescaler @ 1024
  TIMSK5 = (1 << OCIE1A);  // Enable interrupt
  OCR5A = 15624;           // compare value = 1 sec (16MHz AVR)

  // Timer 3 init
  TCCR3A = 0;              // Make sure it is zero
  TCCR3B = (1 << WGM12);   // Configure for CTC mode (Set it; don't OR stuff into it)
  TCCR3B |= ((1 << CS10) | (1 << CS12)); // Prescaler @ 1024
  TIMSK3 = (1 << OCIE1A);  // Enable interrupt
  OCR3A = 15624;       // compare value = 1 sec (16MHz AVR)
  
  sei();
  
  
}

void loop() 
{
  // put your main code here, to run repeatedly:
  int i = 0;
  char c = GPS.read();
  if (GPS.newNMEAreceived()) 
  {
    //Serial.print(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  if (data_timer_counter == 1) 
  {
    TIMSK5 &= ~(1 << OCIE1A);
    connectionInteruot = true;
    TIMSK1 &= ~(1 << OCIE1A);
    delay(100);
    if(GPS.fix) // fixedForFirstTime == true
    {
      
      //Serial.println("first time fixed");
      TIMSK3 &= ~(1 << OCIE1A);
      //Serial.println("Timer running ...");
      //Serial.println("function ...");
      gen_Tracker_packet();
      stringToCharArray(value_data, 310);
      //Serial.write(value_data,310);
      timeBegin = micros();
      
      sendToServer();
      
      timeEnd = micros();
      duration = timeEnd - timeBegin;
      averageDuration = (double)duration / 1000.0;
      Serial.println(averageDuration);
      data_timer_counter = 2;
      delay(8000 - averageDuration > 0 ? 8000 - averageDuration : 0);
      TIMSK3 |= (1 << OCIE1A);
    }
    else
    {
      Serial.println("not fixed");
      data_timer_counter = 2;
    }
    TIMSK1 |= (1 << OCIE1A);
    delay(100);
    //Serial.println("end of loop");
  }
}


ISR(TIMER1_COMPA_vect) 
{
  year = GPS.year;
  mon = GPS.month;
  mday = GPS.day;
  hour = GPS.hour;
  min = GPS.minute;
  sec = GPS.seconds;
  //GPS.milliseconds;
  
  now = (unsigned int)(stamp.timestamp(year, mon, mday, hour, min, sec) / 65536);
  now *= byte_2;
  timeAcc = (unsigned int)(stamp.timestamp(year, mon, mday, hour, min, sec) % 65536);
  timeAccBig = timeAcc/256;
  timeAccBig *= byte_1;
  timeAccBig += timeAcc % 256;
  now += timeAccBig;
  now *= 1000;
  
  if (GPS.fix) 
  {
    if(fixedForFirstTime == false)
    {
      fixedForFirstTime = true;
    }
    //year = GPS.year;
    //mon = GPS.month;
    //mday = GPS.day;
    //hour = GPS.hour;
    //min = GPS.minute;
    //sec = GPS.seconds;
    //GPS.milliseconds;
    //updateTime(1);
    //now = (unsigned int)(stamp.timestamp(year, mon, mday, hour, min, sec) / 65536);
    //now *= byte_2;
    //timeAcc = (unsigned int)(stamp.timestamp(year, mon, mday, hour, min, sec) % 65536);
    //timeAccBig = timeAcc/256;
    //timeAccBig *= byte_1;
    //timeAccBig += timeAcc % 256;
    //now += timeAccBig;
    //now *= 1000;
    longitude = (GPS.longitudeDegrees) * 10000000.0f;
    latitude = (GPS.latitudeDegrees) * 10000000.0f;
    altitude = GPS.altitude;

    // constant coordinates
    //longitude = 362440000.00;
    //latitude = 334931350.00;
    //altitude = 717;
    //Serial.print("lon: "); Serial.println(String(GPS.longitudeDegrees,8));
    //Serial.print("lat: "); Serial.println(String(GPS.latitudeDegrees,8));
    angle = GPS.angle;
    satellites = GPS.satellites;
    speed = GPS.speed * 1.852f;
    HDOP = GPS.HDOP;
    VDOP = GPS.VDOP;
    PDOP = GPS.PDOP;

    longitude_Backup = longitude;
    latitude_Backup = latitude;
    altitude_Backup = altitude;
    angle_Backup = angle;
    satellites_Backup = satellites;
    speed_Backup = speed;
    HDOP_Backup = HDOP;
    VDOP_Backup = VDOP;
    PDOP_Backup = PDOP;
  }
  else
  {
    longitude = longitude_Backup;
    latitude = latitude_Backup;
    altitude = altitude_Backup;
    // constant coordinates
   //longitude = 362440000.00;
    //latitude = 334931350.00;
    //Serial.print("lon: "); Serial.println(String(GPS.longitudeDegrees,8));
    //Serial.print("lat: "); Serial.println(String(GPS.latitudeDegrees,8));
    //angle = angle_Backup;
    //satellites = satellites_Backup;
    //speed = speed_Backup;
    //HDOP = HDOP_Backup;
    //VDOP = VDOP_Backup;
    //PDOP = PDOP_Backup;
    angle = GPS.angle;
    satellites = GPS.satellites;
    speed = GPS.speed * 1.852f;
    HDOP = GPS.HDOP;
    VDOP = GPS.VDOP;
    PDOP = GPS.PDOP;
    
    //year = 21;
    //mon = 10;
    //mday = 31;
    //hour = 14;
    //min = 19;
    //sec = 30;
    //updateTimeViaSIM();
    //now = (unsigned int)(stamp.timestamp(year, mon, mday, hour, min, sec) / 65536);
    //now *= byte_2;
    //timeAcc = (unsigned int)(stamp.timestamp(year, mon, mday, hour, min, sec) % 65536);
    //timeAccBig = timeAcc/256;
    //timeAccBig *= byte_1;
    //timeAccBig += timeAcc % 256;
    //now += timeAccBig;
    //now *= 1000;
  }

  external_voltage_in = analogRead(A0);
  delay(50);
  internal_voltage_in = analogRead(A1);
  delay(50);
  car_is_working_in = analogRead(A2);
  delay(50);
  fuel_sensor_in = analogRead(A3);
  external_voltage = (int)((external_voltage_in * (5.0 / 1023.0)) * 3000);
  internal_voltage = (int)((internal_voltage_in * (5.0 / 1023.0)) * 1000);
  fuel_sensor = (int)((fuel_sensor_in * (5.0 / 1023.0)) * 16);
  charge_battery = digitalRead(charge_battery_in);
  if((int)(car_is_working_in * (5.0 / 1023.0)) > 2)
  {
    car_is_working = 1;
  }
  else
  {
    car_is_working = 0;
  }
  cutoff_command = digitalRead(cutoff_command_in);
  if(cutoff_command == 1)
  {
    digitalWrite(cutoff_engine, HIGH);
  }
  else
  {
    digitalWrite(cutoff_engine, LOW);
  }
  
  preamble = 0;
  AVL_DATA_LENGTH = 1263;
  Codec_id = 8;
  AVL_DATA_COUNT = 1;
  now;
  priority = 2;
  // io
  Event_ID = 9;
  Element_Count = 30;
  // == 1b ==
  _1b_Element_Count = 8;
  _1b_DATA[0][0] = 239;
  _1b_DATA[0][1] = car_is_working;
  _1b_DATA[1][0] = 240;
  _1b_DATA[1][1] = 0;
  _1b_DATA[2][0] = 80;
  _1b_DATA[2][1] = 5;
  _1b_DATA[3][0] = 21;
  _1b_DATA[3][1] = 0;
  _1b_DATA[4][0] = 200;
  _1b_DATA[4][1] = 0;
  _1b_DATA[5][0] = 69;
  _1b_DATA[5][1] = 1;
  _1b_DATA[6][0] = 179;
  _1b_DATA[6][1] = 0;
  _1b_DATA[7][0] = 113;
  _1b_DATA[7][1] = 89;
  // == 2b ==
  _2b_Element_Count = 14;
  _2b_DATA[0][0] = 181;
  _2b_DATA[0][1] = 1;
  _2b_DATA[1][0] = 182;
  _2b_DATA[1][1] = 1;
  _2b_DATA[2][0] = 66;
  _2b_DATA[2][1] = external_voltage;
  _2b_DATA[3][0] = 24;
  _2b_DATA[3][1] = 0;
  _2b_DATA[4][0] = 205;
  _2b_DATA[4][1] = 0;
  _2b_DATA[5][0] = 206;
  _2b_DATA[5][1] = 0;
  _2b_DATA[6][0] = 67;
  _2b_DATA[6][1] = internal_voltage;
  _2b_DATA[7][0] = 68;
  _2b_DATA[7][1] = 141;
  _2b_DATA[8][0] = 9;
  _2b_DATA[8][1] = 218;
  _2b_DATA[9][0] = 13;
  _2b_DATA[9][1] = 128;
  _2b_DATA[10][0] = 17;
  _2b_DATA[10][1] = 68;
  _2b_DATA[11][0] = 18;
  _2b_DATA[11][1] = 8;
  _2b_DATA[12][0] = 19;
  _2b_DATA[12][1] = 64578;
  _2b_DATA[13][0] = 15;
  _2b_DATA[13][1] = 0;
  // == 4b ==
  _4b_Element_Count = 5;
  _4b_DATA[0][0] = 241;
  _4b_DATA[0][1] = 0;
  _4b_DATA[1][0] = 199;
  _4b_DATA[1][1] = 0;
  _4b_DATA[2][0] = 16;
  _4b_DATA[2][1] = 1191;
  _4b_DATA[3][0] = 12;
  _4b_DATA[3][1] = 125019UL;
  _4b_DATA[4][0] = 4;
  _4b_DATA[4][1] = 0;
  // == 8b ==
  _8b_Element_Count = 3;
  _8b_sensor_id[0] = 11;
  _8b_DATA[0] = 0;
  _8b_sensor_id[1] = 238;
  _8b_DATA[1] = 0;
  _8b_sensor_id[2] = 14;
  _8b_DATA[2] = 0;
  // Crc
  Crc = 51151UL;
  //Serial.println("inside ISR");
}

ISR(TIMER3_COMPA_vect) 
{
  
  
      //Serial.println(data_timer_counter);
      if(data_timer_counter >= 3)
      {
        //sdWriting = true;
        TIMSK3 &= ~(1 << OCIE1A);
        if(GPS.fix)
        {
          
          log_file = SD.open(log_file_name, FILE_WRITE);
          //log_file.println("date: " + String(year) + "/" + String(mon) + "/" + String(mday) + " " + String(hour) + "/" + String(min) + "/" + String(sec));
          log_file.println("data: ");
          log_file.println(value_data);
          //log_file.println("-----------------------------");
          //Serial.println("external voltage = " + String(external_voltage));
          //Serial.println("internal voltage = " + String(internal_voltage));
          //Serial.println("charge battery = " + String(charge_battery));
          //Serial.println("car is working = " + String(car_is_working));
          //Serial.println("cut off command = " + String(cutoff_command));
          //Serial.println("Fuel level = " + String(fuel_sensor) + "Liter");
          //Serial.println("-----------------------------------");
          log_file.close();
          Serial.println("saved to SD card");
          //sdWriting = false;
          
        }
        TIMSK3 |= (1 << OCIE1A);
        data_timer_counter = 0;
        
      }
      else
      {
        data_timer_counter++;
      }
}

ISR(TIMER5_COMPA_vect) 
{
  //Serial.println(connectionTimeout);
  if(connectionInteruot == true)
  {
    Serial.println(connectionTimeout);
    if(connectionTimeout > 60)
    {
      resetFunc();
    }
    else
    {
      connectionTimeout++;
    }
  }
}
