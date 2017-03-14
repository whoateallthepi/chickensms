#include <GPRS_Shield_Arduino.h>
#include <OneWire.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>

// OneWire sensors on Pin 2 
// Voltage sensors on Pin A0/14 A1/15
// Software serial on 7/8 for SIM900 Comms
//
//#define SENSOR_DEBUG
//#define DEBUG
//#define SENSOR_TEST  // allows you to check readings without having to receive sms
#define SERIAL_BAUD 19200
#define ONEWIRE_PIN 2
#define BATTERY_PIN 14
#define SOLAR_PIN   15
#define PIN_TX      7
#define PIN_RX      8
#define BAUDRATE  19200
#define SAMPLES 5 // Used for the voltage sample
#define SYNC_INTERVAL 86400 //24 hours
#define MESSAGE_LENGTH 160
#define IDENTITY "Charli\'s Chooks"
#define ALERT "+447804825454" // Maybe this should be software-set?
#define SIGNALS 5 //Number of entries in dB table
#define VOLTAGES 10 //Number of entries in voltages table 

struct signallevel { int db;
                     const char *human; };
                     
struct signallevel signaltable [SIGNALS] = {-73,"excellent",
                                            -83, "good",
                                            -93, "ok",
                                            -109, "marginal",
                                            -999, "stuffed!"};
                                          
struct voltagelevel { float voltage;
                      int percentage; };

struct voltagelevel voltagetable [VOLTAGES] = {12.73, 100,
                                               12.62, 90, 
                                               12.50, 80, 
                                               12.37, 70,
                                               12.24, 60,
                                               12.10, 50,
                                               11.96, 40,
                                               11.81, 30,
                                               11.66, 20,
                                               11.51, 10 };
                                   
                                          
char message[MESSAGE_LENGTH];
char number[16]; 
char timestamp[24];
 
void readTemperatures(float[]);
float getVoltage(int); 

float temperatures[2];
float voltage;
unsigned long last_sync; // Used to resynchronise the time...

OneWire  ds(ONEWIRE_PIN);  // Don't forget the 4.7K resistor is necessary between the sensor wire and live
GPRS gprs(PIN_TX,PIN_RX,BAUDRATE);//RX,TX,PWR,BaudRate

void setup() {
  Serial.begin(SERIAL_BAUD);
  #ifdef DEBUG
  Serial.println("...setup");
  #endif
  
  while(!gprs.init()) {
      Serial.print("init error\r\n");
      // try powering it on 
      SIM900power();
  }
  delay(3000);  
  Serial.println("GRPS initialised...");

  syncTime();
  last_sync = now();
}

void loop() {
  #ifdef DEBUG
  Serial.println("...loop");
  #endif
 
  #ifdef SENSOR_TEST
  statusMessage(message);
  #endif
  
  if (getSMS(message, timestamp, number)){
    if ((!strncmp("DATA", message, 4)) || (!strncmp("!", message, 1 ))) {
     statusMessage(message); 
     gprs.sendSMS(number,message);
    }
  }

  #ifdef DEBUG
  Serial.print("Last sync:");
  Serial.println(last_sync);
  Serial.print("Now:");
  Serial.println(now());
  #endif 
  if ((now()- last_sync) > SYNC_INTERVAL) {
    syncTime();
    last_sync = now();
  }
 
  delay(5000);
} // end main loop

void readTemperatures (float temperatures [])
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;
  int loop;
  int16_t raw; // see comment below on this type
  
  #ifdef DEBUG
  Serial.println("...readTemperatures");
  #endif
  
  temperatures[0] = 0;
  temperatures[1] = 0;
  ds.reset_search(); // just in case things aren't tidy after previous execution
  delay(250);
  
  for( loop = 0; loop < 2; loop++ ){
    // We are expecting two sensors ...
    // Maybe look at this to work better if one/both sensors fail?
    //
    if ( !ds.search(addr)) {
      Serial.println("No addresses found.");
      Serial.println();
      ds.reset_search();
      delay(250);
      return;
    }  
    #ifdef SENSOR_DEBUG
    Serial.print("ROM =");
    for( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
      Serial.println(' ');
    }
    #endif
    
    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }
    
    // the first ROM byte indicates which chip
    switch (addr[0]) {
      case 0x10:
        #ifdef SENSOR_DEBUG
        Serial.println("  Chip = DS18S20");  // or old DS1820
        #endif
        type_s = 1;
        break;
      case 0x28:
        #ifdef SENSOR_DEBUG
        Serial.println("  Chip = DS18B20");
        #endif
        type_s = 0;
        break;
      case 0x22:
        #ifdef SENSOR_DEBUG
        Serial.println("  Chip = DS1822");
        #endif
        type_s = 0;
        break;
      default:
        #ifdef SENSOR_DEBUG
        Serial.println("Device is not a DS18x20 family device.");
        #endif
        return;
    } 
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // start conversion, with parasite power on at the end
                       // I have no idea on the above ....
    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
    
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad
    
    #ifdef SENSOR_DEBUG
    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    #endif
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
      #ifdef SENSOR_DEBUG
      Serial.print(data[i], HEX);
      Serial.print(" ");
      #endif
    }
    #ifdef SENSOR_DEBUG
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();
    #endif
    
    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    
    temperatures[loop] = (float)raw / 16.0;
    #ifdef DEBUG
    Serial.print("  Temperature = ");
    Serial.print(temperatures[loop]);
    #endif
  }                  // ends the for loop on the sensors 
  #ifdef DEBUG
  Serial.println(' ');
  Serial.println("...readTemperatures ends");
  #endif
  
  return;
}

float getVoltage(int pin){
  int sum = 0; // sum of samples taken
  int reading;  
  unsigned char sample_count = 0; // current sample number
  float voltage = -1.0;            // calculated voltage 
  const float r1 = 1000000;
  const float r2 = 100000;
  const float adjust = 0.832; // Fiddle this figure with field test/meter
  const float m = adjust * ((r1+r2)/r2);
   
  #ifdef DEBUG
  Serial.println("...getVoltage");
  Serial.print("Pin:");
  Serial.println(pin);
  Serial.print("m:");
  Serial.println(m);
  #endif
  #ifdef SENSOR_DEBUG
  Serial.println("Pin Readings:"); 
  Serial.println(reading);
  #endif
    while (sample_count < SAMPLES) {
      reading = analogRead(pin);  
      #ifdef SENSOR_DEBUG
      Serial.println("Pin Readings:"); 
      Serial.println(reading);
      #endif
      sum += reading;
        sample_count++;
        delay(10);  
  }
  // calculate the voltage
  // 5.015V is the calibrated reference voltage
  voltage = ((float)sum / (float)SAMPLES * 5.015) / 1024.0;
  // send voltage for display on Serial Monitor
  // Multiply the voltage by the raio m calculated above...
  // value
  voltage = (voltage * m);
  
  //sample_count = 0;
  //sum = 0;
  return voltage;
}

void SIM900power()
// software equivalent of pressing the GSM shield "power" button
{
  digitalWrite(9, HIGH);
  delay(1000);
  digitalWrite(9, LOW);
  delay(7000);
}
void syncTime ()
{
  int hh;
  int mm;
  int ss;
  int dd;
  int mn;
  int yyyy;
  char datetime[24];
  char temp [3];
  
  #ifdef DEBUG
  Serial.println("...syncTime");
  #endif
  
  gprs.getDateTime(datetime);
  
  #ifdef DEBUG
  Serial.println("...gprs.getDateTime");
  Serial.println(datetime);
  #endif
  
  // Split up the timestamp
  strncpy(temp,(datetime+9),2);
  temp[2] = 0;
  hh = atoi(temp);
  strncpy(temp,(datetime+12),2);
  temp[2] = 0;
  mm = atoi(temp);
  strncpy(temp,(datetime+15),2);
  temp[2] = 0;
  ss = atoi(temp);
  
  strncpy(temp,(datetime+6),2);
  temp[2] = 0;
  dd = atoi(temp);
  strncpy(temp,(datetime+3),2);
  temp[2] = 0;
  mn = atoi(temp);
  strncpy(temp,(datetime),2);
  temp[2] = 0;
  yyyy = (2000 + atoi(temp)); //Beware the year 2100 bug
  
  setTime(hh,mm,ss,dd,mn,yyyy);
  /*
  #ifdef DEBUG
  Serial.print("hh: ");
  Serial.println(hh);
  Serial.print("mm: ");
  Serial.println(mm);
  Serial.print("ss: ");
  Serial.println(ss);
  Serial.print("dd: ");
  Serial.println(dd);
  Serial.print("mn: ");
  Serial.println(mn);
  Serial.print("yy: ");
  Serial.println(yyyy);
  Serial.print("Arduino timestamp: ");
  Serial.println(now());
  #endif
  */
}
  

bool getSMS (char *message, char *timestamp, char *number)
{
  int messageIndex = 0;
  
  #ifdef DEBUG
  Serial.println("...getSMS");
  #endif
  
  messageIndex = gprs.isSMSunread();
  if (messageIndex > 0) { //At least, there is one UNREAD SMS
    gprs.readSMS(messageIndex, message, MESSAGE_LENGTH, number, timestamp);           
    //In order not to full SIM Memory, is better to delete it
    gprs.deleteSMS(messageIndex);
    #ifdef DEBUG
    Serial.print("From number: ");
    Serial.println(number);  
    Serial.print("Datetime: ");
    Serial.println(timestamp);        
    Serial.print("Recieved Message: ");
    Serial.println(message);
    Serial.println("Message ends");
    #endif
    return true;
  }
  #ifdef DEBUG
  Serial.println("no messages");
  #endif
  
  return false;
}

void statusMessage (char * message)
{
  /* Handles a request for DATA from SMS message
  Responds with Inside temp, outside temp (both via onewire on pin 2, 
  battery voltage (pin A0) and solar voltage (pin A1)
  */
  float solar, battery;
  int battery_level;
  float temperatures[2]; //inside and outside
  char inside [10];
  char outside [10];
  char battery_s [10];
  char solar_s [10];
  const char * signal_message;
  int signal_level;
  
  signal_level = getSignalLevel();
  
  signal_message = getSignalMessage(signal_level);
  
  #ifdef DEBUG
  Serial.print("Signal Message:");
  Serial.println(signal_message);
  #endif
  
  solar = getVoltage(SOLAR_PIN);
  battery = getVoltage(BATTERY_PIN);
  battery_level = batteryLevel(battery);
  readTemperatures(temperatures);
  //temperatures[1] = 1.6;
  floatToString(solar,2,solar_s);
  floatToString(battery,2,battery_s);
  floatToString(temperatures[0],1,inside);
  floatToString(temperatures[1],1,outside);

  sprintf(message,"%s at %02d:%02d:%02d - Inside temp:%sC Outside:%sC Battery:%sV(%d%%) Solar:%sV Signal:%ddb(%s)", 
                    IDENTITY, hour(), minute(), second(),inside, outside, battery_s, battery_level, solar_s,
                    signal_level,signal_message);
  
  
  #ifdef DEBUG
  Serial.println("...statusMessage");
  Serial.println(message);  
  #endif
}

void floatToString (float number, int decimals, char * stringOut)
{
  int min_width = 5;
  
//  char buffer[16];

  if ((number < 10) && (number >= 0)) {
    min_width = decimals + 2;
  }
  else {
    min_width = decimals + 3;
  }
  
  dtostrf(number,min_width,decimals,stringOut);
  
  #ifdef DEBUG
  Serial.print("...floatToString in = ");
  Serial.print(number);
  Serial.print(" out = ");
  Serial.print(stringOut);  
  #endif
  
}

int batteryLevel (float voltage) {
  #ifdef DEBUG 
  Serial.println("...batteryLevel");
  #endif
  
  for (int x =0; x < VOLTAGES; x++){
    if (voltage >= voltagetable[x].voltage)
      return voltagetable[x].percentage;
  }
  return 0;
  
}
int getSignalLevel () {
  // really should do this with pointers....
  
  int signal_raw = 0;
  int signal;
 
  gprs.getSignalStrength(&signal_raw);
  
   
  if (signal_raw < 2) {
     return -999;
  } 
  else 
  {
    signal = ((signal_raw - 2) * 2 ) - 109;  
  }
  
  #ifdef DEBUG
  Serial.println(" ");
  Serial.print("Signal:");
  Serial.println(signal);
  #endif
  return signal;
}
const char * getSignalMessage (int signal_level){
  
  for (int x = 0; x < SIGNALS; x++) {
    if (signal_level >= signaltable[x].db)
      return signaltable[x].human;
  }
}
