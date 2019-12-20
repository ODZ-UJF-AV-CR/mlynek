#define DEBUG
/*
* Readout data from electric field mill
*/
#define MSG_NO 10    // number of logged NMEA messages
#define GPSerror 70000 // number of cycles for waitig for GPS in case of GPS error 
#define GPSdelay 100   // number of measurements between obtaining GPS position
 
#include <SD.h>             // Tested with version 1.2.2.
#include "wiring_private.h"
#include <Wire.h>           // Tested with version 1.0.0.
#include "src/RTCx/RTCx.h"  // Modified version

#define LED  23   // PC7
#define PHASE1  21  
#define PHASE2  22
#define SS          4    // PB4

const int analogInPin = A0;  // Analog input pin 
uint16_t sensorValue = 0;    // value read from the pot
int16_t sign;
int16_t value;
String dataString = "";
uint16_t count = 0;
struct RTCx::tm tm;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(PHASE1, INPUT);
  pinMode(PHASE2, INPUT);
  pinMode(LED, OUTPUT);

  // Initiating RTC
  rtc.autoprobe();
  rtc.resetClock();
/*
  {
    // Switch off Galileo and GLONASS; UBX-CFG-GNSS (6)+4+8*7+(2)=68 configuration bytes
    const char cmd[68]={0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x20, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x53, 0x1F};
    for (int n=0;n<(68);n++) Serial1.write(cmd[n]); 
  }          
  {
    // airborne <2g; UBX-CFG-NAV5 (6)+36+(2)=44 configuration bytes
    const char cmd[44]={0xB5, 0x62 ,0x06 ,0x24 ,0x24 ,0x00 ,0xFF ,0xFF ,0x07 ,0x03 ,0x00 ,0x00 ,0x00 ,0x00 ,0x10 ,0x27 , 0x00 ,0x00 ,0x05 ,0x00 ,0xFA ,0x00 ,0xFA ,0x00 ,0x64 ,0x00 ,0x5E ,0x01 ,0x00 ,0x3C ,0x00 ,0x00 , 0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x85 ,0x2A};
    for (int n=0;n<(44);n++) Serial1.write(cmd[n]); 
  }
*/
  {
    // switch to UTC time; UBX-CFG-RATE (6)+6+(2)=14 configuration bytes
    const char cmd[14]={0xB5 ,0x62 ,0x06 ,0x08 ,0x06 ,0x00 ,0xE8 ,0x03 ,0x01 ,0x00 ,0x00 ,0x00 ,0x00 ,0x37};
    for (int n=0;n<(14);n++) Serial1.write(cmd[n]); 
  }

  dataString = "$MLYNEK";
  {
    DDRB = 0b10111110;
    PORTB = 0b00001111;  // SDcard Power ON
  
    // make sure that the default chip select pin is set to output
    // see if the card is present and can be initialized:
    if (!SD.begin(SS)) 
    {
      Serial.println("#Card failed, or not present");
      // don't do anything more:
      return;
    }
  
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
  
    // if the file is available, write to it:
    if (dataFile) 
    {
      dataFile.println(dataString);  // write to SDcard (800 ms)     
      dataFile.close();
  
      digitalWrite(LED, HIGH);  // Blink for Dasa
      Serial.println(dataString);  // print SN to terminal 
      digitalWrite(LED, LOW);          
    }  
    // if the file isn't open, pop up an error:
    else 
    {
      Serial.println("#error opening datalog.txt");
    }
    
    DDRB = 0b10011110;
    PORTB = 0b00000001;  // SDcard Power OFF          
  }    

}


void loop() 
{
  for(uint8_t nn=0; nn<GPSdelay; nn++)
  {  
    rtc.readClock(tm);
    RTCx::time_t t = RTCx::mktime(&tm);
  
    // make a string for assembling the data to log:
    dataString = "$TIME,";
  
    dataString += String(t-946684800);  // Time of measurement
    dataString += "\r\n";
  
    for(uint16_t n=0; n<1000; n++)
    {
      digitalWrite(LED, HIGH);  
      //dataString += "$M,";
      //dataString += String(count++); 
      //dataString += ",";
      while (!digitalRead(PHASE1));
      while (!digitalRead(PHASE1));
      while (!digitalRead(PHASE1));
      while (!digitalRead(PHASE1));
      while (digitalRead(PHASE1));
      while (digitalRead(PHASE1));
      while (digitalRead(PHASE1));
      while (digitalRead(PHASE1));
      if (digitalRead(PHASE2))
      {
        sign = 1;
      }
      else
      {
        sign = -1;
      }
      
      // read the analog in value:
      sensorValue = analogRead(analogInPin);
    
      value = sensorValue * sign;
      
      // print the results to the Serial Monitor:
      Serial.println(value);
      dataString += String(value); 
      dataString += "\r\n";
    
      // wait before the next loop for the analog-to-digital
      // converter to settle after the last reading:
      digitalWrite(LED, LOW);  
      delay(80);
    }
  
    // Write to SDcard
    {
      rtc.readClock(tm);
      RTCx::time_t t = RTCx::mktime(&tm);
  
      // make a string for assembling the data to log:
      dataString += "$TIME,";
  
      dataString += String(t-946684800);  // Time of measurement
      dataString += "\r\n";
  
  
      DDRB = 0b10111110;
      PORTB = 0b00001111;  // SDcard Power ON
      digitalWrite(LED, HIGH);  // Blink for Dasa
  
      // make sure that the default chip select pin is set to output
      // see if the card is present and can be initialized:
      if (!SD.begin(SS)) 
      {
        Serial.println("#Card failed, or not present");
        // don't do anything more:
        return;
      }
  
      // open the file. note that only one file can be open at a time,
      // so you have to close this one before opening another.
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
    
      // if the file is available, write to it:
      if (dataFile) 
      {
        dataFile.print(dataString);  // write to SDcard (800 ms)     
        dataFile.close();
      }  
      // if the file isn't open, pop up an error:
      else 
      {
        Serial.println("#error opening datalog.txt");
      }
      digitalWrite(LED, LOW);          
      DDRB = 0b10011110;
      PORTB = 0b00000001;  // SDcard Power OFF        
    }
  }

  // GPS ***************************
  {
    // make a string for assembling the data to log:
    String dataString = "";

    // flush serial buffer
    while (Serial1.available()) Serial1.read();

    boolean flag = false;
    char incomingByte; 
    int messages = 0;
    uint32_t nomessages = 0;

    while(true)
    {
      if (Serial1.available()) 
      {
        // read the incoming byte:
        incomingByte = Serial1.read();
        nomessages = 0;
        
        if (incomingByte == '$') {messages++;}; // Prevent endless waiting
        if (messages > 300) break; // more than 26 s

        if (flag && (incomingByte == '*')) break;
        flag = false;
        
        //!!! if (incomingByte == 'A') flag = true;   // Waiting for FIX
        flag = true;   // Waiting for FIX
      }
      else
      {
        nomessages++;  
        if (nomessages > GPSerror) break; // preventing of forever waiting
      }
    }
    
    // make a string for assembling the NMEA to log:
    dataString = "";

    flag = false;
    messages = 0;
    nomessages = 0;
    while(true)
    {
      if (Serial1.available()) 
      {
        // read the incoming byte:
        incomingByte = Serial1.read();
        nomessages = 0;
        
        if (incomingByte == '$') {flag = true; messages++;};
        if (messages > MSG_NO)
        {
          rtc.readClock(tm);
          RTCx::time_t t = RTCx::mktime(&tm);
        
          dataString += "$TIME,";
          dataString += String(t-946684800);  // RTC Time of the last GPS NMEA Message
          
          break;
        }
        
        // say what you got:
        if (flag && (messages<=MSG_NO)) dataString+=incomingByte;
      }
      else
      {
        nomessages++;  
        if (nomessages > GPSerror) break; // preventing of forever waiting
      }
    }

    {
        DDRB = 0b10111110;
        PORTB = 0b00001111;  // SDcard Power ON
        
        // make sure that the default chip select pin is set to output
        // see if the card is present and can be initialized:
        if (!SD.begin(SS)) 
        {
          Serial.println("#Card failed, or not present");
          // don't do anything more:
          return;
        }
        
        // open the file. note that only one file can be open at a time,
        // so you have to close this one before opening another.
        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        
        // if the file is available, write to it:
        if (dataFile) 
        {
          digitalWrite(LED, HIGH);  // Blink for Dasa
          dataFile.println(dataString);  // write to SDcard (800 ms)     
          digitalWrite(LED, LOW);          
          dataFile.close();
        }  
        // if the file isn't open, pop up an error:
        else 
        {
          Serial.println("#error opening datalog.txt");
        }
        
        DDRB = 0b10011110;
        PORTB = 0b00000001;  // SDcard Power OFF
    }  
#ifdef DEBUG
    Serial.println(dataString);  // print to terminal (additional 700 ms)
#endif

  }


}
