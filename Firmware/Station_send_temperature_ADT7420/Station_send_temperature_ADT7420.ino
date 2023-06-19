#include <LowPower.h>
#include <Wire.h>
#include <DS3232RTC.h>        // https://github.com/JChristensen/DS3232RTC
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "SIM800L.h"
#include <avr/wdt.h>


#define SIM800_TX_PIN 3
#define SIM800_RX_PIN 4
#define SIM800_RST_PIN A2

SoftwareSerial* serial = new SoftwareSerial(SIM800_TX_PIN, SIM800_RX_PIN);

     
const char APN[] = "internet.proximus.be";

  time_t t;
  float temperature;
  float humidity;
  int voltage;

struct MyObject {
  time_t t;
  float temperature;
  float humidity;
  int voltage;
};

char buf[40];
int eeAddress = 10; 
int send_interval = 1;
int measurement_interval = 1;
int working_mode = 0;
char* settings_buffer;
SIM800L* sim800l;

boolean stringComplete = false;

void(* resetFunc) (void) = 0; //declare reset function @ address 0


// example can use comma or semicolon
#define CSV_DELIM ','

#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht31 = Adafruit_SHT31();


char input = 0;
long lastPeriod = -1;

 
const int moteino_led = 9;
const int wakeupPin = 2 ;
const int switch_i2c =  A1;
const int switch_microsd =  A0;
const int switch_usart = 8;
const int switch_opto1 = 5;
const int switch_opto2 = 6;
const int switch_SIM800L = A3;

volatile int woken = 0 ;
int counter_measure = 0;
byte do_measure = 0;

//const int chipSelect = 7;

void wakeUp(){
    woken = 1 ;
}


void setup() {
 
    pinMode(wakeupPin, INPUT_PULLUP) ;
    pinMode(moteino_led, OUTPUT) ;
    pinMode(switch_i2c, OUTPUT);
    pinMode(switch_microsd, OUTPUT);
    pinMode(switch_usart, OUTPUT);
    pinMode(switch_SIM800L, OUTPUT);
    pinMode(switch_opto1, OUTPUT);
    pinMode(switch_opto2, OUTPUT);
    digitalWrite(switch_i2c, 1);
    digitalWrite(switch_microsd, 0); 
    digitalWrite(switch_usart, 0);    
    digitalWrite(switch_SIM800L, 0); 
    digitalWrite(switch_opto1, 0);
    digitalWrite(switch_opto2, 0);
  
    RTC.begin();
    
    sht31.begin(0x45);
    
    Serial.begin(9600) ;
    delay(50) ;
    Serial.println(F("Setup...")) ;
    temperature = sht31.readTemperature();
    Serial.println(temperature) ;
            
// Initialize a SoftwareSerial

    serial->begin(9600);
    //Initialize SIM800L driver with an internal buffer of 200 bytes and a reception buffer of 100 bytes, debug disabled
    sim800l = new SIM800L((Stream *)serial, SIM800_RST_PIN, 200, 100);
    //Equivalent line with the debug enabled on the Serial
    //sim800l = new SIM800L((Stream *)serial, SIM800_RST_PIN, 200, 100, (Stream *)&Serial);

    setSyncProvider(RTC.get);   // the function to get the time from the RTC

    if(timeStatus() != timeSet)
        Serial.println(F("Unable to sync with the RTC"));
    else
        Serial.println(F("RTC has set the system time"));

    char buf[40];
    t =  RTC.get();
    sprintf(buf, "%02u:%02u:%02u %02u/%02u/%02u", hour(t), minute(t), second(t), day(t), month(t), year(t));
    Serial.println(buf);    
    
    delay(50) ;

    
    set_mode();
    Serial.println(send_interval);


    t =  RTC.get();
   
    // initialize the alarms to known values, 
    // clear the alarm flags, clear the alarm interrupt flags
    // ALARM_1 will trigger at 30 seconds into each minute
    // ALARM_2 will trigger every minute, at 0 seconds.
    RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 0);
    RTC.setAlarm(ALM2_MATCH_MINUTES, 0, 0, 0, 0);
      
    // clear both alarm flags
    RTC.alarm(ALARM_1); RTC.alarm(ALARM_2);
 
    // We are going to output the alarm by going low onto the 
    // SQW output, which should be wired to wakeupPin
    RTC.squareWave(SQWAVE_NONE);
 
    // Both alarms should generate an interrupt
    RTC.alarmInterrupt(ALARM_1, true);
    RTC.alarmInterrupt(ALARM_2, true);



}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

    
    Serial.println(F("Main loop"));
    digitalWrite(switch_i2c, 1);
    digitalWrite(switch_microsd, 0); 
    digitalWrite(switch_usart, 0);    
    digitalWrite(switch_SIM800L, 0); 
    digitalWrite(switch_opto1, 0);
    digitalWrite(switch_opto2, 0);   
    delay(100);
    
    RTC.begin();
    sht31.begin(0x45);

   
    // For now, let's just ignore transitions on this pin...
    detachInterrupt(digitalPinToInterrupt(wakeupPin)) ;
    // clear both alarm flags
    if (do_measure == 1) {   // don't measure on the first wakeup...
        digitalWrite(9, 1);
    
        if (counter_measure < send_interval) {
         delay(100);
             
          Serial.println(F("Measuring"));   
       
    
          measure();
          record_data();
          Serial.print(F("Measure number : "));  
          Serial.println(counter_measure); 
          counter_measure++; 

          Serial.println(F("Going to sleep"));    
        }
        else {
          digitalWrite(switch_SIM800L, 1);
    
          serial->listen();
          delay(100);
          Serial.println(F("Sending")); 
          send_data();
          eeAddress=10;
          counter_measure=0;
          Serial.println("resetting");
          delay(200);
          resetFunc();  //call reset
        }
        
        digitalWrite(9, 0);
    } else {
    
    delay(200);
    Serial.println(F("direct sleep")); 
    }
    
    do_measure = 1;
    
    RTC.alarm(ALARM_1) ;RTC.alarm(ALARM_2) ; 
    // The INT/SQW pin from the DS3231 is wired to the wakeup pin, and
    // will go low when the alarm is triggered.  We are going to trigger
    // on the falling edge of that pulse.
    attachInterrupt(digitalPinToInterrupt(wakeupPin), wakeUp, FALLING) ;
    
    digitalWrite(9, 0);
    digitalWrite(switch_i2c, 0);
    digitalWrite(switch_microsd, 0);
    digitalWrite(SDA, 0);
    digitalWrite(SCL, 0);
    digitalWrite(switch_usart, 0);    
    digitalWrite(switch_SIM800L, 0); 
    digitalWrite(switch_opto1, 0);
    digitalWrite(switch_opto2, 0);   
    digitalWrite(MOSI, 0);
    digitalWrite(SCK, 0);
    digitalWrite(3, 0);
    digitalWrite(4, 0);
    digitalWrite(11, 0);
    digitalWrite(12, 0);
    
    t =  RTC.get();
    //wakeup every 5 minute
    RTC.setAlarm(ALM1_MATCH_MINUTES, 0, (minute(t)+measurement_interval) %60 , 0, 1 );
    //wakeup every minute
   // RTC.setAlarm(ALM1_MATCH_SECONDS, 0, 0, 0, 0 );
   //wakeup every hours
    //RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 0 );       
    // wait to empty serial buffer
    delay(100);
    // Go into powerdown mode, waiting to be woken up by INT pin...
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF) ;
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void measure(){
  

  
    t =  RTC.get();

    temperature = sht31.readTemperature();
    humidity = sht31.readHumidity();

    unsigned char counter;
    float batteryVoltage;
    int adcReading;
  
    // Discard first inaccurate reading
    adcReading = analogRead(A6);
    adcReading = 0;
    // Perform averaging
    for (counter = 10; counter > 0; counter--){
      adcReading += analogRead(A6);
    }

    voltage = adcReading/10;
    

}

void record_data(){

  //Data to store.

  MyObject data_to_record={
    t,
    temperature,
    humidity,
    voltage
  };

  EEPROM.put(eeAddress, data_to_record);

  eeAddress += sizeof(data_to_record); //Move address to the next byte after float 'f'.
  delay(50) ;
}



void send_data(){

    setupModule();
    delay(2000);
    // Establish GPRS connectivity (5 trials)
    bool connected = false;
    for(uint8_t i = 0; i < 5 && !connected; i++) {
      delay(1000);
      connected = sim800l->connectGPRS();
    }
    
    // Check if connected, if not reset the module and setup the config again
    if(connected) {
      Serial.println(F("GPRS connected !"));
    } else {
      Serial.println(F("GPRS not connected correctly but..."));
//      Serial.println(F("Reset the module."));
//      digitalWrite(switch_SIM800L, 0); 
//      delay(1000);
//      digitalWrite(switch_SIM800L, 1); 
//      delay(200);
//      setupModule();
//      return;
    }
    delay(1000);
  
    Serial.println(F("Start HTTP GET..."));
    MyObject data_to_read; //Variable to store custom object read from EEPROM.
    const char* pre_url = "--------------------------------------- put here an URL";
    eeAddress =10;
    for (int i = 0; i < send_interval; i++) {
              EEPROM.get(eeAddress, data_to_read);
              char datetime[14];
              dtostrf(data_to_read.t,10,2,datetime);
              Serial.println(data_to_read.t); 
              t = data_to_read.t;
              sprintf(buf, "%02u-%02u-%02u %02u:%02u:%02u", year(t), month(t), day(t), hour(t), minute(t), second(t));
              Serial.println(buf);
              char temp[8];
              dtostrf(data_to_read.temperature,6,2,temp);
              char hum[8];
              dtostrf(data_to_read.humidity,6,2,hum);
              char volt[8];
              dtostrf(data_to_read.voltage,6,2,volt);
              char current_mode[4];
              dtostrf(working_mode,4,0,current_mode);
              char url [200];
              strcpy(url, pre_url);
              strcat(url,buf);
              strcat(url,"%22&t=");
              strcat(url,temp);
              strcat(url,"&h=");
              strcat(url,hum);
              strcat(url,"&b=");
              strcat(url,volt);
              strcat(url,"&m=");
              strcat(url,current_mode);
              Serial.println(url);
            // Do HTTP GET communication with 10s for the timeout (read)
     
              uint16_t rc = sim800l->doGet(url, 10000);
     
             if(rc == 200) {
              // Success, output the data received on the serial
              Serial.print(F("HTTP GET successful ("));
              Serial.print(sim800l->getDataSizeReceived());
              Serial.println(F(" bytes)"));
              Serial.print(F("Received : "));
              settings_buffer=(sim800l->getDataReceived());
              Serial.println(settings_buffer);
              Serial.println(settings_buffer[0]);
            
            } else {
              // Failed...
              Serial.print(F("HTTP GET error "));
              Serial.println(rc);
             
            }
    eeAddress += sizeof(data_to_read);
    }
 
    working_mode = settings_buffer[0];
    Serial.println(working_mode);
    EEPROM.put(0, working_mode);
    // Close GPRS connectivity (5 trials)
    bool disconnected = sim800l->disconnectGPRS();
    for(uint8_t i = 0; i < 5 && !connected; i++) {
      delay(1000);
      disconnected = sim800l->disconnectGPRS();
    }
    
    if(disconnected) {
      Serial.println(F("GPRS disconnected !"));
    } else {
      Serial.println(F("GPRS still connected !"));
    }
  
    // Go into low power mode
    bool lowPowerMode = sim800l->setPowerMode(MINIMUM);
    if(lowPowerMode) {
      Serial.println(F("Module in low power mode"));
    } else {
      Serial.println(F("Failed to switch module to low power mode"));
    }

}

void setupModule() {
  
  wdt_enable(WDTO_8S);
  
  // Wait until the module is ready to accept AT commands
  
  while(!sim800l->isReady()) {
    Serial.println(F("Problem to initialize AT command, retry in 1 sec"));
    delay(1000);
  }
 
  wdt_reset();
 
  Serial.println(F("Setup Complete!"));

  // Wait for the GSM signal
  uint8_t signal = sim800l->getSignal();
  while(signal <= 0) {
    delay(1000);
    signal = sim800l->getSignal();
  }
 
  wdt_reset();
 
  Serial.print(F("Signal OK (strenght: "));
  Serial.print(signal);
  Serial.println(F(")"));
  delay(1000);
  
  wdt_reset(); 
  
  // Wait for operator network registration (national or roaming network)
  NetworkRegistration network = sim800l->getRegistrationStatus();
  while(network != REGISTERED_HOME && network != REGISTERED_ROAMING) {
    delay(1000);
    network = sim800l->getRegistrationStatus();
  }
  
  wdt_reset();

  Serial.println(F("Network registration OK"));
  delay(1000);

  wdt_disable();
  
  // Setup APN for GPRS configuration
  bool success = sim800l->setupGPRS(APN);
  int i=0;
  while(!success && i<5) {
    success = sim800l->setupGPRS(APN);
    i++;
    delay(5000);
  }
  


  Serial.println(F("GPRS config OK"));
 

   
}

void set_mode()
{    
    EEPROM.get(0, working_mode);
    
    if (working_mode==48){
        send_interval = 2;
        measurement_interval = 1;
        Serial.println(F("Test mode"));
    }else if (working_mode==49){
        send_interval = 6;
        measurement_interval = 5;
        Serial.println(F("legacy mode"));
    }else if (working_mode==50){
        send_interval =6;
        measurement_interval = 30;
        Serial.println(F("normal mode"));
    }else if (working_mode==51){
        send_interval = 3;
        measurement_interval = 60;
        Serial.println(F("eco mode"));
    }else if (working_mode==52){
        send_interval = 2;
        measurement_interval = 1;
        Serial.println(F("eco mode 2"));
    }else{
        send_interval = 1;
        measurement_interval = 1;
        Serial.println(F("mode error"));
        EEPROM.put(0, 0);
    }
}
