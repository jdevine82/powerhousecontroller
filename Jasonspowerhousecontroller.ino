

/*This program is the heart of the powerhouse. It measure the current shunts, battery voltages. It turns on the generator when needed and can also control the wind generator if need. It does
 * most of its work through the IIC network using INA226 ICs which are IIC connected to 75mv shunts and the 24v bus. It will also control the grid feed inverter when it needs to do so. It can 
 * also interface with a sd card via SPI to log results. The sd card logger contains the RTC as well. Also connected to the IIC is the lcd display driven with 3 buttons for user interaction. The line
 * voltage (240v) is also monistored directly by this IC to see if all is well to enable backup inverter operation. Also connected to the SPI is a modbus module for interconnection with lipo
 * bms system later.
 * This uses the ESP8266 library for rest api via slip over serial port https://github.com/tuanpmt/espduino
 * This uses the INA226 library from https://github.com/jarzebski/Arduino-INA226
 */

#include <Wire.h>
#include <INA226.h>

#include <espduino.h>
#include <mqtt.h>

//pins definitions here

# define IIC 4 // on this bus will reside the lcd screen, and 4 INA226 ics to measure various system loads.
# define IIC 5

# define SdCardSelect 7  //slave select for talking to sd card.
# define CanBusSelect 8     //select can bus module to talk to with spi.
# define ESP8266Power 10 // this is a physcial power enable pin.
# define LineMeasure A0 //used for measureing line voltage.
# define CanInt 18 //used to provide an interrupt for the can bus module.
//# define SoftwareSerialTX 7        // used to send data out 
# define GeneratorRunOutput 3     // used to run the generator if battery is low and no solar or wind coming in.
# define WindGeneratorShutdown 4    //used to shutdown the wind turbine if battery is full or remotely if needed.
# define DumpLoadOutput 9      // used to switch  on heaters etc during the day.
# define InverteroneEnable 5    // main inverter output enable
# define InvertertwoEnable 17   //used to turn on backup inverter. Will only run if main inverter is off.
# define GridFeedEnable 16   //turn on the gridfeed inverter if trying to bulk charge.
# define EnterButton 11 //this is connected to some resistors to enable reading 3 buttons.
# define UpButton 12
# define DownButton 13
# define StormTrigger 350   //number of Watts before shutdown gets triggered by controller.
# define StormTimer 60*1000*60 //1 hour


//constatns used within program
#define multiplyer 50000000/78000/1024*5








//variables
//SoftwareSerial Serial(SoftwareSerialRX, SoftwareSerialTX); // RX, TX
ESP esp(&Serial2, &Serial, ESP8266Power);   //use the Serial port connected to the usb as the debug port and serial port 1 to communicate with the esp8266
MQTT mqtt(&esp);


INA226 InaInverter;
INA226 InaSolar;
INA226 InaWind;
INA226 InaGenerator;
float SystemVoltage,InverterPower,InverterCurrent,SolarPower,SolarCurrent,WindPower,WindCurrent,GeneratorPower,GeneratorCurrent;  
int LineVoltage;
boolean WifiConnected = false;
boolean Inverter1Enable = true;
boolean Inverter2Enable =false;
boolean GridConnectEnable =false;
boolean WindShutdownEnable =false;
boolean DumpLoadEnable =false;
boolean GeneratorEnable = false;
long StormTimerStart;
boolean StormTimerEnable = false;


void setup() {
  // put your setup code here, to run once:
  pinMode(SdCardSelect,OUTPUT); // set dir
  pinMode(CanBusSelect,OUTPUT); //set dir
  pinMode(ESP8266Power,OUTPUT);
  pinMode(LineMeasure, INPUT);
//  pinMode(SoftwareSerialRX,INPUT);
 // pinMode(SoftwareSerialTX,OUTPUT);
  pinMode(WindGeneratorShutdown,OUTPUT);
  pinMode(DumpLoadOutput,OUTPUT);
  pinMode(Inverter1Enable, OUTPUT);
  pinMode(Inverter2Enable, OUTPUT);
  pinMode(GridFeedEnable, OUTPUT);
  pinMode(UpButton,INPUT);
  pinMode(DownButton,INPUT);
  pinMode(EnterButton,INPUT);


//setup INA226's
   // Default INA226 address is 0x40 all addresses grounded
  InaInverter.begin(0x40); //A0=GND,A1=GND
  InaSolar.begin(0x41);    //A0=Vs,A1=GND
  InaWind.begin(0x42);      //A0=SDA,A1=GND
  InaGenerator.begin(0x43); //A0=SCL,A1=GND
  // Configure INA226
  InaInverter.configure(INA226_AVERAGES_256, INA226_BUS_CONV_TIME_2116US, INA226_SHUNT_CONV_TIME_2116US, INA226_MODE_SHUNT_BUS_CONT);  //setup averages, conversion times and operation mode.
  InaSolar.configure(INA226_AVERAGES_256, INA226_BUS_CONV_TIME_2116US, INA226_SHUNT_CONV_TIME_2116US, INA226_MODE_SHUNT_BUS_CONT);
  InaWind.configure(INA226_AVERAGES_256, INA226_BUS_CONV_TIME_2116US, INA226_SHUNT_CONV_TIME_2116US, INA226_MODE_SHUNT_BUS_CONT);
  InaGenerator.configure(INA226_AVERAGES_256, INA226_BUS_CONV_TIME_2116US, INA226_SHUNT_CONV_TIME_2116US, INA226_MODE_SHUNT_BUS_CONT);
  InaInverter.calibrate(0.000375, 200);  //200a shunt
  InaSolar.calibrate(0.00075,100); //100a shunt
  InaWind.calibrate(0.0015,50); //50a shunt
  InaGenerator.calibrate(0.00075,100); //100shunt
  
  Serial.begin(19200);
  Serial.begin(19200);
  esp.enable();
  delay(500);
  esp.reset();
  delay(500);
  while(!esp.ready());

  Serial.println("ARDUINO: setup mqtt client");
  if(!mqtt.begin("jasonsmqtt", "admin", "password1914", 120, 1)) {  //need to edit this when server setup
    Serial.println("ARDUINO: fail to setup mqtt");
    
  }


  Serial.println("ARDUINO: setup mqtt lwt");
  mqtt.lwt("/lwt", "offline", 0, 0); //or mqtt.lwt("/lwt", "offline");

/*setup mqtt events */
  mqtt.connectedCb.attach(&mqttConnected);
  mqtt.disconnectedCb.attach(&mqttDisconnected);
  mqtt.publishedCb.attach(&mqttPublished);
  mqtt.dataCb.attach(&mqttData);

  /*setup wifi*/
  Serial.println("ARDUINO: setup wifi");
  esp.wifiCb.attach(&wifiCb);

  esp.wifiConnect("jasonswifi","password1914");


  Serial.println("ARDUINO: system started");


}

void loop() {


  //This section we will Read the values in.
SystemVoltage=InaInverter.readBusVoltage();
InverterPower=InaInverter.readBusPower();
InverterCurrent=InaInverter.readShuntCurrent();
SolarPower=InaSolar.readBusPower();
SolarCurrent=InaSolar.readShuntCurrent();
WindPower=InaWind.readBusPower();
WindCurrent=InaWind.readShuntCurrent();
GeneratorPower=InaGenerator.readBusPower();
GeneratorCurrent=InaGenerator.readShuntCurrent();
LineVoltage=analogRead(LineMeasure)*multiplyer;  //this ends up being a integer. uses hardware filtering of rectified ac so only rough..




  //This section we will Read mqtt Serial etc commands
  esp.process();  //mqtt commands will be processed on interrupt vector.



 //This section we will run some logics
//Wind
//firstly run a logic block that detects is the wind generator is generating more than some trigger point amount. if it is shutdown for 1 hour
if (WindPower>StormTrigger) {StormTimerEnable=1; StormTimerStart=millis();}
if (StormTimerEnable){if ((StormTimerStart+StormTimer)<millis()) StormTimerEnable=0; //end Storm event
                      }// End StormTimerEnable Block




 
  //This section we will setoutpus and send mqtt,
 char SolarCurrentmqtt[10]; 
 char WindCurrentmqtt[10];
 char GeneratorCurrentmqtt[10];
 char InverterCurrentmqtt[10];
 char BatteryVoltagemqtt[10];
 char SolarPowermqtt[10];
 char WindPowermqtt[10];
 char GeneratorPowermqtt[10];
 char InverterPowermqtt[10];
 char MainsVoltagemqtt[10];

 dtostrf(SolarCurrent, 4, 2, SolarCurrentmqtt);
 dtostrf(WindCurrent, 4, 2, WindCurrentmqtt);
 dtostrf(GeneratorCurrent, 4, 2, GeneratorCurrentmqtt);
 dtostrf(InverterCurrent, 4, 2, InverterCurrentmqtt);
 dtostrf(SystemVoltage, 4, 2, BatteryVoltagemqtt);
 dtostrf(SolarPower, 4, 2, SolarPowermqtt);
 dtostrf(WindPower, 4, 2, WindPowermqtt);
 dtostrf(GeneratorPower, 4, 2, GeneratorPowermqtt);
 dtostrf(InverterPower, 4, 2, InverterPowermqtt);
 dtostrf(LineVoltage,3, 0, MainsVoltagemqtt);
 char DumpLoadEnablemqtt[]="ON ";
 char WindShutdownmqtt[]="ON ";
if (DumpLoadEnable>0) char DumpLoadEnablemqtt[]="ON "; else char DumpLoadEnablemqtt[]="OFF";
if (StormTimerEnable==1) char WindShutdownmqtt[]="ON "; else char WindShutdownmqtt[]="OFF";
  mqtt.publish("/powerhouse/SolarCurrent", SolarCurrentmqtt);

  mqtt.publish("/powerhouse/WindCurrent", WindCurrentmqtt);
  mqtt.publish("/powerhouse/GeneratorCurrent", GeneratorCurrentmqtt);
  mqtt.publish("/powerhouse/InverterCurrent",InverterCurrentmqtt);
  mqtt.publish("/powerhouse/BatteryVoltage", BatteryVoltagemqtt);
  mqtt.publish("/powerhouse/SolarPower", SolarPowermqtt);
  mqtt.publish("/powerhouse/WindPower", WindPowermqtt);
  mqtt.publish("/powerhouse/GeneratorPower", GeneratorPowermqtt);
  mqtt.publish("/powerhouse/InverterPower", InverterPowermqtt);
  mqtt.publish("/powerhouse/MainsVoltage", MainsVoltagemqtt);
  mqtt.publish("/powerhouse/DumpLoadEnable", DumpLoadEnablemqtt);
  mqtt.publish("/powerhouse/WindShutdown", WindShutdownmqtt);


   // Convert data to JSON string 
 String json =
 "{\"data\":{"
 "\"SolarCurrent\": \"" + String(SolarCurrent) + "\","
 "\"WindCurrent\": \"" + String(WindCurrent) + "\","
 "\"GeneratorCurrent\": \"" + String(GeneratorCurrent) + "\","
 "\"InverterCurrent\": \"" + String(InverterCurrent) + "\","
 "\"BatteryVoltage\": \"" +String(SystemVoltage) + "\","
 "\"SolarPower\": \"" + String(SolarPower) + "\","
 "\"WindPower\": \"" + String(WindPower) + "\","
 "\"GeneratorPower\": \"" + String(GeneratorPower) + "\","
 "\"InverterPower\": \"" + String(InverterPower) + "\","
 "\"MainsVoltage\": \"" + String(LineVoltage) + "\","
  "\"WindShutdown\": \"" + String(StormTimerEnable) + "\"}"
 "\"DumpLoadEnable\": \"" + String(DumpLoadEnable) + "\"}"
 "}";
 
 // Convert JSON string to character array
 char jsonChar[100];
 json.toCharArray(jsonChar, json.length()+1);
 
 // Publish JSON character array to MQTT topic
 mqtt.publish("/powerhouse/json", jsonChar); 




  //This section we will update display and do menu routine.

}

void wifiCb(void* response)
{
  uint32_t status;
  RESPONSE res(response);

  if(res.getArgc() == 1) {
    res.popArgs((uint8_t*)&status, 4);
    if(status == STATION_GOT_IP) {
      Serial.println("WIFI CONNECTED");
      mqtt.connect("10.0.0.106", 1883, false);
      WifiConnected = true;
      //or mqtt.connect("host", 1883); /*without security ssl*/
    } else {
      WifiConnected = false;
      mqtt.disconnect();
    }

  }
}

void mqttConnected(void* response)
{
  Serial.println("Connected");
  mqtt.subscribe("/powerhouse/Inverter1"); //or mqtt.subscribe("topic"); /*with qos = 0*/
  mqtt.subscribe("/powerhouse/Inverter2");
  mqtt.subscribe("/powerhouse/GridConnect");
  mqtt.subscribe("/powerhouse/WindShutdown");
  mqtt.subscribe("/powerhouse/Generator");
  mqtt.subscribe("/powerhouse/DumpLoadEnable");

  

}
void mqttDisconnected(void* response)
{

}


void mqttData(void* response)
{
  RESPONSE res(response);

  Serial.print("Received: topic=");
  String topic = res.popString();
  Serial.println(topic);

  Serial.print("data=");
  String data = res.popString();
  Serial.println(data);

  if ((topic=="Inverter1") & (data=="ON")) {Inverter1Enable=true; Inverter2Enable =false;} else Inverter1Enable=false;
  if ((topic=="Inverter2") & (data=="ON")){ Inverter2Enable=true; Inverter1Enable =false;} else Inverter2Enable=false; 
  if ((topic=="GridConnect") & (data=="ON")) GridConnectEnable=true; else GridConnectEnable=false; 
  if ((topic=="WindShutdown") & (data=="ON")) WindShutdownEnable=true; else WindShutdownEnable=false; 
  if ((topic=="Generator") & (data=="ON")) GeneratorEnable=true; else GeneratorEnable=false; 
  if ((topic=="DumpLoadEnabe") & (data=="ON")) DumpLoadEnable=true; else DumpLoadEnable=false; 
}
void mqttPublished(void* response)
{

}
