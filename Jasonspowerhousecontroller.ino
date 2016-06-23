

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
#include <SoftwareSerial.h>
#include <espduino.h>
#include <mqtt.h>

//pins definitions here

# define IIC 4 // on this bus will reside the lcd screen, and 4 INA226 ics to measure various system loads.
# define IIC 5
# define SPIpins 11
# define SPIpins 12
# define SPIpins 13
# define SdCardSelect 10  //slave select for talking to sd card.
# define CanBusSelect 8     //select can bus module to talk to with spi.
# define Esp8266Power 2 // this is a physcial power enable pin.
# define 240Measure A0 //used for measureing line voltage.
# define SoftwareSerialRX 6       // used to listen for data maybe...
# define SoftwardSerialTX 7        // used to send data out 
# define GeneratorRunOutput 3     // used to run the generator if battery is low and no solar or wind coming in.
# define WindGeneratorShutdown 4    //used to shutdown the wind turbine if battery is full or remotely if needed.
# define DumpLoadOutput 9      // used to switch  on heaters etc during the day.
# define Inverter1Enable 5    // main inverter output enable
# define Inverter2Enable 17   //used to turn on backup inverter. Will only run if main inverter is off.
# define GridFeedEnable 16   //turn on the gridfeed inverter if trying to bulk charge.
# define ButtonsADC A1 //this is connected to some resistors to enable reading 3 buttons.

//constatns used within program
240multiplyer 50000000/78000/1024*5








//variables
SoftwareSerial debugPort(SoftwareSerialRx, SoftwareSerialTx); // RX, TX
ESP esp(&Serial, &debugPort, ESP8266Power);   //need to remove the debugport later.  
MQTT mqtt(&esp);


INA226 InaInverter;
INA226 InaSolar;
INA226 InaWind;
INA226 InaGenerator;
float SystemVoltage,InverterPower,InverterCurrent,SolarPower,SolarCurrent,WindPower,WindCurrent,GeneratorPower,GeneratorCurrent;  
int LineVoltage;
boolean WifiConnected = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(SdCardSelect,OUTPUT); // set dir
  pinMode(CanBusSelect,OUTPUT); //set dir
  pinMode(ESP8266Interrupt,INPUT);
  pinMode(240Measure, INPUT);
  pinMode(SoftwareSerialRx,INPUT);
  pinMode(SoftwareSerialTx,OUTPUT);
  pinMode(WindGeneratorShutdown,OUTPUT);
  pinMode(DumpLoadOutput,OUTPUT);
  pinMode(Inverter1Enable, OUTPUT);
  pinMode(Inverter2Enable, OUTPUT);
  pinMode(GridFeedEnable, OUTPUT);
  pinMode(ButtonsADC,INPUT);


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
  debugPort.begin(19200);
  esp.enable();
  delay(500);
  esp.reset();
  delay(500);
  while(!esp.ready());

  debugPort.println("ARDUINO: setup mqtt client");
  if(!mqtt.begin("jasonsmqtt", "admin", "password1914", 120, 1)) {  //need to edit this when server setup
    debugPort.println("ARDUINO: fail to setup mqtt");
    while(1);
  }


  debugPort.println("ARDUINO: setup mqtt lwt");
  mqtt.lwt("/lwt", "offline", 0, 0); //or mqtt.lwt("/lwt", "offline");

/*setup mqtt events */
  mqtt.connectedCb.attach(&mqttConnected);
  mqtt.disconnectedCb.attach(&mqttDisconnected);
  mqtt.publishedCb.attach(&mqttPublished);
  mqtt.dataCb.attach(&mqttData);

  /*setup wifi*/
  debugPort.println("ARDUINO: setup wifi");
  esp.wifiCb.attach(&wifiCb);

  esp.wifiConnect("jasonswifi","password1914");


  debugPort.println("ARDUINO: system started");


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
LineVoltage=analogRead(240Measure)*240multiplyer;  //this ends up being a integer. uses hardware filtering of rectified ac so only rough..




  //This section we will Read mqtt Serial etc commands
  esp.process();
  if(wifiConnected) {

  }



 //This section we will run some logics





 
  //This section we will setoutpus and send mqtt,




  //This section we will update display and do menu routine.

}

void wifiCb(void* response)
{
  uint32_t status;
  RESPONSE res(response);

  if(res.getArgc() == 1) {
    res.popArgs((uint8_t*)&status, 4);
    if(status == STATION_GOT_IP) {
      debugPort.println("WIFI CONNECTED");
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
  debugPort.println("Connected");
  mqtt.subscribe("/topic/0"); //or mqtt.subscribe("topic"); /*with qos = 0*/
  mqtt.subscribe("/topic/1");
  mqtt.subscribe("/topic/2");
  mqtt.publish("/topic/0", "data0");

}
void mqttDisconnected(void* response)
{

}
void mqttData(void* response)
{
  RESPONSE res(response);

  debugPort.print("Received: topic=");
  String topic = res.popString();
  debugPort.println(topic);

  debugPort.print("data=");
  String data = res.popString();
  debugPort.println(data);

}
void mqttPublished(void* response)
{

}
