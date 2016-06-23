/*This program is the heart of the powerhouse. It measure the current shunts, battery voltages. It turns on the generator when needed and can also control the wind generator if need. It does
 * most of its work through the IIC network using INA226 ICs which are IIC connected to 75mv shunts and the 24v bus. It will also control the grid feed inverter when it needs to do so. It can 
 * also interface with a sd card via SPI to log results. The sd card logger contains the RTC as well. Also connected to the IIC is the lcd display driven with 3 buttons for user interaction. The line
 * voltage (240v) is also monistored directly by this IC to see if all is well to enable backup inverter operation. Also connected to the SPI is a modbus module for interconnection with lipo
 * bms system later.
 */


//pins definitions here

# define IIC 4 // on this bus will reside the lcd screen, and 4 INA226 ics to measure various system loads.
# define IIC 5
# define SPIpins 11
# define SPIpins 12
# define SPIpins 13
# define SdCardSelect 10  //slave select for talking to sd card.
# define CanBusSelect 8     //select can bus module to talk to with spi.
# define Esp8266Interrupt 2 // this is a physcial interrupt pin.
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

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
