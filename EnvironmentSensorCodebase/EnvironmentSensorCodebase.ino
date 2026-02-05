//Hardware in use:
//BH1750FVI - Digital Light intensity Sensor - DLI
//BME680 - Temperature, Humidity, Pressure and Gas Sensor - BME

//=================================Libraries=======================================
#include <Wire.h>                //Common Library needed for I2C communications
#include <SPI.h>
#include "RTClib.h"              //RTC Library for datetime, etc
#include <DFRobot_BMX160.h>      //For the IMU
#include <hp_BH1750.h>           //For the DLI sensor
#include <Adafruit_Sensor.h>     //This and below the BME sensor
#include "Adafruit_BME680.h"
#include <DueTimer.h>            //For Timer inturrupts
#include <Adafruit_GFX.h>        //For the TFT
#include <Adafruit_ST7735.h>     //For the TFT
#include <LiquidCrystal.h>
#include <SdFat.h>               //For the SD Card

//==============================Variable definitions===============================
//BME sensor variables if not using I2C protocol pins
// #define BME_SCK 13
// #define BME_MISO 12
// #define BME_MOSI 11
// #define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
char daysOfWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
Adafruit_BME680 bme;  //Sets variable for this to 'bme'
hp_BH1750 lightMeter;    //Sets variable for this to 'lightMeter'
RTC_DS1307 rtc;       //Sets variable for this to 'rtc'
DFRobot_BMX160 IMU;   //Sets variable for this to 'IMU'
SdFs sd;
FsFile file;          //For the SD card stuff
#define DIRO 9
String command;       //For the SDI recieve function to work
int deviceAddress = 0;
String deviceIdentification = "14ENG20009103612165"; //Doesn't include the device address as the first number
#define INTERPIN 19

//TFT and Menu stuff below
#define TFT_CS   10
#define TFT_RST  6 
#define TFT_DC   7         
#define TFT_SCLK 13   
#define TFT_MOSI 11
#define button1 5
#define button2 4
#define button3 3
#define button4 2
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
bool interruptFlag = false;
bool cancelRequest = false;           //For the continuous measurement inturrupt routine
bool continuousMActivated = false;    //For the continuous measurement inturrupt routine
bool buttonInterruptTimeFlag = false; //For the initial start up so the interrupts dont break
int currentMenuScreen = 0;            /*To determine where in the menu the user is currently:

1 = Main Menu
2 = BME Menu
3 = Light Menu

*/

//SD Card Items
const uint8_t SD_CS_PIN = A3;
const uint8_t SOFT_MISO_PIN = 12;
const uint8_t SOFT_MOSI_PIN = 11;
const uint8_t SOFT_SCK_PIN = 13;     

SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)

long temptemp = 0;
long temphum = 0;
long temppres = 0;                            //Variables for BME readings
long tempgas = 0;
long templux = 0;

long tempyear = 0;
long tempmonth = 0;
long tempday = 0;                             //Variables for RTC readings
long temphour = 0;
long tempminute = 0;

bool WDflag1 = false;
bool WDflag2 = false;                         //WatchDog flags
bool WDflag3 = false;
//===================================StartUp===========================================
void setup() {

Serial.begin(9600);
delay(2000);                            //Stops the non-character boxes being printed to the serial monitor
Wire.begin();                           //Initialise the I2C bus as the BH1750 library doesn't do this automatically
lightMeter.begin(BH1750_TO_GROUND);     //Start DLI
Serial1.begin(1200, SERIAL_7E1);        //SDI-12 communication begin. SDI-12 UART, configures serial port for 7 data bits, even parity, and 1 stop bit
pinMode(DIRO, OUTPUT);                  //DIRO Pin
digitalWrite(DIRO, HIGH);               //Open for communication
rtc.begin();                            //Start the rtc module
rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
IMU.begin();                            //Start the IMU module
lightMeter.begin(BH1750_TO_GROUND);     //Start the DLI seonsor - *Not sure if this needs an address or not.
bme.begin(0x76);                        //Start the BME sensor
BMEFullRead();

// Set up BME oversampling and filter initialization
bme.setTemperatureOversampling(BME680_OS_8X);
bme.setHumidityOversampling(BME680_OS_2X);
bme.setPressureOversampling(BME680_OS_4X);
bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
bme.setGasHeater(320, 150); // 320*C for 150 ms
//hardwareCheck(); //Run function for hardware check and debug.

//TFT and Munu stuff
tft.initR(INITR_BLACKTAB);
tft.setRotation(3);
pinMode(button1, INPUT_PULLUP);
pinMode(button2, INPUT_PULLUP);
pinMode(button3, INPUT_PULLUP);
pinMode(button4, INPUT_PULLUP);
tftBlack();
tftInit();
tftMenu();

Timer1.attachInterrupt(buttonInterruptFlag).start(3000000); //Set for 3 seconds so the interrupts dont break
Timer3.attachInterrupt(watchDog).start(10000000); //Set for 10 seconds

attachInterrupt(digitalPinToInterrupt(button1), MenuinterruptButton1, CHANGE);
attachInterrupt(digitalPinToInterrupt(button2), MenuinterruptButton2, CHANGE);
attachInterrupt(digitalPinToInterrupt(button3), MenuinterruptButton3, CHANGE);
attachInterrupt(digitalPinToInterrupt(button4), continuousMeasureCancel, FALLING);    //Interrupt for cancelling the continuous measurement.

//For the SD card stuff
sd.begin(SD_CONFIG);
if (!file.open("sensor_datanew.txt", O_RDWR | O_CREAT)) {
    sd.errorHalt(F("open failed"));
  }
file.close(); //release file



}

//==============================LOOP Function===============================
void loop() {
  int byte;
  if(Serial1.available()) {
    byte = Serial1.read();        //Reads incoming communication in bytes
    //Serial.println(byte);         //Testing line. Comment out when not testing.
    if (byte == 33) {             //If byte is command terminator (!)
      SDI12Receive(command);
      command = "";               //reset command string
    } else {
      if (byte != 0) {            //do not add start bit (0)
      command += char(byte);      //append byte to command string
      }
    }
  }
   }



//Check function for hardware debug
void hardwareCheck(){
    if (!Serial) {
    Serial.println("Serial failed to initialise.");}
    else {
    Serial.println("Serial connected");}

    if (!bme.begin(0x76)) {
    Serial.println("BME sensor not connected.");}
    else {
    Serial.println("BME connected.");}

    if (rtc.begin() == false) {
    Serial.println("RTC not connected.");}
    else{
    Serial.println("RTC connected.");}

    if (IMU.begin() == false) {
    Serial.println("IMU not connected.");}
    else{
    Serial.println("IMU connected.");}

    if (lightMeter.begin(BH1750_TO_GROUND) == false) {
    Serial.println("DLI not connected.");}
    else{
    Serial.println("DLI connected.");}
    }



//============================SDI12 COMMAND CENTRE - Recieve and Send Functions================================================

void SDI12Receive(String input) {
  int timeValue = 3;                                    //This is fixed. No function changes this value.
  int sensorMeasureTotal = 0; 
  String address = String(deviceAddress);               //convert device address to string
  String questionmark = "?";
  //Serial.println(address);                            //Testing line. Comment out when project complete
  //Serial.println(input);                              //Testing line. Comment out when project complete
  if (String(input.charAt(0)) == address) {             //Determines if the command is addressed for this device
    //Serial.println("First IF Statement");             //Testing line. Comment out when project complete
//                       ======================= A COMMAND ========================   
    if (String(input.charAt(1)) == "A") {
      //Serial.println("Second IF Statement");          //Testing line. Comment out when project complete
      switch (input.charAt(2)){                         //Switch case determines the character, and changes the device address accordingly.
        case 48:
          deviceAddress = 0;
          SDI12Send(String(deviceAddress));
          break;
        case 49:
          deviceAddress = 1;
          SDI12Send(String(deviceAddress));
          break;
        case 50:
          deviceAddress = 2;
          SDI12Send(String(deviceAddress));
          break;
        case 51:
          deviceAddress = 3;
          SDI12Send(String(deviceAddress));
          break;
        case 52:
          deviceAddress = 4;
          SDI12Send(String(deviceAddress));
          break;
        case 53:
          deviceAddress = 5;
          SDI12Send(String(deviceAddress));
          break;
        case 54:
          deviceAddress = 6;
          SDI12Send(String(deviceAddress));
          break;
        case 55:
          deviceAddress = 7;
          SDI12Send(String(deviceAddress));
          break;
        case 56:
          deviceAddress = 8;
          SDI12Send(String(deviceAddress));
          break;
        case 57:
          deviceAddress = 9;
          SDI12Send(String(deviceAddress));
          break;
      }
    }
//                       ======================= I COMMAND ========================   
    else if (String(input.charAt(1)) == "I"){
      SDI12Send(String(deviceAddress) + deviceIdentification);}

//                       ======================= M COMMAND ========================   
    else if (String(input.charAt(1)) == "M"){
      if(bme.performReading() == true && lightMeter.begin(BH1750_TO_GROUND) == true){
        sensorMeasureTotal = 5;}
        else if (bme.performReading() == true && lightMeter.begin(BH1750_TO_GROUND) == false){
          sensorMeasureTotal = 4;}
          else if (bme.performReading() == false && lightMeter.begin(BH1750_TO_GROUND) == true){
            sensorMeasureTotal = 1;}
            else if (bme.performReading() == false && lightMeter.begin(BH1750_TO_GROUND) == false){
              sensorMeasureTotal = 0;}
        
        digitalWrite(DIRO, LOW);
        delay(100);
        Serial1.print(deviceAddress);
        Serial1.print("00");
        Serial1.print(timeValue);
        Serial1.print(sensorMeasureTotal);
        Serial1.print(String("\r\n"));
        Serial1.flush();                             
        Serial1.end();
        Serial1.begin(1200, SERIAL_7E1);
        digitalWrite(DIRO, HIGH);}

//                       ======================= D COMMAND ========================   
    else if (String(input.charAt(1)) == "D"){
      if (String(input.charAt(2)) == "1"){
        if(bme.performReading() == true){
          digitalWrite(DIRO, LOW);
          delay(100);
          Serial1.print(deviceAddress);
          Serial1.print("+");
          Serial1.print(bme.temperature);
          Serial1.print("+");
          Serial1.print(bme.humidity);
          Serial1.print("+");
          Serial1.print(bme.pressure / 100.0);
          Serial1.print("+");
          Serial1.print(bme.gas_resistance / 1000.0);
          Serial1.print(String("\r\n"));
          Serial1.flush();                             
          Serial1.end();
          Serial1.begin(1200, SERIAL_7E1);
          digitalWrite(DIRO, HIGH);}
          else{SDI12Send("BME Failed to Read. Check Hardware and Try Again");
          //WDflag1 = true; //uncomment this later
          }}
        
        else if (String(input.charAt(2)) == "2"){
          if(lightMeter.begin(BH1750_TO_GROUND) == true){
            digitalWrite(DIRO, LOW);
            delay(100);
            Serial1.print(deviceAddress);
            Serial1.print("+");
            float luxRead = lightMeter.getLux();
            Serial1.print(luxRead);
            Serial1.print(String("\r\n"));
            Serial1.flush();                             
            Serial1.end();
            Serial1.begin(1200, SERIAL_7E1);
            digitalWrite(DIRO, HIGH);}
          else{SDI12Send("LightMeter Failed to Read. Check Hardware and Try Again");}}
      }
//                       ======================= R COMMAND ========================    
    else if (String(input.charAt(1)) == "R"){
      if (String(input.charAt(2)) == "0"){
        continuousMeasurement(1);}
        else if (String(input.charAt(2)) == "1"){
          continuousMeasurement(2);}
        else if (String(input.charAt(2)) == "2"){
          continuousMeasurement(3);}
        else if (String(input.charAt(2)) == "3"){
          continuousMeasurement(4);}
        else if (String(input.charAt(2)) == "4"){
          continuousMeasurement(5);}
        else if (String(input.charAt(2)) == "5"){
          continuousMeasurement(6);}
        else if (String(input.charAt(2)) == "6"){
          continuousMeasurement(7);}
        else if (String(input.charAt(2)) == "7"){
          continuousMeasurement(8);}
        else if (String(input.charAt(2)) == "8"){
          continuousMeasurement(9);}
        else if (String(input.charAt(2)) == "9"){
          continuousMeasurement(10);}
    }
  }
  else if (String(input.charAt(0)) == questionmark) {
    SDI12Send(String(deviceAddress));}
  else{
    SDI12Send("ERROR! If you have entered the correct address, check SDI12Recieve function."); //Sends error is address given does not match
    SDI12Send("Current Device Address Is:");
    SDI12Send(String(deviceAddress));
  }
}

void SDI12Send(String message) {
  digitalWrite(DIRO, LOW);
  //Serial.println("SDI12SEND Initiated");        //Testing line. Comment out when project complete
  delay(100);
  Serial1.print(message + String("\r\n"));        //Without the \r\n this line doesnt work.
  Serial1.flush();                                //Wait for print to finish
  Serial1.end();
  Serial1.begin(1200, SERIAL_7E1);
  digitalWrite(DIRO, HIGH);
  //Serial.println("SDI12SEND Finished");        //Testing line. Comment out when project complete
}

void continuousMeasureCancel(){
  if (continuousMActivated == true){
    cancelRequest = true;}
  }

void continuousMeasurement(int timeChosen){
 continuousMActivated = true; 
  while (cancelRequest == false){
      if(bme.performReading() == true){
      digitalWrite(DIRO, LOW);
      delay(1000*timeChosen);                             //Need to get rid of the delay for some kind of Timer choice.
      Serial1.print(deviceAddress);
      Serial1.print("+");
      Serial1.print(bme.temperature);
      Serial1.print("+");
      Serial1.print(bme.humidity);
      Serial1.print("+");
      Serial1.print(bme.pressure / 100.0);
      Serial1.print("+");
      Serial1.print(bme.gas_resistance / 1000.0);
      // Serial1.print("+");
      // float luxRead = lightMeter.getLux();                       //Uncomment these three lines if the Lux reader starts working
      // Serial1.print(luxRead);
      Serial1.print(String("\r\n"));}
      Serial1.flush();                             
      Serial1.end();
      Serial1.begin(1200, SERIAL_7E1);
      digitalWrite(DIRO, HIGH);}

  Serial1.flush();                             
  Serial1.end();
  Serial1.begin(1200, SERIAL_7E1);
  digitalWrite(DIRO, HIGH);
  continuousMActivated = false;
  cancelRequest = false;
}

//======================================= MAIN MENU CODES =================================================

void buttonInterruptFlag(){
  buttonInterruptTimeFlag = true;
  //Serial.println("Button Timer Interrupt");
}

void tftInit() {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST7735_WHITE); 
}

void tftBlack() {
  tft.fillScreen(ST7735_BLACK);
}

void tftMenu() {
  currentMenuScreen = 1;
  tft.setTextSize(2);
  tft.println(" Main Menu");
  tft.setTextSize(1);
  tft.setTextColor(ST7735_CYAN);
  tft.print(" 1. ");
  tft.setTextColor(ST7735_WHITE);
  tft.println("BME680 Environment");
  tft.setTextColor(ST7735_CYAN);
  tft.print(" 2. ");
  tft.setTextColor(ST7735_WHITE);
  tft.println("BH1750 Light");
  tft.setTextColor(ST7735_CYAN);
  tft.print(" 3. ");
  tft.setTextColor(ST7735_WHITE);
  tft.println("Magic Button");
}

void BMEFullRead(){
    if(bme.performReading() == true){
      delay(100);
      temptemp = bme.temperature;
      temphum = bme.humidity;
      temppres = bme.pressure / 100.0;
      tempgas = bme.gas_resistance / 1000.0;
    }
}
void BMEFullReadAGAIN(){
    bme.performReading();                              //Needs troubleshooting.
    temptemp = bme.temperature;
    temphum = bme.humidity;
    temppres = bme.pressure / 100.0;
    tempgas = bme.gas_resistance / 1000.0;

}

void LIGHTFullRead(){
      float luxRead = lightMeter.getLux();
      templux = luxRead;
}
void MenuinterruptButton1() {
  if (buttonInterruptTimeFlag == true){
  switch (currentMenuScreen){
    case 1:
      //If button 1 is pressed on Main Menu
      currentMenuScreen = 2;
      tftBlack();
      tftInit();
      buttonInterruptTimeFlag = false;
      tft.println(" BME680 selected");
      tft.println(" Current Reading:");
      tft.println("");
      tft.print(" Temperature: ");
      tft.setTextColor(ST7735_GREEN);
      tft.print(temptemp);
      tft.setTextColor(ST7735_WHITE);
      tft.println("C");
      tft.print(" Humidity: ");
      tft.setTextColor(ST7735_GREEN);
      tft.print(temphum);
      tft.setTextColor(ST7735_WHITE);
      tft.println("%");
      tft.print(" Pressure: ");
      tft.setTextColor(ST7735_GREEN);
      tft.print(temppres);
      tft.setTextColor(ST7735_WHITE);
      tft.println("hPa");
      tft.print(" Gas Resistance: ");
      tft.setTextColor(ST7735_GREEN);
      tft.print(tempgas);
      tft.setTextColor(ST7735_WHITE);
      tft.println("Kohms");
      tft.println("");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("1");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to Repeat");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("2");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to SAVE to SD");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("3");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to EXIT");
      break;
    case 2:
      tftBlack();
      tftInit();
      buttonInterruptTimeFlag = false;
      //BMEFullReadAGAIN();                           //Not working as intended. If called, code stalls becasue BME reads forever.
      tft.println(" New Reading:");
      tft.println("");
      tft.print(" Temperature: ");
      tft.setTextColor(ST7735_GREEN);
      tft.print(temptemp);
      tft.setTextColor(ST7735_WHITE);
      tft.println("C");
      tft.print(" Humidity: ");
      tft.setTextColor(ST7735_GREEN);
      tft.print(temphum);
      tft.setTextColor(ST7735_WHITE);
      tft.println("%");
      tft.print(" Pressure: ");
      tft.setTextColor(ST7735_GREEN);
      tft.print(temppres);
      tft.setTextColor(ST7735_WHITE);
      tft.println("hPa");
      tft.print(" Gas Resistance: ");
      tft.setTextColor(ST7735_GREEN);
      tft.print(tempgas);
      tft.setTextColor(ST7735_WHITE);
      tft.println("Kohms");
      tft.println("");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("1");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to Repeat");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("2");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to SAVE to SD");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("3");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to EXIT");      
      break;
    case 3:
      //If button 1 is pressed on Light Meter screen screen, perform the lightmeter reading again.
      tftBlack();
      tftInit();
      buttonInterruptTimeFlag = false;
      tft.println(" Light Meter selected");
      tft.println(" Current Reading:");
      tft.println("");
      tft.print(" Light: ");
      tft.setTextColor(ST7735_GREEN);
      tft.print(templux);
      tft.setTextColor(ST7735_WHITE);
      tft.println(" LUX");
      tft.println("");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("1");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to Repeat");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("2");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to SAVE to SD");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("3");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to EXIT"); 
      break;
    
    default:
      WDflag1 = true;
      break;
  }
  }
}
void MenuinterruptButton2() {
  if (buttonInterruptTimeFlag == true){
  switch (currentMenuScreen){
    case 1:
      //If button 2 is pressed on Main Menu
      currentMenuScreen = 3;
      tftBlack();
      tftInit();
      buttonInterruptTimeFlag = false;
      tft.println(" Light Meter selected");
      tft.println(" Current Reading:");
      tft.println("");
      tft.print(" Light: ");
      tft.setTextColor(ST7735_GREEN);
      tft.print(templux);
      tft.setTextColor(ST7735_WHITE);
      tft.println(" LUX");
      tft.println("");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("1");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to Repeat");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("2");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to SAVE to SD");
      tft.print(" Select ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("3");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to EXIT"); 
      break;
    case 2:
      //If button 2 is pressed on BME screen - Make the BME Reading Save to SD with RTC data
      buttonInterruptTimeFlag = false;
      saveSensorDataBME();
      tftBlack();
      tftInit();
      tft.println(" BME data save... ");
      tft.setTextColor(ST7735_GREEN);
      tft.setTextSize(2);
      tft.println(" Complete");
      tft.setTextColor(ST7735_WHITE);
      tft.setTextSize(1);
      tft.print(" Push ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("3");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to Exit");
      break;
    case 3:
      //If button 2 is pressed on Lightmeter screen. Save the lightmeter reading to SD card with RTC data
      saveSensorDataLight();
      buttonInterruptTimeFlag = false;
      tftBlack();
      tftInit();
      tft.println(" Light data save... ");
      tft.setTextColor(ST7735_GREEN);
      tft.setTextSize(2);
      tft.println(" Complete");
      tft.setTextColor(ST7735_WHITE);
      tft.setTextSize(1);
      tft.print(" Push ");
      tft.setTextColor(ST7735_CYAN);
      tft.print("3");
      tft.setTextColor(ST7735_WHITE);
      tft.println(" to Exit");
      break;

    default:
      WDflag2 = true;
      break;
  }
  }
}
void MenuinterruptButton3() {
  if (buttonInterruptTimeFlag == true){
  switch (currentMenuScreen){
    case 1:
      //If button 3 is pressed on Main Menu - MAGIC BUTTON
      buttonInterruptTimeFlag = false;
      tftBlack();
      for (int16_t x=0; x < tft.width(); x+=6) {
        tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, ST77XX_BLUE);}
      tft.setCursor(20, 60);
      tft.setTextColor(ST7735_GREEN);
      tft.setTextSize(3);
      tft.println(" MAGIC!");
      tft.setTextSize(1);
      tft.setTextColor(ST7735_WHITE);
      tftBlack();
      tftInit();
      tftMenu();
      break;
    case 2:
      //If button 3 is pressed on BME screen
      tft.setTextColor(ST7735_RED);
      tft.println(" Exiting to Main Menu...");
      tft.setTextColor(ST7735_WHITE);
      buttonInterruptTimeFlag = false;
      currentMenuScreen = 1;
      tftBlack();
      tftInit();
      tftMenu();
      break;
    case 3:
      //If button 3 is pressed on Lightmeter screen
      tft.setTextColor(ST7735_RED);
      tft.println(" Exiting to Main Menu...");
      tft.setTextColor(ST7735_WHITE);
      buttonInterruptTimeFlag = false;
      currentMenuScreen = 1;
      tftBlack();
      tftInit();
      tftMenu();
      break;
    
    default:
      WDflag3 = true;
      break;
  }
  }
}

void watchDog(){
  if(WDflag1 == false && WDflag2 == false && WDflag3 == false){
    //do nothing
  }
  else{
    SDI12Send("WatchDog has been initiated. Reset the Hardware");
    tftBlack();
    tftInit();
    tft.println("WatchDog Has been initiated.");
    tft.println("Reset the Hardware");
    WDflag1 = false;
    WDflag2 = false;
    WDflag3 = false;
  }
}
//===========================================

void saveSensorDataBME() {
  DateTime now = rtc.now();
  tempyear = now.year();
  tempmonth = now.month();
  tempday = now.day();
  temphour = now.hour();
  tempminute = now.minute();
  // Construct a string with sensor data

  String sensorData = "Temperature: " + String(temptemp) + "°C \nHumidity: " + String(temphum) +  "%" + "\nPressure: " + String(temppres) + "\nGas: " + String(tempgas) + " \nTimestamp YYYYMDHHMM: " + String(tempyear) + String(tempmonth) + String(tempday) + String(temphour) + String(tempminute);
  // Write sensor data to SD card
  //Serial.println(sensorData);
  WriteSD(file, sensorData);
}
void saveSensorDataLight() {
  DateTime now = rtc.now();
  tempyear = now.year();
  tempmonth = now.month();
  tempday = now.day();
  temphour = now.hour();
  tempminute = now.minute();
  // Construct a string with sensor data
  String sensorData = "Light Reading: " + String(templux) + " LUX \n" +" Timestamp YYYYMDHHMM: " + String(tempyear) + String(tempmonth) + String(tempday) + String(temphour) + String(tempminute);
  // Write sensor data to SD card
  //Serial.println(sensorData);
  WriteSD(file, sensorData);
}
void WriteSD(File file, String message) {  
  file.open("sensor_datanew.txt", O_RDWR);              //This does not APPEND! It will rewrite any previous saves.
  file.rewind();                //Go to file position 0
  file.println(message);
  file.close();
}
