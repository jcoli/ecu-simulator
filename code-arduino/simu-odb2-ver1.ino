// OBD2 simulator
// Simulates an ECU responding with CAN OBD-II messages to OBD-II scanner software requests.
// Work in Mode 1: Live Data Flow, Mode 3: Read DTC Fault Codes and Mode 4: Clear Faults.
// Author: Jeferson Coli - 10/2018
// Baseado no codigo de José Beltran Zambrano - Trabajo Fin de Grado - Grado en Ingeniería de las Tecnologías de Telecomunicación - Universidad de Sevilla - 2015

//libraries
#include <SPI.h>
#include "mcp_can.h"




/*Using Arduino Mega Pins
  Uno Mega  SPI
   12 50    MISO
   11 51    MOSI
   13 52    SCK
*/



#include <Adafruit_GFX.h> // graphics library for display
#include <Adafruit_PCD8544.h> // library for display Nokia 5110
//
Adafruit_PCD8544 display = Adafruit_PCD8544(5, 6, 7, 8, 9);

#include <Bounce2.h>
#define NUM_BUTTONS 8

#define INT32U unsigned long int

// Constants
const int SPI_CS_PIN = 10; // Pin CS
//Pins DTC
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {30, 31, 32, 33, 34, 35, 36, 37};
Bounce * buttons = new Bounce[NUM_BUTTONS];

//Pins Leds
const int LedMIL = 22; // Orange LED on pin 22 for the MIL indicator
const int LedOn = 23; // Blue LED on pin 23 for the ON indicator
const int LedRx = 24; // Green LED on pin 24 for the RX indicator
const int LedTx = 25; // Red LED on pin 25 for the TX indicator
const int LedEng = 26; // Red LED on pin 26 for the EngineOn indicator


//Analog inputs
const char SpeedPin = A0; // Analog speed input (potentiometer 1)
const char BateryPin = A3; // Analog battery input (potentiometer 2)
const char RPMPin = A1; // Analog rpm input (potentiometer 3)
const char O2Pin = A5; // Analog 03 input (potentiometer 4)
const char ThrottlePin = A4; // Analog 04 input (potentiometer 5)
const char TempPin = A2; // Analog 04 input (potentiometer 5)


// Variables
INT32U canId = 0x7E8; // CAN identifier
INT32U can_rx_Id = 0x700; // CAN identifier
unsigned char len = 0; // Length of the data received
unsigned char buf[8]; // Message storage buffer OBD-II
String BuildMessage = ""; // String to print the received message
int MODE = 0; // OBD-II operating mode
int PID = 0; // PParameters IDs of Mode 1
bool MILflag = 0; // Flag: 0 = MIL Off / 1 = MIL On
bool DTCflag = 0; // Flag indicating that there is at least one DTC in memory
int operMode = 0; // operating mode 0=normal(read analog inputs) 1=auto random 2=simulating normal car operation

bool DTC0flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC1flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC2flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC3flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC4flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC5flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC6flag = LOW; // Flag indicating that there is at least one DTC in memory
bool DTC7flag = LOW; // Flag indicating that there is at least one DTC in memory

bool OnFlag = LOW; // Flag indicating that there is at least one DTC in memory
bool TxFlag = LOW;
bool RxFlag = LOW;
bool EngFlag = LOW;

int dtcNumbers = 0; // Flag indicating that there is at least one DTC in memory
unsigned long TimeMIL = 0; // Timestamp in which MIL indicator was lit
float MetersMIL = 0; // Meters traveled while MIL is on
unsigned long Tans = 0; // Timestamp is used to count a second

int wMenu = 0; //menu select


float rpm = 0;
float speed = 0;
float o2 = 0;
float throttle = 0;
float temp = 0;
float volt = 0;
float fuelStatus = 0;
float maf = 0;
float rateEthanol = 0;

unsigned long start = millis();

// Define the CS pin for communication between Arduino-Uno and CAN-BUS Shield
MCP_CAN CAN(SPI_CS_PIN);

void setup()
{
  Serial.begin(115200);
  // Definition of the pins
  pinMode(LedMIL, OUTPUT); // Pin LedMIL output
  pinMode(LedOn, OUTPUT); // Pin LedOn output
  pinMode(LedRx, OUTPUT); // Pin LedRx output
  pinMode(LedTx, OUTPUT); // Pin LedTx output
  pinMode(LedEng, OUTPUT); // Pin LedEng output


  for (int i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    buttons[i].interval(25);              // interval in ms
  }

  // Definition of interruptions
  attachInterrupt( 1, interrupDTC1, RISING); // Sera atendida en flancos de subida

  //Display
  display.begin();
  display.setContrast(60); //Ajusta o contraste do display
  display.clearDisplay();   //Apaga o buffer e o display
  display.setTextSize(1);  //Seta o tamanho do texto
  display.setTextColor(BLACK); //Seta a cor do texto
  display.setCursor(0, 0); //Seta a posição do cursor
  display.print("TECNOCOLI");
  display.display();
  delay(2000);

  Serial.println("LCD started successfully");


START_INIT:
  // If it starts correctly we continue

  if (CAN_OK == CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS started successfully");
    display.setCursor(0, 10); //Seta a posição do cursor
    display.print("CAN BUS OK");
    display.display();
    delay(2000);
  }

  // otherwise we try again the start of the bus

  else {

    Serial.println("Error in the start of CAN BUS");
    Serial.println("Starting the CAN BUS again");
    delay(100);
    goto START_INIT;
  }
  Serial.println("Waiting message");
  display.clearDisplay();
  display.setTextSize(0);
  display.setCursor(0, 0); //Seta a posição do cursor
  display.println("CAN BUS OK");
  display.println("Waiting msg");
  display.println("");
  display.display();

  OnFlag = HIGH;
  digitalWrite(LedOn, OnFlag);

}



void loop()
{
  if (millis() - start > 700) {
    start = millis();

    switch (operMode) {
        case 0:
          readValuesAna();
          break;
        case 1:
          randomValues();
          break;
    }


    showValueDisplay();
  }
  // If it receives a CAN frame
  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    MODE = 0;
    PID = 0;
    // read the message
    BuildMessage = readMessage();
    // in mode

    switch (MODE) {
      case 1: // If we are working in mode 01 -> current data
        replyMode01(); //

        //Serial.println("CAN BUS Mode 0");
        break;
      case 3: // If we are working in mode 03 -> DTC
        //        display.clearDisplay();
        //        display.setCursor(0, 0); //Seta a posição do cursor
        //        display.print("CAN BUS Mode 3");
        //        display.display();
        Serial.println("CAN BUS Mode 3");
        replyMode03(); //
        break;
      case 4: // If we are working in mode 04 -> clean DTCs
        //        display.clearDisplay();
        //        display.setCursor(0, 0); //Seta a posição do cursor
        //        display.print("CAN BUS Mode 4");
        //        display.display();
        Serial.println("CAN BUS Mode 4");
        DTCflag = 0; // We lower the flag
        TimeMIL = 0; // We restart the time with MIL
        MetersMIL = 0; //
        Tans = 0; //
        dtcNumbers = 0; //
        digitalWrite(LedMIL, LOW);
        DTC0flag = false;
        DTC1flag = false;
        DTC2flag = false;
        DTC3flag = false;
        DTC4flag = false;
        DTC5flag = false;
        DTC6flag = false;
        DTC7flag = false;
        break;
    }

    BuildMessage = "";
  }

  for (int i = 0; i < NUM_BUTTONS; i++)  {
    // Update the Bounce instance :
    buttons[i].update();
    // If it fell, flag the need to toggle the LED
    if ( buttons[i].fell() ) {
      switch (i) {
        case 0:
          if (!DTC0flag) {
            DTC0flag = HIGH;
            DTCflag = HIGH;
            dtcNumbers++;
            replyMode03();
          }
          //Serial.println("Button 0 fell");
          break;
        case 1:
          if (!DTC1flag) {
            DTC1flag = HIGH;
            DTCflag = HIGH;
            dtcNumbers++;
            replyMode03();
          }
          //Serial.println("Button 1 fell");
          break;
        case 2:
          if (!DTC2flag) {
            DTC2flag = HIGH;
            dtcNumbers++;
          }
          //Serial.println("Button 2 fell");
          break;
        case 3:
          if (!DTC3flag) {
            DTC3flag = HIGH;
            dtcNumbers++;
          }
          //Serial.println("Button 3 fell");
          break;
        case 4:
          operMode++;
          if (operMode>2){
            operMode = 0;
          }
          Serial.println("but 4");
          //Serial.println("Button 4 fell");
          break;
        case 5:
            operMode++;
            if (operMode>2){
              operMode = 0;
            }
            Serial.println("but 5");

          //Serial.println("Button  5 fell");
          break;
        case 6:
          if (!DTC6flag) {
            //            DTC6flag = HIGH;
            //            dtcNumbers++;
            wMenu++;
            if (wMenu > 2) wMenu = 0;
          }
          //Serial.println("Button  6 fell");
          break;
        case 7:
          DTCflag = 0; // We lower the flag
          TimeMIL = 0; // We restart the time with MIL
          MetersMIL = 0; //
          Tans = 0; //
          dtcNumbers = 0; //
          digitalWrite(LedMIL, LOW);
          DTC0flag = false;
          DTC1flag = false;
          DTC2flag = false;
          DTC3flag = false;
          DTC4flag = false;
          DTC5flag = false;
          DTC6flag = false;
          DTC7flag = false;
          //          if (!DTC7flag) {
          //            DTC7flag = HIGH;
          //            dtcNumbers++;
          //          }
          //Serial.println("Button  7 fell");
          break;


      }
      if (dtcNumbers > 0) {

        digitalWrite(LedMIL, HIGH);

      }

      //Serial.println("Button fell");
    }
  }


}

void operModeAuto(){


}

String readMessage() {

  RxFlag = !RxFlag;
  digitalWrite(LedRx, RxFlag);

  String Message = "";
  CAN.readMsgBuf(&len, buf); // We store the OBD-II message in buf and its length in len
  // Print the address Id
  canId = CAN.getCanId(); // Message identifier
  Serial.print("<"); Serial.print(canId); Serial.print(",");
  // Build the message and print it
  for (int i = 0; i < len; i++)
  {
    if ( i == len - 1 ) {
      Message = Message + buf[i];
    }
    else {
      Message = Message + buf[i] + ",";

    }
  }
  Serial.println(Message);
  // We keep the mode and the PID required
  if ( buf[0] != 0 ) { // We check that the message is not empty
    MODE = buf[1];
    Serial.print("Modo: "); Serial.println(MODE);
    // If you work in mode one we keep the PID
    if (MODE == 1) {
      PID = buf[2];
      Serial.print("PID: "); Serial.println(PID);
    }
  }
  return Message;
}

// void replyMode01()
// Function to respond to an OBD-II request in Mode 1
void replyMode01()
{
  // Predefined OBD-II message to respond in mode 1
  // {len datos, 01+40hex, PID, DatoA, DatoB, DatoC, DatoD}
  TxFlag = !TxFlag;
  digitalWrite(LedTx, TxFlag);
  unsigned char OBDIImsg[8] = {4, 65, PID, 0, 0, 0, 0, 0}; // Initialize len to 4
  // Data = 85 or 0 in case they are not used
  int A = 0; // Variables for calculations
  int B = 0; // Variables for calculations
  switch (PID) {
    case 0: // PID 00 hex
      // The useful length in this case is 6
      OBDIImsg[0] = 6;
      // PIDs supported from 01-20 hex
      OBDIImsg[3] = 0x88;
      OBDIImsg[4] = 0x18;
      OBDIImsg[5] = 0x80;
      OBDIImsg[6] = 0x13;

      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg); // Answer
      break;
    case 1: // PID 01 hex
      // MIL encendido si DTCflag = 1
      if (DTCflag == 1) {
        OBDIImsg[3] = 129; // Light MIL ON and Number of detected faults
      }
      // MIL off, 0 stored DTCs, no possible test
      //DatoA,DatoB,DatoC,DatoD = 0;
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg); // Answer
      break;
    case 3: // PID 03 hex
      // Coolant temperature
      OBDIImsg[3] = fuelStatus;
      //
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;

      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
    case 5: // PID 05 hex
      // Coolant temperature
      OBDIImsg[3] = temp;
      //
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;

      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
    case 12: // PID 0C hex
      // Calculation of the RPM ((A * 256) + B) / 4

      A = rpm;
      OBDIImsg[3] = A * 4 / 256;
      OBDIImsg[4] = A - (A * 4 / 256);
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 13: // PID 0D hex
      // Calculation of speed
      OBDIImsg[3] = speed;
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 16: // PID 10 hex
      // Calculation of speed
      OBDIImsg[3] = maf;
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
    case 17: // PID 11 hex
      // Calculation of the position of the accelerator pedal
      OBDIImsg[3] = throttle;
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 28:

      // Standard OBD-II used
      OBDIImsg[3] = 3; // The value 3 indicates compatible with OBD and OBD-II
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 31: // PID 19 hex
      // Time in seconds since startup
      // The millis function measures the time in ms since the program started
      // Value between 0 and 65535s formula: (A * 256) + B
      A = millis() / 1000;
      OBDIImsg[3] = A / 256;
      OBDIImsg[4] = A - A / 256;
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 32: // PID 20 hex
      // The useful length in this case is 6
      OBDIImsg[0] = 6;
      // PIDs supported from 21-40 hex

      OBDIImsg[3] = 0x80;
      OBDIImsg[4] = 0x02;
      OBDIImsg[5] = 0x00;
      OBDIImsg[6] = 0x01;
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 33: // PID 21 hex
      // km traveled since the MIL indicator is on
      A = MetersMIL / 1000;
      // Between 0 and 65535 formula: (A * 256) + B
      OBDIImsg[3] = A / 256;
      OBDIImsg[4] = A - A / 256;
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 47: // PID 2F hex
      // fuel tank level
      OBDIImsg[3] = readFuelLvl();
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 64: // PID 40 hex
      // The useful length in this case is 6
      OBDIImsg[0] = 6;
      // PIDs supported from 41-60 hex
      OBDIImsg[3] = 0x04;
      OBDIImsg[4] = 0x08;
      OBDIImsg[5] = 0x80;
      OBDIImsg[6] = 0x00;
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 70: // PID 46 hex
      // Temperatura ambiente A-40
      OBDIImsg[3] = random(50, 80); // As we do not know, we put a random value
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 77: // PID 4D hex
      // Time since the MIL indicator is on
      // Value between 0 and 65535s formula: (A * 256) + B

      if (TimeMIL != 0) {
        A = (millis() - TimeMIL) / 60000; // Time in minutes
      }
      OBDIImsg[3] = A / 256;
      OBDIImsg[4] = A - A / 256;
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 81: // PID 51 hex
      // Indicates the type of fuel: bifuel running ethanol
      OBDIImsg[3] = 11;
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;
    case 83: // PID 51 hex
      // Indicates the % Ethanol
      OBDIImsg[3] = rateEthanol;
      // We build the answer
      CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
      break;

  }
}
// replyMode03
// Function to respond to an OBD-II request in Mode 3
void replyMode03() {
  // Predefined OBD-II message to respond in mode 1
  // {len data, 03+40hex, num DTCs, DTC1H, DTC1L, DTC2H, DTC2L}
  // {len data, 03+40hex, num DTCs, DTC1H, DTC1L, DTC2H, DTC2L, DTC3H, DTC3L }
  TxFlag = !TxFlag;
  digitalWrite(LedTx, TxFlag);

  unsigned char OBDIImsg[8] = {6, 67, 0, 0, 0, 0, 0, 0}; // Initialize to 0 there are no DTCs


  // Motor overheating has been detected

  if (DTCflag) {
    if (dtcNumbers <= 2) {
      if (DTC0flag) {

        OBDIImsg[2] = dtcNumbers; // amount of DTCs
        OBDIImsg[3] = 2; // P0217 Engine overheating HIGH
        OBDIImsg[4] = 23; // P0217 Engine overheating LOW
        //CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
        Serial.println("Mode 3 flag 0");
      }
      if (DTC1flag ) {

        OBDIImsg[2] = dtcNumbers; // amount of DTCs
        OBDIImsg[5] = 1; // P0217 Engine overheating HIGH
        OBDIImsg[6] = 22; // P0217 Engine overheating LOW
        //CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
        Serial.println("Mode 3 flag 1: ");


      }
      if (DTC2flag ) {

        OBDIImsg[2] = dtcNumbers; // amount of DTCs
        OBDIImsg[5] = 2; // P0217 Engine overheating HIGH
        OBDIImsg[6] = 2; // P0217 Engine overheating LOW
        //CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg);
        //Serial.println("Mode 3 flag 2");

      }
      //Serial.print("Mode 3: ");


      //Serial.println("=========SEND============");
      Serial.println(CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg));
    }
  }  else  {
    //Serial.println("=========SEND============");
    //Serial.println(CAN.sendMsgBuf(0x7E8, 0, 8, OBDIImsg));
  }

  //  for (int i = 0; i < 10; i++) {
  //    Serial.print(OBDIImsg[i]);
  //    Serial.print(":");
  //  }
  //  Serial.println("");
  //  Serial.println("=======");



}

void showValueDisplay() {
  if (wMenu == 0) {
    display.clearDisplay();
    display.setCursor(0, 0); //Seta a posição do cursor
    display.print("Speed: ");
    display.println(speed);
    display.print("RPM:   ");
    display.println(rpm);
    display.print("Temp:  ");
    display.println(temp);
    display.print("Volt:  ");
    display.println(volt);
    display.print("Thr:   ");
    display.println(throttle);
    display.print("DTC:");
    display.print(dtcNumbers);
    display.print(" Mode:");
    display.println(MODE);
    display.display();
  }
  if (wMenu == 1) {
    display.clearDisplay();
    display.setCursor(0, 0); //Seta a posição do cursor
    display.print("DTC:");
    display.print(dtcNumbers);
    display.print(" Mode:");
    display.println(MODE);
    if (DTC0flag) {
      display.println("P0271");
    }
    if (DTC1flag) {
      display.println("P0217");
    }
    if (DTC2flag) {
      display.println("P0171");
    }
    display.display();
  }
  if (wMenu == 2) {
    display.clearDisplay();
    display.setCursor(0, 0); //Seta a posição do cursor
    display.print("TX: ");
    display.println(TxFlag);
    display.print("RX: ");
    display.println(RxFlag);
    display.print("Engine: ");
    display.println(EngFlag);
    display.print("Can: ");
    display.println(EngFlag);
    display.print("Oper Mode: ");
    display.println(operMode);

    display.display();
  }

}

void randomValues(){

  rpm = (random(0, 8000));
  speed = (random(0, 200));
  o2 = (random(0, 100));;
  throttle = (random(0, 100));;
  temp = (random(20, 140));;
  volt = (random(10, 16));;
  fuelStatus = (random(0, 100));;
  maf = (random(0, 100));;
  rateEthanol = (random(0, 100));;

}

void readValuesAna() {

  speed = readSpeed();
  rpm = readRPM();
  throttle = readThrottle();
  volt = readFuelLvl();
  temp = readTemp();
  calMeters();
}

// Functions
// int readTemp()
// Inputs: none / * / Outputs: Speed
// Function to read the coolant temperature according to the position of the potentiometer 1
int readTemp() {
  // Read the analog input of PIN A0
  int sensorValue = analogRead(TempPin);
  // Convert the value read between 0-1023 to a value between 0-150
  // Change to map map(readValue, 0 , 1023, 100, 0);
  //int temp = sensorValue * (150.0 / 1023.0);
  int temp = map(sensorValue, 0 , 1023, 302, 32);

  return temp;
}


// Functions
// int readSpeed()
// Inputs: none / * / Outputs: Speed
// Function to read the speed according to the position of the potentiometer 1
int readSpeed() {
  // Read the analog input of PIN A0
  int sensorValue = analogRead(SpeedPin);
  // Convert the value read between 0-1023 to a value between 0-150
  // Change to map
  //int Speed = sensorValue * (150.0 / 1023.0);
  int Speed = map(sensorValue, 0 , 1023, 200, 0);
  if (Speed < 5) Speed = 0;

  return Speed;
}

// int readRPM()
// Inputs: none / * / Outputs: RPM
// Function to read the rpm according to the position of the potentiometer 3
int readRPM() {
  // Read the analog input of PIN A2

  int sensorValue = analogRead(RPMPin);
  // Convert the value read between 0-1023 to a value between 0-4000
  // Change to map 0 - 8000
  //int RPM = sensorValue * (4000.0 / 1023.0);
  int RPM = map(sensorValue, 0 , 1023, 8000, 0);
  if (RPM < 200) RPM = 0;
  if (RPM > 50) {
    EngFlag = !EngFlag;
  } else {
    EngFlag = false;
  }
  digitalWrite(LedEng, EngFlag);
  return RPM;
}

// int readThrottle()
// Inputs: none / * / Outputs: TPosition
// Function to read the position of the accelerator according to the position of the potentiometer 4
int readThrottle() {
  // Read analog entry of PIN A4
  int sensorValue = analogRead(ThrottlePin);
  // Convert the value read between 0-1023 to a value between 0-255
  // Change to map 0 - 100
  //int TPosition = sensorValue * (255.0 / 1023.0);
  int TPosition = map(sensorValue, 0 , 1023, 255, 0);


  return TPosition;
}

// int readFuelLvl()
// Inputs: none / * / Outputs: Level
// Function to read the level of the batery according to the position of the potentiometer 2
int readFuelLvl() {
  // Read analog entry of PIN A1
  int sensorValue = analogRead(BateryPin);
  // Convert the value read between 0-1023 to a value between 0-255
  // Change to map 0 - 16
  //int Level = sensorValue * (255.0 / 1023.0);
  int Level = map(sensorValue, 0 , 1023, 16, 0);

  return Level;
}

// void calMeters()
// Inputs: none / * / Outputs: none
// Calculate the meters traveled per second while MIL is on
// the meters are stored in the global variable MetersMIL
void calMeters() {

  if (Tans == 0) {
    Tans = millis();
  }
  else {
    // Calculate the meters traveled per second while MIL is on
    // We pass km / h to meters / second and add up every second elapsed
    int minCicle = (millis() - Tans);
    if (minCicle >= 1000) { // Si ha trascurrido 1 segundo
      // Calculamos los metros recorridos por segundo y sumamos
      MetersMIL = MetersMIL + (readSpeed() * 10 / 36);
      Tans = millis(); // Actualizamos Tans
    }
  }
}







// void interrupDTC1()
// Function that attends interruption 1
void interrupDTC1() {
  DTCflag = 1;
  TimeMIL = millis();
}
