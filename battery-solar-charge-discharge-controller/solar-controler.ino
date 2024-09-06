#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  

// Set the LCD I2C address
LiquidCrystal_I2C lcd(0x27, 16, 2); //SDA - Pin A4 //SCL - Pin A5


//Sensors PINs
//const int invVoltageSensor = A0;//not used
//const int edpVoltageSensor = A1; //not used
const int batVoltageSensor = A2;
const int minPotentiometer = A6;
const int maxPotentiometer = A7;

//230V to 5V sensors
const int invSense = 2;
const int edpSense = 3;
const int outSense = 4;

//relays PINs
const int relayEDP = 12;
const int relayINV = 5;
const int solarRelay1 = 6;
const int solarRelay2 = 7;
const int solarRelay3 = 8;
const int solarRelay4 = 9;

//leds PINs
const int chargeLed = 10;
const int dischargeLed = 11;

//buttons_Switch PINs
const int changeModeButton = 13;

//vars
float batVoltage;
//float edpVoltage;
//float invVoltage;

int invSenseStatus;
int edpSenseStatus;
int outSenseStatus;

int manualModeChange = 1;
int mode = 2; //1 = EDP; 0 = INV; 2 = ALL OFF
int edpON;
int invON;
int outON;

int countDelay;
float minPotentiometerValue;
float maxPotentiometerValue;

float minVoltage;
float maxVoltage;

//Constants
const int loopDelay = 500; //miliseconds (1000 = 1seg)
//const int ModeChangeDelay = 1000 * 60; //miliseconds (1000 = 1seg)
const int countDelayLimit = 240; //2 min to change
const int relayChangeDelay = 2000; //miliseconds (1000 = 1seg)
const int INV = 0;
const int EDP = 1;
const int OFF = 2;

//debug mode
const bool debugMode = false;

//Solar charger controler values
const float deltaMaxToMin          = 0.4;
const float chargeRelay1MaxVoltage = 14.00; //10.00
const float chargeRelay2MaxVoltage = 13.20; //12.00
const float chargeRelay3MaxVoltage = 14.40; //14.00
const float chargeRelay4MaxVoltage = 14.60; //16.00
const float chargeRelay1MinVoltage = chargeRelay1MaxVoltage - deltaMaxToMin;
const float chargeRelay2MinVoltage = chargeRelay2MaxVoltage - deltaMaxToMin;
const float chargeRelay3MinVoltage = chargeRelay3MaxVoltage - deltaMaxToMin;
const float chargeRelay4MinVoltage = chargeRelay4MaxVoltage - deltaMaxToMin;

//custom charaters
byte batPic[8]        = {0b01100, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b00000};
byte edpONPic[8]      = {0b01010, 0b01010, 0b11111, 0b11111, 0b11111, 0b01110, 0b00100, 0b00100};
//{0b01010, 0b01010, 0b11111, 0b11111, 0b01110, 0b00100, 0b00100, 0b00000};
//{0b01010, 0b01010, 0b11111, 0b10001, 0b10001, 0b01110, 0b00100, 0b00100};
byte error[8]         = {0b11111, 0b11011, 0b11011, 0b11011, 0b11111, 0b11011, 0b11111, 0b00000};
//{0b00000, 0b11011, 0b01010, 0b00100, 0b00100, 0b01010, 0b11011, 0b00000};
byte separatorPic[8]  = {0b00111, 0b00011, 0b00101, 0b01000, 0b00010, 0b10100, 0b11000, 0b11100};
//{0b01000, 0b11111, 0b01000, 0b00000, 0b00010, 0b11111, 0b00010, 0b00000};
byte ok[8]            = {0b00000, 0b00001, 0b00011, 0b10110, 0b11100, 0b01000, 0b00000, 0b00000};
byte out[8]           = {0b00000, 0b01111, 0b00011, 0b00101, 0b01001, 0b10000, 0b01000, 0b00100};
byte lock[8]          = {0b01110, 0b10001, 0b10001, 0b11111, 0b11011, 0b11011, 0b11111, 0b00000};
byte unlock[8]        = {0b01110, 0b10001, 0b10000, 0b11111, 0b11011, 0b11011, 0b11111, 0b00000};
//byte clock[8]         = {0b00000, 0b00000, 0b01010, 0b10101, 0b10111, 0b10001, 0b01010, 0b00000};



void setup()
{
  if (debugMode) {
    Serial.begin(9600);
    Serial.println("Debug ON");
  }
  lcd.init();
  lcd.begin(16,2);
  lcd.backlight();
  lcd.clear();

  //Set the custom charaters
  lcd.createChar(0, batPic);
  lcd.createChar(1, edpONPic);
  lcd.createChar(2, error);
  lcd.createChar(3, separatorPic);
  lcd.createChar(4, ok);
  lcd.createChar(5, out);
  lcd.createChar(6, lock);
  lcd.createChar(7, unlock);
  //lcd.createChar(8, clock);
  //pins
  pinMode(batVoltageSensor, INPUT);
  //pinMode(edpVoltageSensor, INPUT); //not used
  //pinMode(invVoltageSensor, INPUT); //not used
  pinMode(invSense, INPUT_PULLUP);
  pinMode(edpSense, INPUT_PULLUP);
  pinMode(outSense, INPUT_PULLUP);

  //relays
  pinMode(relayEDP, OUTPUT);
  pinMode(relayINV, OUTPUT);
  pinMode(solarRelay1, OUTPUT);
  pinMode(solarRelay2, OUTPUT);
  pinMode(solarRelay3, OUTPUT);
  pinMode(solarRelay4, OUTPUT);

  //Leds
  pinMode(chargeLed, OUTPUT);
  pinMode(dischargeLed, OUTPUT);

  //buttons
  pinMode(changeModeButton, INPUT);

  //setMode(1); //start as charging
  //digitalWrite(dischargeLed, HIGH); // HIGH = LED ON
  digitalWrite(relayEDP, HIGH);     // HIGH = relay OFF
  digitalWrite(relayINV, HIGH);     // HIGH = relay OFF
}




void debugPrint(int i, char *txt)
{ 
  if (debugMode) {
    switch (i)
    {
    case 1: //
      Serial.print(txt);
      break;
    default: 
      Serial.println(txt);
      break;
    }
  }
}




//Bat Voltage Stuff
#define BAT_VAL_SIZE 10
float batValues[BAT_VAL_SIZE];

struct Tuple {
  int count;
  float mean;
};

struct Tuple countAndMean() {
  int count = 0;
  float countF = 0.0;
  float sum = 0.0;
  for (int i = 0; i < BAT_VAL_SIZE; i++) {
    if (batValues[i] > 0) {
      sum += batValues[i];
      count++;
      countF++;
    } else {
      break;
    }
  }

  Tuple result;
  if (count == 0) {
    float batValue = getBatVoltage();
    batValues[0] = batValue;
    result = {1, batValue};
  } else {
    result = {count, sum / countF};
  }
  return result;
}


float getBatVoltage() { //ANALOG READ
  //float intPart = 12.0;
  //float decimalPart = random(0,200) / 1000.0;
  //float batValue = intPart + decimalPart;
  //debugPrint(0,batValue);
  float batValue = analogRead(batVoltageSensor) / 40.92; //40.92

  return batValue;
}

float updateBatValues() {
  Tuple cm = countAndMean();
  float updatedVal = getBatVoltage();

  for (int i = 0; i < cm.count; i++) {
    if (cm.count < BAT_VAL_SIZE) {
      batValues[cm.count] = updatedVal;
    } else if (i < cm.count - 1) {
      batValues[i] = batValues[i + 1];
    } else {
      batValues[i] = updatedVal;
    }
  }

  cm = countAndMean();
  //debugPrint(1,cm.count);
  //debugPrint(1," - ");
  //debugPrint(0,cm.mean);
  return cm.mean;
}


float roundToOneDecimal(float x) {
  return (int)(x * 10) / 10.0;
}








void updatePics()
{
  /*
  lcd.createChar(0, batPic);
  lcd.createChar(1, edpONPic);
  lcd.createChar(2, error);
  lcd.createChar(3, separatorPic);
  lcd.createChar(4, ok);
  lcd.createChar(5, out);
  lcd.createChar(6, lock);
  lcd.createChar(7, unlock);
  */

  lcd.setCursor(10, 0);
  lcd.write((uint8_t)0);
  lcd.setCursor(12, 0);
  lcd.write((uint8_t)1);
  lcd.setCursor(14, 0);
  lcd.write((uint8_t)5);
  
  
  if (invSenseStatus) {
    lcd.setCursor(11, 0);
    lcd.write((uint8_t)4);
  } else {
    lcd.setCursor(11, 0);
    lcd.write((uint8_t)2);
  }
  
  
  if (edpSenseStatus) {
    lcd.setCursor(13, 0);
    lcd.write((uint8_t)4);
  } else {
    lcd.setCursor(13, 0);
    lcd.write((uint8_t)2);
  }
  
  if (outSenseStatus) {
    lcd.setCursor(15, 0);
    lcd.write((uint8_t)4);
  } else {
    lcd.setCursor(15, 0);
    lcd.write((uint8_t)2);
  }

  if (manualModeChange) {
    lcd.setCursor(9, 0);
    lcd.write((uint8_t)7);
  } else {
    lcd.setCursor(9, 0);
    lcd.write((uint8_t)6);
  }

  if (countDelay > 0) {
    lcd.setCursor(8, 0);
    lcd.print("*");
  } else {
    lcd.setCursor(8, 0);
    lcd.print(" ");
  }
}


void updateDisplay()
{
  if (batVoltage < 10) {
    lcd.setCursor(0, 1);
    lcd.print(batVoltage, 2);
    lcd.print(" ");
  } else {
    lcd.setCursor(0, 1);
    lcd.print(batVoltage, 2);
  }

  lcd.setCursor(7, 1);
  lcd.print(minVoltage, 1);
  lcd.setCursor(11,1);//
  lcd.write((uint8_t)3);
  lcd.setCursor(12,1);//
  lcd.print(maxVoltage, 1);

  updatePics();
  
  switch (mode) {
  case 0:
    //mode == 0  Inverter
    lcd.setCursor(0, 0);
    lcd.print("INV");
    //debugPrint(0,"Battery power...");
    break;
  case 1:
    //mode == 1  EDP
    lcd.setCursor(0, 0);
    lcd.print("EDP");
    //debugPrint(0,"Mains Power...");
    break;
  default:
    //mode == 2  All OFF
    lcd.setCursor(0, 0);
    lcd.print("OFF");
    break;
  }
}

void updatePinsData()
{
  //Mode button
  if (digitalRead(changeModeButton)) buttonPressed();

  //Voltage Readers
  batVoltage = updateBatValues();
  //batVoltage = analogRead(batVoltageSensor) / 40.92; //40.92 (already updated)
  //edpVoltage = analogRead(edpVoltageSensor) / 40.92; //40.92 //not used
  //invVoltage = analogRead(invVoltageSensor) / 40.92; //40.92 //not used
  invSenseStatus = !digitalRead(invSense); // !digitalRead(invSense);
  edpSenseStatus = !digitalRead(edpSense); // !digitalRead(edpSense);
  outSenseStatus = !digitalRead(outSense); // !digitalRead(outSense);
  invON = invSenseStatus;
  edpON = edpSenseStatus;
  outON = outSenseStatus;

  //Potentiometer reads
  minPotentiometerValue = analogRead(minPotentiometer);
  maxPotentiometerValue = analogRead(maxPotentiometer);

  minVoltage = roundToOneDecimal(minPotentiometerValue / (1023 / 1.5)) + 11;
  maxVoltage = roundToOneDecimal(maxPotentiometerValue / (1023 / 1.5)) + 13;

  /*
    debugPrint(1,"Bat: ");
    debugPrint(1,batVoltage);
    debugPrint(1,"\tEDP: ");
    debugPrint(1,edpVoltage);
    debugPrint(1,"\tINV: ");
    debugPrint(1,invVoltage);
    debugPrint(0,"");
  */
}




void setMode(int m) {
  countDelay = 0;
  if (mode != m) {
    mode = m;
    // ALL OFF
    digitalWrite(relayEDP, HIGH);     // HIGH = relay OFF
    digitalWrite(relayINV, HIGH);     // HIGH = relay OFF
    digitalWrite(chargeLed, LOW);     // HIGH = led ON
    digitalWrite(dischargeLed, LOW);  // HIGH = led ON
    
    while (outSenseStatus){
      debugPrint(0,"w8 for out = OFF");
      delay(relayChangeDelay);
      outSenseStatus = !digitalRead(outSense);
    }    
    

    switch (mode) {
    case INV:
      //mode == 0  Inverter
      digitalWrite(relayINV, LOW);        // HIGH = relay OFF
      digitalWrite(dischargeLed, HIGH);   // HIGH = led ON
      debugPrint(0,"Battery power...");
      while (!outSenseStatus)
      {
        debugPrint(0,"w8 for out = ON");
        delay(relayChangeDelay);
        outSenseStatus = !digitalRead(outSense);
        if (outSenseStatus) {
          lcd.setCursor(15, 0);
          lcd.write((uint8_t)4);
        } else {
          lcd.setCursor(15, 0);
          lcd.write((uint8_t)2);
        }
      }
      break;
    case EDP:
      //mode == 1  EDP
      digitalWrite(relayEDP, LOW);     // HIGH = relay OFF
      digitalWrite(chargeLed, HIGH);   // HIGH = led ON
      debugPrint(0,"Mains Power...");
      while (!outSenseStatus)
      {
        debugPrint(0,"w8 for out = ON");
        delay(relayChangeDelay);
        outSenseStatus = !digitalRead(outSense);
        if (outSenseStatus) {
          lcd.setCursor(15, 0);
          lcd.write((uint8_t)4);
        } else {
          lcd.setCursor(15, 0);
          lcd.write((uint8_t)2);
        }
      }
      break;
    default:
      //mode == 2  All OFF
      debugPrint(0,"All Off...");
      break;
    }
  }
}

void setModeWithDelay(int m) {
  if (mode != m) {
    countDelay = countDelay + 1;
    //char *t = "%i",countDelayLimit;
    //debugPrint(1,t);
    //debugPrint(1," --- ");
    //debugPrint(0,countDelay);

    if (countDelay > countDelayLimit) {
      setMode(m);
    }
  }

}




void buttonPressed()
{
  delay(relayChangeDelay);
  if (mode == 2){
    if (edpSenseStatus){
      lcd.setCursor(0, 0);
      lcd.print("ON ");
      delay(relayChangeDelay);
      setMode(EDP);
    } else if(invSenseStatus){
      lcd.setCursor(0, 0);
      lcd.print("ON ");
      delay(relayChangeDelay);
      setMode(INV);
    }
  } else if (manualModeChange){
    debugPrint(0,"buttonPressed - Wating 3 secs...");
    lcd.setCursor(0, 0);
    lcd.print("O.Manual");
    delay(relayChangeDelay);
    switch (mode)
    {
    case INV:
      setMode(EDP);
      break;
    case EDP:
      setMode(INV);
      break;
    }
    lcd.setCursor(0, 0);
    lcd.print("         ");
  }
}





void automaticOperation() {

  if (outON){ //Normal Operation
    debugPrint(0,"automaticOperation - outON");
    if (edpON && invON) {
      if (batVoltage < minVoltage) {
      //debugPrint(0,"< 12V");
      manualModeChange = 0;
      setModeWithDelay(EDP);
      } else if (batVoltage > maxVoltage) {
      //debugPrint(0,"> 14V");
      manualModeChange = 0;
      setModeWithDelay(INV);
      } else {
        manualModeChange = 1;
        countDelay = 0;
      }
    } else{
      manualModeChange = 0;
      countDelay = 0;
    }
  } else{
    if (invON) {
      manualModeChange = 0;
      setMode(INV);
    } else if (edpON) {
      manualModeChange = 0;
      setMode(EDP);
    } else {
      manualModeChange = 0;
      setMode(OFF);
    }
  }
}



void solarChargeController() {
  //Turn OFF relays
  if (batVoltage < chargeRelay1MaxVoltage)
  {
    //do nothing
  } else if (batVoltage < chargeRelay2MaxVoltage)
  {
    digitalWrite(solarRelay1, HIGH);
  } else if (batVoltage < chargeRelay3MaxVoltage)
  {
    digitalWrite(solarRelay1, HIGH);
    digitalWrite(solarRelay2, HIGH);
  } else if (batVoltage < chargeRelay4MaxVoltage)
  {
    digitalWrite(solarRelay1, HIGH);
    digitalWrite(solarRelay2, HIGH);
    digitalWrite(solarRelay3, HIGH);
  } else
  {
    digitalWrite(solarRelay1, HIGH);
    digitalWrite(solarRelay2, HIGH);
    digitalWrite(solarRelay3, HIGH);
    digitalWrite(solarRelay4, HIGH);
  }
  //Turn ON relays
  if (batVoltage < chargeRelay1MinVoltage)
  {
    digitalWrite(solarRelay1, LOW);
    digitalWrite(solarRelay2, LOW);
    digitalWrite(solarRelay3, LOW);
    digitalWrite(solarRelay4, LOW);
  } else if (batVoltage < chargeRelay2MinVoltage)
  {
    digitalWrite(solarRelay2, LOW);
    digitalWrite(solarRelay3, LOW);
    digitalWrite(solarRelay4, LOW);
  } else if (batVoltage < chargeRelay3MinVoltage)
  {
    digitalWrite(solarRelay3, LOW);
    digitalWrite(solarRelay4, LOW);
  } else if (batVoltage < chargeRelay4MinVoltage)
  {
    digitalWrite(solarRelay4, LOW);
  } else
  {
    //do nothing
  }

}



void loop()
{
  updatePinsData();
  updateDisplay();
  automaticOperation();
  solarChargeController();

  delay(loopDelay);
}