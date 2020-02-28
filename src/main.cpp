  // Libraries
#include <Wire.h>                             // include I2C library
#include <MCP79410_Timer.h>                   // Scullcom Hobby Electronics library  http://www.scullcom.com/MCP79410Timer-master.zip
#include <Adafruit_MCP4725.h>                 // Adafruit DAC library  https://github.com/adafruit/Adafruit_MCP4725
#include <math.h>                             // more mathematical functions
#include <MCP342x.h>                          // Steve Marple library avaiable from    https://github.com/stevemarple/MCP342x
#include <LiquidCrystal_I2C.h>                // F Malpartida's NewLiquidCrystal library
                                              // https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/NewliquidCrystal_1.3.4.zip               
#include <Keypad.h>                           //http://playground.arduino.cc/Code/Keypad

// to change what is tested or not, change to a 0 (not tested) or 1 (tested)
#define TEST_I2C 0 // reports what I2C addresses are active on the line
#define TEST_ADC 0
#define TEST_DAC 0 // WARNING - DO NOT ACTIVATE THIS UNLESS THE MOSFETS ARE NOT CONNECTED - WARNING - THIS WILL DRIVE THE MOSFETS
#define TEST_RTC 0
#define TEST_LCD 0
#define TEST_THERMAL 0

#define CAL_DAC 0

#define KILO 1000

// LCD
//Set the pins on the I2C chip used for LCD connections
//ADDR,EN,R/W,RS,D4,D5,D6,D7
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);    //0x3F (0b0111111) is the default address of the LCD with I2C bus module (PCF8574A)
int VoltsDecimalPlaces = 3;                   //number of decimal places used for Voltage display on LCD

// THERMAL SENSOR
#define I2C_ADD_THERMAL 0b1001101

// RTC
#define I2C_ADD_RTC 0x6f
MCP79410_Timer timer = MCP79410_Timer(I2C_ADD_RTC);
#define DELAY_RTC 1000

// DAC
Adafruit_MCP4725 dac;                         //constructor
#define I2C_ADD_DAC 0x63
#define DAC_VOLTAGE 4.102 // DAC voltage reference in V
#define DAC_BASIS 4096 // 12 bit DAC has 4096 possibilites
#define DELAY_DAC 1500

// ADC
#define I2C_ADD_ADC 0x68                       //0x68 is the default address for the MCP3426 device
MCP342x adc = MCP342x(I2C_ADD_ADC);

// KEYPAD
const byte ROWS = 4;                          //four rows
const byte COLS = 4;                          //four columns
byte rowPins[ROWS] = {5, 6, 7, 8}; //connect to the row Arduino pinouts of the keypad
byte colPins[COLS] = {9, 10, 11, 12}; //connect to the column Arduino pinouts of the keypad
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); //initialize an instance of class NewKeypad
char customKey;
char decimalPoint;

char numbers[20];     // keypad number entry - Plenty to store a representation of a float
byte index = 0;
int z = 0;
float x = 0;
int y = 0;
int r = 0;

// CALIBRATION

// error relationship between set current and input current (linear)
#define CURRENT_SET_OFFSET 0.0395 // A
#define CURRENT_SET_SLOPE 0.0864 // A/A

// error relationship between measured current and input current (constant)
#define CURRENT_MEASURE_OFFSET 0.041 // A

// error relationship between measured voltage and input voltage (linear)
#define VOLTAGE_ERROR_OFFSET 0.1562 // V
#define VOLTAGE_ERROR_SLOPE -0.007032 // V/V

// FAN
#define PIN_FAN 3
#define TEMP_FAN_MIN 20 // temperature at which fan will activate
int tempCutOff = 60;                             // temperature at which circuit will shut off
int tempMax = 75;                             //maximum temperature when fan speed at 100%
int fanSpeed;

// SETTINGS
String Mode ="  ";                            //used to identify which mode
int modeSelected = 0;                         //Mode status flag
float reading = 0;                            //variable for Rotary Encoder value divided by 1000 (unused in current code)
int setReading = 0;                           //
int ControlVolts = 0;                         //used to set output current
float OutputVoltage = 0;                      //

float setCurrent = 0;                         //variable used for the set current of the load
float setPower = 0;                           //variable used for the set power of the load
float setResistance = 0;                      //variable used for the set resistance of the load
float setCurrentCalibrationFactor = 1;        //calibration adjustment - set as required (was 0.997)

float setControlCurrent = 0;                  //variable used to set the temporary store for control current required

// ERRORS
float voltageOffset = 0;                      //variable to store voltage reading zero offset adjustment at switch on
float currentOffset = 0;                      //variable to store current reading zero offset adjustment at switch on

// MEASUREMENTS
unsigned long controlVoltage = 0;             //used for DAC to control MOSFET
long current = 0;                             //variable used by ADC for measuring the current
long voltage = 0;                             //variable used by ADC for measuring the voltage
float ActualVoltage = 0;                      //variable used for Actual Voltage reading of Load
float ActualCurrent = 0;                      //variable used for Actual Current reading of Load
float ActualPower = 0;                        //variable used for Actual Power reading of Load
int temp;                                     //

// CUTOFFS
float PowerCutOff = 50;                       //maximum Power allowed in Watts - then limited to this level CAN BE CHANGED AS REQUIRED
float CurrentCutOff = 4;                      //maximum Current setting allowed in Amps - then limited to this level (was 5)
float ResistorCutOff = 999;                   //maximum Resistor we want to deal with in software
float BatteryCurrent;                         //
float LoadCurrent;                            //
float LVC = 0;

// ON/OFF
boolean toggle = false;                       //used for toggle of Load On/Off button
#define PIN_LOAD_ON_OFF 15                   //analog pin A1 used as a digital pin to set Load ON/OFF

// ADC
void readVoltageCurrent (void);
void ActualReading(void);
void zeroOffset (void);

// DAC
void dacControl (void);

// LCD
void LCDStart();
void LCDI2CCheckGood();
void LCDI2CCheckBad();
void displayEncoderReading (void);

// keypad
void readKeypadInput (void);
void inputValue (void);
void userSetUp (void);

// temp
void temperatureCutOff (void);
void getTemp();
void fanControl (void);
void LoadSwitch();


// I2C
int checkAddress(byte address);
void i2c_scanner();
int checkI2CDevices();

// ctrl
void Current(void);
void Power(void);
void Resistance(void);
void dacControlVoltage (void);
void powerLevelCutOff (void);
void maxConstantCurrentSetting (void);


void setup() {

  Serial.begin(9600);                                      //used for testing only

  Wire.begin();                                            //join i2c bus (address optional for master)
  Wire.setClock(400000L);                                  //sets bit rate to 400KHz

  // need to reset DAC straight away as default EEPROM setting is driving the DAC at 1/2 (should be fixed now by changing it EEPROM default)
  dac.begin(I2C_ADD_DAC);
  dac.setVoltage(0,false);                                 //reset DAC to zero for no output current set at Switch On

  if(TEST_I2C){
    i2c_scanner();
  }

  checkI2CDevices();
  delay(1000);

  pinMode(PIN_FAN, OUTPUT);
  TCCR2B = (TCCR2B & 0b11111000) | 1;                      //change PWM to above hearing
  pinMode (PIN_LOAD_ON_OFF, INPUT_PULLUP);


  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms

  LCDStart();

  lcd.setCursor(8,0);
  lcd.print("OFF");                                        //indicate that LOAD is off at start up
  Current();                                               //sets initial mode to be CC (Constant Current) at Power Up

  customKey = customKeypad.getKey();

  // zeroOffset();
  
}

void loop() {

  readKeypadInput();                                     //read Keypad entry
  LoadSwitch();                                          //Load on/off

  lcd.setCursor(18,3);                                   //sets display of Mode indicator at bottom right of LCD
  lcd.print(Mode);                                       //display mode selected on LCD (CC, CP, CR or BC)
  
  temperatureCutOff();                                   //check if Maximum Temperature is exceeded

  if(Mode != "TC" && Mode != "TP" && Mode != "TT"){      //if NOT transient mode then Normal Operation
    // reading = encoderPosition/1000;                        //read input from rotary encoder 
    maxConstantCurrentSetting();                           //set maxiumum Current allowed in Constant Current Mode (CC)
    powerLevelCutOff();                                    //Check if Power Limit has been exceeded
    displayEncoderReading();                               //display rotary encoder input reading on LCD
  }

  readVoltageCurrent();                                  //routine for ADC's to read actual Voltage and Current
  ActualReading();                                       //Display actual Voltage, Current readings and Actual Wattage
  
  dacControl();
  dacControlVoltage();                                   //sets the drive voltage to control the MOSFET

  fanControl();                                          //call heatsink fan control

}

//c_ADC.ino
//-----------------------Read Voltage and Current---------------------------------------------
void readVoltageCurrent (void) {
  
  MCP342x::Config status;
  // Initiate a conversion for voltage; convertAndRead() will wait until it can be read
  adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain1,                         //"gain1" means we have select the input amp of the ADC to x1
           1000000, voltage, status);
  
  // Initiate a conversion for current; convertAndRead() will wait until it can be read
  adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain4,                         //"gain4" means we have select the input amp of the ADC to x4
           1000000, current, status);
  
}

//----------------------Calculate Actual Voltage and Current and display on LCD---------------------
void ActualReading(void) {

  float VoltageError = 0.0;
  
  ActualCurrent = ((((current - currentOffset)*2.048)/32767) * 2.5) + CURRENT_MEASURE_OFFSET;        //calculate load current
  
  ActualVoltage = ((((voltage - voltageOffset)*2.048)/32767) * 50.4);       //calculate load voltage upto 100v (was 50)
  VoltageError = ActualVoltage*VOLTAGE_ERROR_SLOPE + VOLTAGE_ERROR_OFFSET;
  ActualVoltage = ActualVoltage + VoltageError;
  
  ActualPower = ActualVoltage*ActualCurrent;

  if (ActualPower <=0){
    ActualPower = 0;
  }

 if (ActualVoltage <=0.0){                              //added to prevent negative readings on LCD due to error
  ActualVoltage = 0.0;
 }
 
 if (ActualCurrent <= 0.0){                             //added to prevent negative readings on LCD due to error
  ActualCurrent = 0.0;
 }
  
 lcd.setCursor(0,1);
    
    if ( ActualCurrent < 10.0 ) {
        lcd.print(ActualCurrent,3);
    } else {
        lcd.print(ActualCurrent,2);
    }
    
    lcd.print("A");
    lcd.print(" ");
    
    if (ActualVoltage < 10.0) {
        lcd.print(ActualVoltage, 3);
    } else {
        lcd.print(ActualVoltage, 2);
    }    

    lcd.print("V");
    lcd.print(" ");
     
    if (ActualPower < 100 ) {
        lcd.print(ActualPower,2);
    } else {
        lcd.print(ActualPower,1);
    }
    lcd.print("W");
    lcd.print(" ");
}


//--------------------------Zero Setting Offset Routine--------------------------------------------
void zeroOffset (void) {

  delay(200);                                            //simple key bounce delay 
  readVoltageCurrent();                                  //routine for ADC to read actual Voltage and Current
  voltageOffset = voltage;
  currentOffset = current;

 //Serial.print("voltageOffset = ");                    //used for testing only
 //Serial.println(voltageOffset);                       //used for testing only
 //Serial.print("currentOffset = ");                    //used for testing only
 //Serial.println(currentOffset);                       //used for testing only
 
}

//c_DAC.ino
//--------------------------Set DAC Voltage--------------------------------------------
void dacControl (void) {
  
  if (!toggle){
    dac.setVoltage(0,false);                                 //set DAC output voltage to 0 if Load Off selected
  }
  else{
    //Serial.println("Control Voltage");                    //used for testing only
    //Serial.println(controlVoltage);                       //used for testing only
    dac.setVoltage(controlVoltage,false);                   //set DAC output voltage for Range selected
  }
  
}


//-----------------------DAC Control Voltage for Mosfet---------------------------------------
void dacControlVoltage (void) {

  float controlCurrentError = 0.0;
  
  if (Mode == "CC"){
    setCurrent = reading*1000;                                //set current is equal to input value in Amps
    setReading = setCurrent;                                  //show the set current reading being used
    setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  }

  if (Mode == "CP"){
    setPower = reading*1000;                                  //in Watts
    setReading = setPower;
    setCurrent = setPower/ActualVoltage;
    setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  }

  if (Mode == "CR"){
    setResistance = reading;                                  //in ohms
    setReading = setResistance;
    setCurrent = (ActualVoltage)/setResistance*1000;
    setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  }

  if (Mode == "TC" || Mode == "TP" || Mode == "TT"){                            //Transient Mode - Continuous
    setControlCurrent = (setCurrent * 1000) * setCurrentCalibrationFactor;
  }

  controlCurrentError = CURRENT_SET_SLOPE*setControlCurrent + CURRENT_SET_OFFSET;
  controlVoltage = setControlCurrent + controlCurrentError; 

}
//c_keypad.ino
// reads from keypad input
void readKeypadInput (void) {
  customKey = customKeypad.getKey();
  
 // if (customKey != NO_KEY){                             //only used for testing keypad
 //Serial.print("customKey = ");                          //only used for testing keypad
 //Serial.println(customKey);                             //only used for testing keypad
 // }                                                     //only used for testing keypad
/*
  if(customKey == '#'){                                      //check if Zero Offset Selected (press * and Cursor Button together){
    toggle = false;                                         //switch Load OFF
    zeroOffset();
  }
  
  if(customKey == '*'){                                     //check if Set-Up Mode Selected (press * and Cursor Button together)
    delay(200);
    toggle = false;                                         //switch Load OFF
    userSetUp();
    index = 0;
    z = 0;
    decimalPoint = (' ');                                   //clear decimal point text character reset
  }
          */       
  if(customKey == 'A'){                                   //check if Constant Current button pressed
    toggle = false;                                         //switch Load OFF
    lcd.setCursor(8,0);
    lcd.print("OFF");
    Current();                                              //if selected go to Constant Current Selected routine
    index = 0;
    z = 0;
    decimalPoint = (' ');                                   //clear decimal point test character reset
    }
           
  if(customKey == 'B'){                                   //check if Constant Power button pressed
    toggle = false;                                         //switch Load OFF
    lcd.setCursor(8,0);
    lcd.print("OFF"); 
    Power();                                                //if selected go to Constant Power Selected routine
    index = 0;
    z = 0;
    decimalPoint = (' ');                                   //clear decimal point test character reset
  }
            
  if(customKey == 'C'){                                   //check if Constant Resistance button pressed  
    toggle = false;                                         //switch Load OFF
    lcd.setCursor(8,0);
    lcd.print("OFF");  
    Resistance();                                           //if selected go to Constant Resistance Selected routine
    index = 0;
    z = 0;
    decimalPoint = (' ');                                   //clear decimal point test character reset
  }

  /*
    if(customKey == 'D'){                                   //check if Battery Capacity button pressed
    dac.setVoltage(0,false);                                //Ensures Load is OFF - sets DAC output voltage to 0
    toggle = false;                                         //switch Load OFF
    batteryType();                                          //select battery type
    index = 0;
    z = 0;
    decimalPoint = (' ');                                   //clear decimal point test character reset
  
      if (exitMode == 1){                                   //if NO battery type selected revert to CC Mode
      lcd.setCursor(8,0);
      lcd.print("OFF");
      Current();                                            //if selected go to Constant Current Selected routine
      encoderPosition = 0;                                  //reset encoder reading to zero
      customKey = 'A';
      }
      else
      {
      lcd.setCursor(16,2);
      lcd.print(BatteryType);                               //print battery type on LCD 
      lcd.setCursor(8,0);
      lcd.print("OFF");
      timer.reset();                                        //reset timer
      BatteryLifePrevious = 0;
      BatteryCapacity();                                    //go to Battery Capacity Routine
      }
    }
    */
  
  if (Mode != "BC"){
  
    if(customKey >= '0' && customKey <= '9'){               //check for keypad number input
         numbers[index++] = customKey;
         numbers[index] = '\0';
         lcd.setCursor(z,3);                              
         lcd.print(customKey);                              //show number input on LCD
         z = z+1;
       }
    
    if(customKey == '*'){                                   //check if decimal button key pressed
        if (decimalPoint != ('*')){                         //test if decimal point entered twice - if so skip 
        numbers[index++] = '.';
        numbers[index] = '\0';
        lcd.setCursor(z,3);
        lcd.print(".");
        z = z+1;
        decimalPoint = ('*');                             //used to indicate decimal point has been input
          }
        }
  
    if(customKey == '#') {                                //check if Load ON/OFF button pressed
          x = atof(numbers);     
           reading = x;
           // encoderPosition = reading*1000;
           index = 0;
           numbers[index] = '\0';
           z = 0;
           lcd.setCursor(0,3);
           lcd.print("        ");
           decimalPoint = (' ');                          //clear decimal point test character reset
            }
      }
    
}

//------------------------Key input used for Battery Cut-Off and Transient Mode------------------------
void inputValue (void){

 while(customKey != '#'){
  
  customKey = customKeypad.getKey();
  if(customKey >= '0' && customKey <= '9'){               //check for keypad number input
       numbers[index++] = customKey;
       numbers[index] = '\0';
       lcd.setCursor(z,r);                              
       lcd.print(customKey);                              //show number input on LCD
       z = z+1;
     }
  
  if(customKey == '*'){                                   //check if ZERO READING key pressed
      if (decimalPoint != ('*')){                         //test if decimal point entered twice - if so ski
      numbers[index++] = '.';
      numbers[index] = '\0';
      lcd.setCursor(z,r);
      lcd.print(".");
      z = z+1;
      decimalPoint = ('*');                               //used to indicate decimal point has been input
        }
      }

 if(customKey == 'C'){                                    //clear entry
    index = 0;
    z = y;
    lcd.setCursor(y,r);
    lcd.print("     ");
    numbers[index] = '\0';                                //
    decimalPoint = (' ');                                 //clear decimal point test character reset
  }

 }
 
  if(customKey == '#') {                                  //check if Load ON/OFF button pressed
    x = atof(numbers);     
    index = 0;
    numbers[index] = '\0';
    decimalPoint = (' ');                                 //clear decimal point test character reset
  }
}

// allows the user to setup their system
void userSetUp (void) {
  y = 14;
  z = 14;
    
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("User Set-Up");
  lcd.setCursor(0,1);
  lcd.print("Current Limit=");
  lcd.setCursor(19,1);
  lcd.print("A");
  r = 1;
  inputValue();
  CurrentCutOff = x;
  lcd.setCursor(14,r);
  lcd.print(CurrentCutOff,3);
  
  customKey = '0';

  z = 14;

  lcd.setCursor(0,2);
  lcd.print("Power Limit  =");
  lcd.setCursor(19,2);
  lcd.print("W");
  r = 2;
  inputValue();
  PowerCutOff = x;
  lcd.setCursor(14,r);
  lcd.print(PowerCutOff,2);
  
  customKey = '0';
  
  z = 14;
  
  lcd.setCursor(0,3);
  lcd.print("Temperature  =");
  lcd.setCursor(18,3);
  lcd.print((char)0xDF);
  lcd.print("C");
  r = 3;
  inputValue();
  tempCutOff = x;
  lcd.setCursor(14,r);
  lcd.print(tempCutOff);

  //delay(500);                                             //used in testing only
  lcd.clear();

  lcd.setCursor(8,0);
  lcd.print("OFF");
  Current();                                            //if selected go to Constant Current Selected routine
  customKey = 'A';
  
}
//c_lcd.ino
// prints a short starting message
void LCDStart(){
  

  // lcd.setBacklightPin(3,POSITIVE);                         // BL, BL_POL
  // lcd.setBacklight(HIGH);                                  //set LCD backlight on
 
  lcd.clear();                                             //clear LCD display
  lcd.setCursor(4,0);                                      //set LCD cursor to column 4, row 0
  lcd.print("BLUEsat");                                   //print BLUEsat to display with 5 leading spaces (you can change to your own)
  lcd.setCursor(1,1);                                      //set LCD cursor to column 0, row 1 (start of second line)
  lcd.print("Balloon Power");
  lcd.setCursor(1,2);
  lcd.print("DC Electronic Load"); //
  lcd.setCursor(1,3);
  lcd.print("Version 2.0"); //
  delay(3000);                                             //3000 mSec delay for intro display
  lcd.clear();                                             //clear dislay



}

// prints at start if all I2C devices can be found
void LCDI2CCheckGood(){

  lcd.begin(20, 4);                                        //set up the LCD's number of columns and rows 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("I2C devices good");
  lcd.setCursor(0,1);
  lcd.print("Disconnect loads");
  lcd.setCursor(0,2);
  lcd.print("Zeroing inputs");
  delay(2000);
  lcd.clear();
  
}

// prints at start if not all I2C devices can be found
void LCDI2CCheckBad(){

  lcd.begin(20, 4);                                        //set up the LCD's number of columns and rows 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("I2C issues");
  lcd.setCursor(0,1);
  lcd.print("Check cross power");
  lcd.setCursor(0,2);
  lcd.print("Debug w/ serial");
  delay(2000);
  lcd.clear();
    
}

//----------------------Display Rotary Encoder Input Reading on LCD---------------------------
void displayEncoderReading (void) {

    lcd.setCursor(8,2);                                      //start position of setting entry

    if ( ( Mode == "CP" || Mode == "CR" ) && reading < 100 ) {
        lcd.print("0");
    }
    
    if (reading < 10) {                                      //add a leading zero to display if reading less than 10
        lcd.print("0"); 
    }

    if ( Mode == "CP" || Mode == "CR" ) {
        lcd.print (reading, 2);                              //show input reading from Rotary Encoder on LCD
    } else {
        lcd.print (reading, 3);
    }
    //lcd.setCursor (CP, 2);                                   //sets cursor position
    //lcd.cursor();                                            //show cursor on LCD
}
//c_temp.ino
void temperatureCutOff (void){

  getTemp(); // retrieve the temperature
  
  if (temp >= tempCutOff){                                 //if Maximum temperature is exceeded
    reading = 0;
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(0,3);
    lcd.print("Over Temperature");
    lcd.setCursor(8,0);
    lcd.print("OFF");
    toggle = false;                                         //switch Load Off
  }
  
}

// pulls the temperature from the thermal setting
void getTemp(){

  Wire.beginTransmission(I2C_ADD_THERMAL);
  Wire.write(byte(0x00));
  Wire.endTransmission();
  
  Wire.requestFrom(I2C_ADD_THERMAL, 1);
  if (1 <= Wire.available()) { // if two bytes were received
    temp = Wire.read();  // receive high byte (overwrites previous reading)
  }

}


//-----------------------Fan Control----------------------------------------------------------
void fanControl (void) {

  getTemp(); // retrieve the temperature

  //Serial.print("Temp = ");                          //used for testing only
  //Serial.println(temp);                             //used for testing only

  if (temp < TEMP_FAN_MIN-4) {                                    //is temperature lower than really cold always turn off fan
      fanSpeed = 0;
      digitalWrite(PIN_FAN, LOW);                          //then fan turned off
  }

  if (temp >= TEMP_FAN_MIN+4) {      //Hysteresis to avoid fan starting and stopping around set point - needs to be large hystersis as fan causes thermal sensor to change temp easily
      fanSpeed = 131;
      digitalWrite(3, HIGH);
      //analogWrite(PIN_FAN, fanSpeed);
  }

/*
  if (temp >= 32  && temp < 40 ) {                     //Below 40 we run fan fixed at minimum
      fanSpeed = 131;
      analogWrite(PIN_FAN, fanSpeed);
  }
  

  if ((temp >= 40) && (temp < 50)){                    //OK we need the fan but let us keep it quiet if we can
      fanSpeed = map(temp, 40, 50, 131, 200);
      analogWrite(PIN_FAN, fanSpeed);
 }

  if ((temp >= 50) && (temp <= tempMax)){
      fanSpeed = map(temp, 50, 75, 131, 255);           //OK we need a jet stream and hearing protection
      analogWrite(PIN_FAN, fanSpeed);
  }

  if (temp > tempMax) {                                     //OK we need fan at full steam. Ready for take off
      fanSpeed = 255;
      digitalWrite(PIN_FAN, HIGH);
  }
*/
  lcd.setCursor(16,0);
  lcd.print(temp);
  lcd.print((char)0xDF);
  lcd.print("C");

  //Serial.print("Fan Speed ");                      //used for testing only
  //Serial.println(fanSpeed);                        //used for testing only
  
}
//d_ctrl.ino
//-----------------------Select Constant Current LCD set up--------------------------------
void Current(void) {
  
  Mode = ("CC");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");  
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set I = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  // CP = 9;                                               //sets cursor starting position to units.
  
}

//----------------------Select Constant Power LCD set up------------------------------------
void Power(void) {
  
  Mode = ("CP");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set W = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print("W");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  // CP = 10;                                               //sets cursor starting position to units.
  
}

//----------------------- Select Constant Resistance LCD set up---------------------------------------
void Resistance(void) {
  
  Mode = ("CR");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");  
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set R = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print((char)0xF4);
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  // CP = 10;                                               //sets cursor starting position to units.
  
}


//----------------------Power Level Cutoff Routine-------------------------------------------
void powerLevelCutOff (void) {
  
  if (ActualPower  > PowerCutOff){                        //Check if Power Limit has been exceed
    reading = 0;
    // encoderPosition = 0; 
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(0,3);
    lcd.print("Exceeded Power");
    lcd.setCursor(8,0);
    lcd.print("OFF");
    toggle = false;                                         //switch Load Off
  }
  
}

//----------------------Limit Maximum Current Setting-----------------------------------------
void maxConstantCurrentSetting (void) {
  
  if (Mode == "CC" && reading > CurrentCutOff){           //Limit maximum Current Setting
    reading = CurrentCutOff;
    lcd.setCursor(0,3);
    lcd.print("                    ");                      //20 spaces to clear last line of LCD 
  }

  if (Mode == "CP" && reading > PowerCutOff) {             //Limit maximum Current Setting
    reading = PowerCutOff;
    lcd.setCursor(0,3);
    lcd.print("                    ");                   //20 spaces to clear last line of LCD 
  }

   if (Mode == "CR" && reading > ResistorCutOff ) {             //Limit maximum Current Setting
    reading = ResistorCutOff;
    lcd.setCursor(0,3);
    lcd.print("                    ");                   //20 spaces to clear last line of LCD 
  }

}

//-----------------------Toggle Current Load ON or OFF------------------------------
void LoadSwitch(void) {

   delay(100);                                          //simple delay for key debounce (commented out if not required)
  if(ActualVoltage <= 8.0) { //code edit
    lcd.setCursor(8,0);
    lcd.print("LVC");
    //Load = 1;
    toggle = false;
  }
  else if (digitalRead(PIN_LOAD_ON_OFF) == LOW) {
    lcd.setCursor(8,0);
    lcd.print("On  ");
    lcd.setCursor(0,3);
    lcd.print("                    ");                 //clear bottom line of LCD
    //Load = 0;
    toggle = true;        
  }
  else{
    lcd.setCursor(8,0);
    lcd.print("Off ");
    //Load = 1;
    toggle = false;
  }
  
}
//d_I2C.ino
int checkAddress(byte address){

    byte error = 0;
    int retVal = -1;

    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0){
      Serial.print("I2C device found at address 0x");
      retVal = 1;
    }
    else if (error==4){
      Serial.print("Unknown error at address 0x");
      retVal = 0;
    }    
    else{
      Serial.print("I2C device NOT found at address 0x");
      retVal = 0;
    }

  if (address<16){
    Serial.println("0");
  }
  Serial.println(address,HEX);

  return retVal;

}

void i2c_scanner(){

  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknow error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

}

// checks if all the specified devices are present on the line
int checkI2CDevices(){

  if (checkAddress(I2C_ADD_DAC) & checkAddress(I2C_ADD_ADC) & checkAddress(I2C_ADD_RTC) & checkAddress(I2C_ADD_RTC) & checkAddress(I2C_ADD_THERMAL)){
    LCDI2CCheckGood();
    return 1;
  }
  else{
    LCDI2CCheckBad();
    return 0;
  }

}
//d_test_cal.ino
void testDAC();
void testRTC();

// outputs voltages that increase
// THIS WILL DRIVE THE MOSFET IF USED IN THE COMPLETE CIRCUIT - UNSAFE AS COULD DRAW LARGE AMOUNTS OF CURRENT
void testDAC(){
  
    Serial.println("Start: Testing DAC...");
    
    Serial.println("Output: 0V");
    dac.setVoltage(DAC_BASIS*0/DAC_VOLTAGE, false);
    delay(DELAY_DAC);
    
    Serial.println("Output: 1V");
    dac.setVoltage(DAC_BASIS*1/DAC_VOLTAGE, false);
    delay(DELAY_DAC);

    Serial.println("Output: 2V");
    dac.setVoltage(DAC_BASIS*2/DAC_VOLTAGE, false);
    delay(DELAY_DAC);

    Serial.println("Output: 3V");
    dac.setVoltage(DAC_BASIS*3/DAC_VOLTAGE, false);
    delay(DELAY_DAC);

    Serial.println("Output: 4V");
    dac.setVoltage(DAC_BASIS*4/DAC_VOLTAGE, false);
    delay(DELAY_DAC);

    Serial.println("Finish: Testing DAC\n");
}

// measures time after various intervals to check RTC measurements
void testRTC(){

   Serial.println("Start: Testing RTC...");

    uint32_t seconds = 0;
    String strTime;

    // reset the timer
    timer.reset();

    // start the timer
    timer.start();

    delay(50); // to avoid rounding errors from operating near a second exactly

    for(int i = 1; i <= 5; i++){
      delay(i*DELAY_RTC);
      seconds = timer.getTotalSeconds();
      strTime = timer.getTime();
      // print on the console
      // Serial.println(seconds);
      Serial.println(strTime);
    }

    // stop the timer
    timer.stop();

    // reset the timer
    timer.reset();

    Serial.println("Finish: Testing RTC\n");

}

// reads the channel one input to make sure the ADC is measuring reasonably
void testADC(){

   Serial.println("Start: Testing ADC...");

  long value = 0;
  MCP342x::Config status;
  
  // Initiate a conversion; convertAndRead() will wait until it can be read
  adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain1,                         //"gain1" means we have select the input amp of the ADC to x1
           1000000, value, status);
  
  Serial.print("ADC Channel 1: ");
  Serial.print(((value*2.048)/32767) * 2.5);
  Serial.println("V");

  lcd.print("ADC Channel 1: ");
  lcd.print(((value*2.048)/32767) * 2.5);
  lcd.print("V");

  lcd.print("Rail Voltage: ");
  lcd.print(((value*2.048)/32767) * 2.5 * 50);
  lcd.print("V");

  Serial.println("Finish: Testing ADC\n");

  
}

// allows divide by 10 resistor to be precisely set
void DACcalibrate(){

  Serial.println("Calibrating DAC...");
  
  dac.setVoltage(DAC_BASIS*4/DAC_VOLTAGE, false);
  Serial.println("Output voltage at 4V, adjust until 1/10 divider accurate");
  delay(10000);
  
  dac.setVoltage(DAC_BASIS*2/DAC_VOLTAGE, false);
  Serial.println("Output voltage at 2V, check 1/10 divider is still accurate");
  delay(50000);

  Serial.println("DAC calibrated...\n");

  
}

// pulls the temperature from the thermal setting
void testThermal(){

  Serial.println("Start: Testing thermal...");


  int8_t reading = 0;

  Wire.beginTransmission(I2C_ADD_THERMAL);
  Wire.write(byte(0x00));
  Wire.endTransmission();
  
  Wire.requestFrom(I2C_ADD_THERMAL, 1);
  if (1 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
  }

  Serial.print("Temperature is: ");
  Serial.print(reading);   // print the reading
  Serial.println("C");

  Serial.println("Finish: Testing thermal\n");

}

void printCustom(char text[]){

  Serial.println(text);

  if(TEST_LCD){
        lcd.print(text);
  }
  
}

// changes the DAC default output to 0V (by writing to its EEPROM)
// all values are currently hardcoded
// from datasheet C2 = 0, C1 = 1, C0 = 1 to write to EEPROM
void changeDACDefault(){

  Wire.beginTransmission(I2C_ADD_DAC);
  Wire.write(0b01100000); // C2, C1, C0, XX, PD1, PD0, X
  Wire.write(0x00); // D11 - D4
  Wire.write(0x00); // D3 - D0, XXXX
  Wire.endTransmission();
  
}
