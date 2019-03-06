// JMI Smart Focuser Emulation for arduino

// Need license info


//#define LCD_DISPLAY             // Sets up compile options
#define U8G_DISPLAY


#include <Wire.h>
#include <AccelStepper.h>

#ifdef U8G_DISPLAY
  #include "U8glib.h"
  U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);
#endif

#ifdef LCD_DISPLAY
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x27,20,4);  // set LCD address to 0x27 for a 16 chars and 4 line display
#endif

int incomingByte = 0;   // for incoming serial data
int pos_lo=0;
int pos_hi=0;
byte pos_buf[4];
byte SByte;
boolean step_status=0;

int movStatus=0;

/*

Variables for eeprom
EEpromPOS - stored position
EEMaxSpeed - Max Acc stepper speed
EEMotorRev - motor reverse (using define statement currently)
EEMovIn  -  Move In stepper count (for manual movement?)
EEMovOut -  Move Out stepper count (for manual movement?)


Enhancements
    eeprom storage for  above variables
    simple menu display
    lcd display (smaller display)
    Buttons -   IN, OUT  (Select, Next - for menus) -or- encoder
    LCD back light control
    U8G display control, on/off

    reset - restore defaults

*/


// recover from eeprom
int EEpromPOS=8000;
int posval=EEpromPOS;
int goto_loc=posval;
int max_pos=16300;

#define VERSION  "0.70"   // version string
#define REVMOTOR          // reverse motor movement
#define HALFSTEP 8    // HALF4WIRE  (faster)
//#define HALFSTEP 4        // FULL4WIRE
#define motorPin8   8     // IN1 on the ULN2003 driver 2
#define motorPin9   9     // IN2 on the ULN2003 driver 2
#define motorPin10  10    // IN3 on the ULN2003 driver 2
#define motorPin11  11    // IN4 on the ULN2003 driver 2

// for remote manual push button
#define InBut   5         // Digital pin 5
#define OutBut  6         // Digital pin 6

// define step count for manual push button
#define MButStep 100

// Define a stepper and the pins it will use
//AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

#ifdef REVMOTOR
  // reversed
  AccelStepper stepper(HALFSTEP, motorPin9, motorPin11, motorPin8, motorPin10);
#else
  // non-reversed
  AccelStepper stepper(HALFSTEP, motorPin8, motorPin10, motorPin9, motorPin11);
#endif

// version
char Version[6]=VERSION;

void setup() {

  // pin defs
  pinMode(InBut,INPUT_PULLUP);
  pinMode(OutBut,INPUT_PULLUP);

// Splash Screens
#ifdef LCD_DISPLAY
  lcd.init();     // initial lcd display
  // Display Splash
  lcd.noAutoscroll();
  lcd.backlight();
  lcd.setBacklight(2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Focuser, Version: ");
  lcd.setCursor(2,1);
  lcd.print (VERSION);
  lcd.setCursor(6,2);
  lcd.print ("by W0ANM");
  lcd.setCursor(0,3);
  lcd.print("Initializing....");
  delay(2000);
  lcd.clear();
#endif

#ifdef U8G_DISPLAY
  u8g.firstPage();
  do {
       u8g.setFont(u8g_font_7x13);
       //u8g.drawStr(x,y,"<string>");
       //where x is starting pix (width) and y is starting pix (Height)

       u8g.drawStr(0,20,"Focuser Version ");
       u8g.drawStr(0,30,Version);
       u8g.drawStr(0,40,"by W0ANM ");
  } while(u8g.nextPage() );

  delay(2000);

#endif

  // stepper init
  //stepper.setMaxSpeed(400.0);
  //stepper.setAcceleration(100.0);
  stepper.setMaxSpeed(500.0);
  stepper.setAcceleration(200.0);
  stepper.setCurrentPosition(EEpromPOS*2);

  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps

#ifdef LCD_DISPLAY
  displayStatus();
#endif

#ifdef U8G_DISPLAY
  ledDisplayStatus();
#endif

}

void loop() {

  byte buf[2];
  byte serbuf[2];
  buf[0]=81;
  buf[1]=82;

  // Check for Manual IN/OUT Focus
  if (digitalRead(InBut) == 0 ) {
     //move focuser IN
     goto_loc=posval-MButStep;
     // non-blocking
     stepper.moveTo((goto_loc*2)); //cjk
     //stepper.setSpeed(200);  //screws up speed.
     step_status=1;                     // set status 1=running, 0=done
  }

  if (digitalRead(OutBut) == 0) {
     //move focuser OUT
     goto_loc=posval+MButStep;
     // non-blocking
     stepper.moveTo((goto_loc*2)); //cjk
     //stepper.setSpeed(200);  //screws up speed.
     step_status=1;                     // set status 1=running, 0=done
  }

  // send data only when you receive data:
  if (Serial.available() > 0) {
     // read the incoming byte:
     incomingByte = Serial.read();

     switch(incomingByte) {
     case 'b':                                // Read Device identify
       // send 'b', then 'j'
       buf[0]=0x62;        //'b'
       buf[1]=0x6a;        //'j'
       Serial.write(buf, sizeof(buf));
       break;

     case 'p':                                // Read Position Register
       pos_buf[0] = posval & 255;
       pos_buf[1] = (posval >> 8)  & 255;
       Serial.write('p');
       Serial.write(pos_buf[1]);
       Serial.write(pos_buf[0]);
       break;


     case 't':                                // Read Device Status register
       Serial.write('t');
       SByte=0;
       Serial.write(SByte);                     // return status byte
       break;

     case 'i':                                // Move IN Command (slow speed)
       // not used in ekos, not completed
       // after focus controller receives byte, it will start the motor and then
       // look for encoder changes.  When motion is detected, the command is echoed back.
       // if fails to move, then returns 'r'
       Serial.write('i');
       posval=posval+10;
       break;

     case 'o':                                // Move OUT Command (slow speed)
       // not used in ekos, not completed
       // after focus controller receives byte, it will start the motor and then
       // look for encoder changes.  When motion is detected, the command is echoed back.
       // if failes to move, then returns 'r'
       Serial.write('o');
       posval=posval-10;
       break;

     case 's':                                // stop focuser, abort
       stepper.stop();
       turnmotoroff();
       step_status=0;
       movStatus=2;

#ifdef U8G_DISPLAY
       ledDisplayStatus();
#endif

#ifdef LCD_DISPLAY
       lcd.setCursor(8,0);
       lcd.print("ABORTED");
#endif
       Serial.write('s');
       break;

     case 'g':                               //Goto Specified Position
       // read next two bytes
       Serial.readBytes(serbuf,2);
       pos_hi=serbuf[0];
       pos_lo=serbuf[1];
       Serial.write('g');
       // 2 bytes to integer value
       goto_loc=BitShiftCombine(pos_hi, pos_lo);
       if (goto_loc > max_pos) {
          goto_loc=max_pos-10;
       }

#ifdef LCD_DISPLAY
       displayStatus();
#endif

#ifdef U8G_DISPLAY
       movStatus=1;
       ledDisplayStatus();
#endif

       // non-blocking
       stepper.moveTo(goto_loc*2);
       //stepper.setSpeed(200);  //screws up speed.
       step_status=1;                     // set status 1=running, 0=done
       break;
     }
  }

   // starts stepper movement from 'g' to goto_loc specified
  if (step_status == 1) {
     if (stepper.currentPosition() != goto_loc*2) {
       stepper.run();

#ifdef LCD_DISPLAY
       lcd.setCursor(8,0);
       lcd.print("MOVING ");
#endif

     }
     else {
       turnmotoroff();
       posval=goto_loc;

#ifdef LCD_DISPLAY
       lcd.setCursor(8,0);
       lcd.print("STOPPED");
       // update location
       lcd.setCursor(7,2);
       lcd.print(posval,DEC);
#endif


#ifdef U8G_DISPLAY
       movStatus=0;
       ledDisplayStatus();
#endif

       // send 'c' to complete, an 'r' if moter fails
       // 'r' not implemented, no easy way to check
       Serial.write('c');
       step_status=0; // not running-- done
     }
  }
}

//   two byte to interger conversion
int BitShiftCombine( unsigned char x_high, unsigned char x_low)
{
  int combined;
  combined = x_high;              //send x_high to rightmost 8 bits
  combined = combined<<8;         //shift x_high over to leftmost 8 bits
  combined |= x_low;              //logical OR keeps x_high intact in combined and fills
                                  //in rightmost 8 bits
  return combined;
}


#ifdef LCD_DISPLAY
void displayStatus() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Status:");
  lcd.setCursor(0,1);
  lcd.print("GoPosI:");
  lcd.setCursor(7,1);
  lcd.print(goto_loc,DEC);
  lcd.setCursor(0,2);
  lcd.print("CuPosI:");
  lcd.setCursor(7,2);
  lcd.print(posval,DEC);
  lcd.setCursor(0,3);
  lcd.print("SByte:");
  lcd.setCursor(6,3);
  lcd.print(SByte,HEX);
}
#endif

void turnmotoroff() {
  digitalWrite(motorPin8,LOW);
  digitalWrite(motorPin9,LOW);
  digitalWrite(motorPin10,LOW);
  digitalWrite(motorPin11,LOW);
}


#ifdef U8G_DISPLAY
void ledDisplayStatus () {

  char goto_locC[6];
  char posvalC[6];


  dtostrf(goto_loc,5,0,goto_locC);
  dtostrf(posval,5,0,posvalC);

  u8g.firstPage();
  do {
       u8g.setFont(u8g_font_7x13);

       //u8g.drawStr(x,y,"<string>");
       //where x is starting pix (width) and y is starting pix (Height)

       u8g.drawStr(0,30,"Status: ");
       if ( movStatus == 0 ) {
          u8g.drawStr(54,30,"STOPPED");
       }
       if ( movStatus == 1 ) {
          u8g.drawStr(54,30,"MOVING");
       }
       if ( movStatus == 2 ) {
          u8g.drawStr(54,30,"ABORTED");
       }

       u8g.drawStr(0,40,"GoPosI: ");
       u8g.drawStr(74,40,goto_locC);

       u8g.drawStr(0,50,"CuPosI: ");
       u8g.drawStr(74,50,posvalC);
  } while(u8g.nextPage() );
}
#endif
