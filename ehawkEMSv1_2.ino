
/*
E-HAWK Avionics
EMS v1.1
Reallocate display pins to free up SPI bus D10->D24, D11->D25 for CAN

Teensy 3.5
Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)
240x128 Graphic LCD CFAG240128B-TFH-TZ (https://www.crystalfontz.com/product/cfag240128btfhtz-graphic-lcd-240x128-display-module)
//https://github.com/coryjfowler/MCP_CAN_lib  
*/





#include <Arduino.h>
#include <U8g2lib.h>
#include <Encoder.h>
#include <ADC.h>
#ifdef U8X8_HAVE_HW_SPI
#include <mcp_can.h>
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif




#define FONT_START  106
#define FONT_WIDTH  16
#define INTERVAL    3000
#define INC 1


/* ************************************************************************** */
/* Global Pin Assignments                                                     */
/* ************************************************************************** */
/*  TEENSY    DISP
 *   
    GND       Vss
    Vin       Vdd
    Wiper     Vo
    D15(A1)   C/D
    Vin       /RD
    D17(A3)   /WR
    D8        DB0
    D9        DB1
    D24       DB2
    D25       DB3
    D4        DB4
    D5        DB5
    D6        DB6
    D7        DB7
    D14(A0)   /CE
    D16(A2)   /RESET
    NegPot    Vee
    GND       FS1
*/

#define BATT_VOLTAGE_PIN   A4
#define AUX_VOLTAGE_PIN   A5
#define CURRENT_PIN       A6
#define ENCODER_BUTTON_PIN
//#define ENCODER_A_PIN       3 // interrupt pins
//#define ENCODER_B_PIN       2
#define RX5               A15 // serial comm with motor controller
#define TX5               A16 // serial comm with motor controller


// #define VREF 3.3
// #define resolution (VREF/((pow(2,16))-1));
const int readPin = A9; // ADC0
const int readPin2 = A16; // ADC1

// CAN bus declaration
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10

// Bitmap startup screen
#define ehawk_bitmap_width 121
#define ehawk_bitmap_height 30
static const unsigned char ehawk_bitmap[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x3f, 0x00,
   0xe0, 0x87, 0x1f, 0xf0, 0x3f, 0xf8, 0xc1, 0x0f, 0x3e, 0xfc, 0xfc, 0x00,
   0x00, 0xfe, 0x3f, 0x00, 0xf0, 0xc3, 0x0f, 0xf8, 0x1f, 0xf8, 0xf0, 0x07,
   0x1f, 0x7e, 0x3f, 0x00, 0x00, 0x7f, 0x00, 0x00, 0xf8, 0xe1, 0x07, 0xbe,
   0x1f, 0xf8, 0xf8, 0xc7, 0x0f, 0xff, 0x0f, 0x00, 0x80, 0x3f, 0x00, 0x00,
   0xfc, 0xf1, 0x03, 0x9f, 0x1f, 0xfc, 0xfc, 0xe7, 0x83, 0xff, 0x03, 0x00,
   0x80, 0xff, 0x07, 0x0f, 0xfc, 0xff, 0x83, 0xff, 0x0f, 0x7c, 0xfe, 0xf3,
   0x81, 0xff, 0x00, 0x00, 0xc0, 0xff, 0x83, 0x0f, 0xfe, 0xff, 0xc1, 0xff,
   0x0f, 0xfc, 0xff, 0xff, 0xc0, 0x3f, 0x00, 0x00, 0xe0, 0x0f, 0x00, 0x00,
   0x7f, 0xfe, 0xe0, 0xff, 0x07, 0xfe, 0xf7, 0x3f, 0xe0, 0x3f, 0x00, 0x00,
   0xf0, 0x07, 0x00, 0x80, 0x1f, 0x7e, 0xf8, 0xe0, 0x07, 0xfe, 0xf3, 0x1f,
   0xf0, 0x7f, 0x00, 0x00, 0xf8, 0x03, 0x00, 0xc0, 0x1f, 0x3f, 0x7c, 0xe0,
   0x07, 0xfe, 0xf1, 0x0f, 0xf8, 0xff, 0x00, 0x00, 0xf8, 0x7f, 0x00, 0xc0,
   0x8f, 0x3f, 0x3e, 0xf0, 0x03, 0x7f, 0xf8, 0x07, 0xf8, 0xfd, 0x00, 0x00,
   0xfc, 0x7f, 0x00, 0xe0, 0xc7, 0x1f, 0x1f, 0xf0, 0x03, 0x3f, 0xf8, 0x01,
   0xfc, 0xfc, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xe0, 0xe0, 0x79, 0xf0, 0x0f,
   0x0e, 0x8e, 0x07, 0xff, 0xc0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf0,
   0xf0, 0x7c, 0xfc, 0x1f, 0x1f, 0xc7, 0xc7, 0xff, 0xf1, 0x7f, 0x00, 0x00,
   0x00, 0x80, 0xff, 0xf0, 0x7c, 0x3e, 0x3e, 0x9f, 0x9f, 0xe7, 0xe3, 0xf3,
   0xf9, 0x7c, 0x00, 0x00, 0x00, 0xc0, 0x7b, 0xf0, 0x1e, 0x1f, 0x9f, 0xcf,
   0x9f, 0xf3, 0xf1, 0x01, 0xfc, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x7f, 0xf0,
   0x0f, 0x8f, 0x8f, 0xc7, 0xff, 0xf1, 0xf8, 0x00, 0xf8, 0x03, 0x00, 0x00,
   0x00, 0xf8, 0x3f, 0xf8, 0x87, 0xcf, 0xc7, 0xe3, 0xfe, 0xf8, 0x7c, 0x00,
   0xe0, 0x0f, 0x00, 0x00, 0x00, 0x3c, 0x3e, 0xf8, 0xc3, 0xc7, 0xe3, 0x73,
   0x7e, 0x7c, 0x3c, 0x3e, 0x8f, 0x0f, 0x00, 0x00, 0x00, 0x1e, 0x3e, 0xf8,
   0xe0, 0xe3, 0xff, 0x39, 0x7e, 0x3e, 0xfe, 0x9f, 0xff, 0x07, 0x00, 0x00,
   0x00, 0x0f, 0x1e, 0x78, 0xf0, 0xe1, 0xff, 0x3c, 0x3e, 0x1f, 0xfe, 0x8f,
   0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// U8g2 Contructor (Picture Loop Page Buffer)

U8G2_T6963_240X128_F_8080 u8g2(U8G2_R0, /*DB0 -> DB7*/ 8, 9, 24, 25, 4, 5, 6, 7, /*enable/wr=*/ A3, /*cs/ce=*/ A0, /*c/d=*/ A1, /*reset=*/ A2); // Connect RD with +5V, FS0 and FS1 with GND


unsigned long previousMillis = 0;
const long interval = 1000; //VSI update interval in ms

const int readPin5 = A5;
ADC *adc = new ADC(); // adc object;

// double adc_value5;

// smoothing declaration
const int numReadings = 10;
int readings[numReadings];      // the readings from the pressure sensor
int readIndex = 0;              // the index of the current reading
long total = 0;                  // the running total
int average = 0;                // the average
int analogPin = A5;
int val = 0;
int raw = 0;
float current_f;
int anglePower = 0;
int angleRPM = 0;
int voltage = 0;
int current = 0;
float power = 0.0;
int rpm = 0;
int mTemp = 0;
int cTemp = 0;
int bTemp = 0;
int requestedPWM, realPWM;
int batteryCap = 99;
int previousVal = 0;
int previousRpm = 0;
float previousPower = 0;

int xCenter = 47;
int yCenter = 45;
int xCenter2 = 188;
int yCenter2 = 45;
int r = 40;
int rIn = 35;
bool flag = true;
bool initFail = false;

void display() {
  flag = false;
  u8g2.clearBuffer();

  // battery capacity numeric % ********************************
  u8g2.setFont(u8g2_font_tenthinguys_tr);
  if(batteryCap < 10) {
  u8g2.setCursor(116, 18);
  }
  if(batteryCap >= 10 && batteryCap < 20) {
    u8g2.setCursor(110, 18);
  }
  if(batteryCap >= 20 && batteryCap < 100) {
    u8g2.setCursor(106, 18);
  }
  if (batteryCap == 100) {
    u8g2.setCursor(102, 18);
  }
  u8g2.print(batteryCap);
  // end battery capacity numeric % ****************************


  // rpm numerical display ***********************************************
  u8g2.setFont(u8g2_font_tenfatguys_tr);
  if (rpm >=0 && rpm <=9) {
  u8g2.setCursor(xCenter-3, yCenter-10);
  }
  if (rpm >=10 && rpm <=99) {
  u8g2.setCursor(xCenter-11, yCenter-10);
  }
  if (rpm >=100 && rpm <=999) {
  u8g2.setCursor(xCenter-16, yCenter-10);
  }
  if (rpm >=1000 && rpm <=3750) {
  u8g2.setCursor(xCenter-22, yCenter-10);
  }
  u8g2.print(rpm);
  // end rpm numerical display *******************************************

// voltage numerical display ***********************************************  

  u8g2.setFont(u8g2_font_tenfatguys_tr);
  if (voltage >=0 && voltage <=9)
  u8g2.setCursor(118, 90);
  if (voltage >=10 && voltage <=99)
  u8g2.setCursor(107, 90);
  if (voltage >=100)
  u8g2.setCursor(96, 90);
  u8g2.print(voltage); 
// end voltage numerical display *******************************************

// current numerical display ***********************************************  
  u8g2.setFont(u8g2_font_tenfatguys_tr);
  if (current >=0 && current <=9)
  u8g2.setCursor(118, 105);
  if (current >=10 && current <=99)
  u8g2.setCursor(107, 105);
  if (current >=100)
  u8g2.setCursor(96, 105);
  u8g2.print(current); 
// end current numerical display *******************************************
  

// power numerical display ***********************************************
  u8g2.setFont(u8g2_font_tenfatguys_tr);
  if (power >=0 && power <=9) {
  u8g2.setCursor(xCenter-3, yCenter-10);
  }
  u8g2.setCursor(xCenter2-13, yCenter2-10);
  u8g2.print((float)power, 1);  
// end power numerical display *******************************************

// motor temp display ***********************************************  
  u8g2.setFont(u8g2_font_koleeko_tn);
  if (mTemp >=0 && mTemp <=9)
  u8g2.setCursor(22, 73);
  if (mTemp >=10 && mTemp <=19)
  u8g2.setCursor(18, 73);
  if (mTemp >=20 && mTemp <= 99)
  u8g2.setCursor(14, 73);
  u8g2.print(mTemp); 
  u8g2.setFont(u8g2_font_t0_14b_tf);
  u8g2.drawStr(28,73,"\xb0"); //degree symbol, font must end in 'e' or 'f'
// bar graph  
  u8g2.setDrawColor(1);
  u8g2.drawBox(15, 75+(40-(mTemp/2.5)), 13, mTemp/2.5);  
// end motor temp numerical display *******************************************

// controller temp display ***********************************************  
  u8g2.setFont(u8g2_font_koleeko_tn);
  if (cTemp >=0 && cTemp <=9)
  u8g2.setCursor(48, 73);
  if (cTemp >=10 && cTemp <=19)
  u8g2.setCursor(44, 73);
  if (cTemp >=20 && cTemp <= 99)
  u8g2.setCursor(40, 73);
  u8g2.print(cTemp);
  u8g2.setFont(u8g2_font_t0_14b_tf);
  u8g2.drawStr(53,73,"\xb0"); 
  // bar graph  
  u8g2.setDrawColor(1);
  u8g2.drawBox(41, 75+(40-(cTemp/2.5)), 13, cTemp/2.5); 
// end controller temp numerical display *******************************************

// battery temp display ***********************************************  
  u8g2.setFont(u8g2_font_koleeko_tn);
  if (bTemp >=0 && bTemp <=9)
  u8g2.setCursor(74, 73);
  if (bTemp >=10 && bTemp <=19)
  u8g2.setCursor(70, 73);
  if (bTemp >=20 && bTemp <= 99)
  u8g2.setCursor(66, 73);
  u8g2.print(bTemp);
  u8g2.setFont(u8g2_font_t0_14b_tf);
  u8g2.drawStr(79,73,"\xb0");
  // bar graph  
  u8g2.setDrawColor(1);
  u8g2.drawBox(67, 75+(40-(bTemp/2.5)), 13, bTemp/2.5); 
// end battery temp numerical display *******************************************

// requested PWM display ***********************************************  
  u8g2.setFont(u8g2_font_koleeko_tn);
  if (requestedPWM >=0 && requestedPWM <=9)
  u8g2.setCursor(180, 73);
  if (requestedPWM >=10 && requestedPWM <=19)
  u8g2.setCursor(180, 73);
  if (requestedPWM >=20 && requestedPWM <= 99)
  u8g2.setCursor(180, 73);
  u8g2.print(requestedPWM);
  u8g2.setFont(u8g2_font_t0_14b_tf);
  u8g2.drawStr(195,73,"\x25");  //percent symbol
// end requested PWM display *******************************************

// real PWM display ***********************************************  
  u8g2.setFont(u8g2_font_koleeko_tn);
  if (realPWM >=0 && realPWM <=9)
  u8g2.setCursor(180, 90);
  if (realPWM >=10 && realPWM <=19)
  u8g2.setCursor(180, 90);
  if (realPWM >=20 && realPWM <= 99)
  u8g2.setCursor(180, 90);
  u8g2.print(requestedPWM);
  u8g2.setFont(u8g2_font_t0_14b_tf);
  u8g2.drawStr(195,90,"\x25");  //percent symbol
// end real PWM display *******************************************


  // battery capacity bar graph ********************************
  u8g2.setDrawColor(1);
  u8g2.drawBox(107, 20+(50-(batteryCap/2)), 23, batteryCap/2);
  // end battery capacity bar graph ****************************

  // rmp gauge *************************************************
  u8g2.drawCircle(xCenter, yCenter, r, U8G2_DRAW_UPPER_RIGHT|U8G2_DRAW_UPPER_LEFT);
  u8g2.drawCircle(xCenter, yCenter, r-6, U8G2_DRAW_UPPER_RIGHT|U8G2_DRAW_UPPER_LEFT);

  int xCoord1 = xCenter - r * (cos(angleRPM/57.296));
  int yCoord1 = yCenter - r * (sin(angleRPM/57.296));
  int xCoord2 = (xCenter) - (r-6) * (cos((angleRPM-6)/57.296));
  int yCoord2 = (yCenter) - (r-6) * (sin((angleRPM-6)/57.296));
  int xCoord3 = (xCenter) - (r-6) * (cos((angleRPM+6)/57.296));
  int yCoord3 = (yCenter) - (r-6) * (sin((angleRPM+6)/57.296));

  int xIn = xCenter - rIn * (cos(angleRPM/57.296));
  int yIn = yCenter - rIn * (sin(angleRPM/57.296));

  u8g2.drawTriangle(xCoord1, yCoord1, xCoord2, yCoord2, xCoord3, yCoord3);
  //end rpm gauge ***********************************************

  // power gauge *************************************************
  u8g2.drawCircle(xCenter2, yCenter2, r, U8G2_DRAW_UPPER_RIGHT|U8G2_DRAW_UPPER_LEFT);
  u8g2.drawCircle(xCenter2, yCenter2, r-6, U8G2_DRAW_UPPER_RIGHT|U8G2_DRAW_UPPER_LEFT);

  int xCoord1_2 = xCenter2 - r * (cos(anglePower/57.296));
  int yCoord1_2 = yCenter2 - r * (sin(anglePower/57.296));
  int xCoord2_2 = (xCenter2) - (r-6) * (cos((anglePower-6)/57.296));
  int yCoord2_2 = (yCenter2) - (r-6) * (sin((anglePower-6)/57.296));
  int xCoord3_2 = (xCenter2) - (r-6) * (cos((anglePower+6)/57.296));
  int yCoord3_2 = (yCenter2) - (r-6) * (sin((anglePower+6)/57.296));

  int xIn_2 = xCenter2 - rIn * (cos(anglePower/57.296));
  int yIn_2 = yCenter2 - rIn * (sin(anglePower/57.296));

  u8g2.drawTriangle(xCoord1_2, yCoord1_2, xCoord2_2, yCoord2_2, xCoord3_2, yCoord3_2);
  //end power gauge ***********************************************
   
}

void setup(void) {
  Serial.begin(115200);
  u8g2.begin();

  u8g2.firstPage();
 do {
     u8g2.drawXBMP( 3, 6, ehawk_bitmap_width, ehawk_bitmap_height, ehawk_bitmap);
     u8g2.setFont(u8g2_font_7x14_mr);
     u8g2.setCursor(0, 54);
     u8g2.print(F("EIS v1.2"));
     u8g2.setCursor(0, 74);
     // CAN bus setup
     if(CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK)
        u8g2.print(F("MCP2515 Initialized Successfully!"));
      else {
        u8g2.print(F("Error Initializing MCP2515..."));
        initFail = true;
      }
        
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);                     // Configuring pin for /INT input 
         
    } while( u8g2.nextPage() );
 while(initFail);
 delay(1500);
 u8g2.clearBuffer();
 u8g2.sendBuffer();
 
}

void loop(void) {
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    if((rxId & 0x1FFFFFFF) == 0x14A10002){  //Packet 1, 7 data bytes, (data[0]+256*data[1])/57.45=voltage
      voltage = (rxBuf[0] + 256*rxBuf[1])/57.45;
      current = (rxBuf[2] + 256*rxBuf[3])/10;
      rpm = (rxBuf[4] + 256*rxBuf[5] + 65536*rxBuf[6])*10;
    }

    if((rxId & 0x1FFFFFFF) == 0x14A10003){  //Packet 2, 7 data bytes, (data[4]=cTemp, data[5]=mTemp, data[6]=bTemp)
      cTemp = (rxBuf[4]);
      mTemp = (rxBuf[5]);
      bTemp = (rxBuf[6]);
    }

    if((rxId & 0x1FFFFFFF) == 0x14A10004){  //Packet 3, 8 data bytes
      requestedPWM = (rxBuf[0]+256*rxBuf[1])/10;
      realPWM = (rxBuf[2]+256*rxBuf[3])/10;
    } 

    if((rxId & 0x1FFFFFFF) == 0x14A10005){  //Packet 4, 8 data bytes
      batteryCap = (rxBuf[0]+256*rxBuf[1]); //remaining battery capacity      
    }
     
  }

  angleRPM = constrain(map(rpm, 0, 3750, 0, 180), 0, 180);
  power = (float(voltage * current/1000.0));
  anglePower = constrain(map(power, 0, 25, 0, 180), 0, 180);
  val = constrain(map(val, 25, 4095, 0, 100), 0, 100);
  
// draw frame for battery capacity  
  u8g2.setDrawColor(1);
  u8g2.drawFrame(106,20,25,50);
  u8g2.drawFrame(105, 19, 27, 52);
//  u8g2.setFont(u8g2_font_7x14_mr);

// draw frame for motor temp
  u8g2.setDrawColor(1);
  u8g2.drawFrame(14,75,14,40);

// draw frame for controller temp
  u8g2.setDrawColor(1);
  u8g2.drawFrame(40,75,14,40);

// draw frame for battery temp
  u8g2.setDrawColor(1);
  u8g2.drawFrame(66,75,14,40);  

  
  u8g2.setFont(u8g2_font_tenthinguys_tr);
  u8g2.drawStr(125, 18,"%");
  u8g2.drawStr(xCenter-13, yCenter+5,"RPM");
  u8g2.drawStr(xCenter2-8, yCenter2+5,"kW");
  u8g2.drawStr(130, 90,"V");
  u8g2.drawStr(130, 105,"A");
  u8g2.sendBuffer();

//  if(previousRpm != rpm || previousPower != current_f || flag == true) {
  display();

}

