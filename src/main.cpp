#include <Arduino.h>
#include <LiquidCrystal_I2C.h>


// remote goes on the left side - black antenna




// RRRRRRRRRRRRRRRRRR
// RR               RR
// RR               RR
// RR               RR                                                                                      tt
// RR               RR                                                                                      tt
// RRRRRRRRRRRRRRRRRR                                                                                    ttttttttttt        
// RRRR                         eeeeeeeeeeeeee           mmmmmmmm    mmmmmmmm          oooooooooo           tt              eeeeeeeeeeeeee
// RR  RR                      ee            ee         mmm     mm  mm     mmm        oo        oo          tt             ee            ee
// RR    RR                    ee            ee         mm        mm        mm        oo        oo          tt             ee            ee
// RR      RR                  eeeeeeeeeeeeeeee         mm        mm        mm        oo        oo          tt             eeeeeeeeeeeeeeee
// RR        RR                ee                       mm        mm        mm        oo        oo          tt             ee
// RR          RR              ee                       mm        mm        mm        oo        oo          tt             ee 
// RR            RR             eeeeeeeeeeeeeee         mm        mm        mm         oooooooooo            ttttttt        eeeeeeeeeeeeeee
    


//                 ##
//               ##
//             ##
//           ##
//         ##
//        ######################################
//         ##
//           ##
//             ##
//               ##
//                 ##




///////////////
// LIBRARIES //
///////////////

// libraries do result in some warnings - the program still works

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

///////////////



/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// Where to send packets to!
#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define MY_ADDRESS     2

// radio pin

#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4



#define connectionLed 13


#define redRGBPin 12
#define greenRGBPin 11
#define blueRGBPin 10


#define VBATPIN A7

#define buzzerPin 6


#define rightSwitch 19
//joystick configuration

// joystick 1

int joystick1_x = A1;                                               
int joystick1_y = A0;
float joystick1_x_pos;
float joystick1_y_pos;  

// joystick 2

int joystick2_x = A3;                                               
int joystick2_y = A2;
float joystick2_x_pos;
float joystick2_y_pos;  


// potentiometer configuration

// potentiometer 1

int pot1 = A4;
float pot1_pos;

// potentiometer 2

int pot2 = A5;
float pot2_pos; 


// mode configuration
float mode = 1;


float inputMode = 1;




float tData[7];


int rpmRight;  
int rpmLeft;  

float bareRpmRight;
float bareRpmLeft;

int batteryPercentage;


int connectionStatus = 0;
unsigned long startTime = millis();
unsigned long lastConnection = 0;






uint8_t bell[8]  = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
uint8_t note[8]  = {0x2, 0x3, 0x2, 0xe, 0x1e, 0xc, 0x0};
uint8_t clock[8] = {0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0};
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
uint8_t duck[8]  = {0x0, 0xc, 0x1d, 0xf, 0xf, 0x6, 0x0};
uint8_t check[8] = {0x0, 0x1 ,0x3, 0x16, 0x1c, 0x8, 0x0};
uint8_t cross[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};
uint8_t retarrow[8] = {	0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4};





// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


int16_t packetnum = 0;  // packet counter, we increment per xmission


LiquidCrystal_I2C lcd(0x27, 16, 2);

void rgbColor(int redColor, int greenColor, int blueColor) {
  analogWrite( redRGBPin, redColor);
  analogWrite( greenRGBPin, greenColor);
  analogWrite( blueRGBPin, blueColor);
}

void buzzer(byte PIN, byte delayOn, byte delayOff, byte loops) {
  if ( digitalRead(rightSwitch) != 1){
    // for (byte i=0; i<loops; i++)  {
    //   // digitalWrite(PIN,HIGH);
    //   // delay(delayOn);
    //   // digitalWrite(PIN,LOW);
    //   // delay(delayOff);
    // }
    delay(0);
  }
}
void connectionLedOn() {
  digitalWrite(connectionLed,HIGH);
}

void connectionLedOff() {
  digitalWrite(connectionLed,LOW);
}



void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);



  lcd.begin();
	lcd.createChar(0, bell);
	lcd.createChar(1, note);
	lcd.createChar(2, clock);
	lcd.createChar(3, heart);
	lcd.createChar(4, duck);
	lcd.createChar(5, check);
	lcd.createChar(6, cross);
	lcd.createChar(7, retarrow);


  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");


  pinMode(connectionLed, OUTPUT);

  pinMode(buzzerPin, OUTPUT);

  pinMode(redRGBPin, OUTPUT); 
  pinMode(greenRGBPin, OUTPUT); 
  pinMode(blueRGBPin, OUTPUT);

  pinMode (rightSwitch, INPUT);

  // analog set up

  // joystick 1

  pinMode (joystick1_x, INPUT);                     
  pinMode (joystick1_y, INPUT);

  // joystick 2

  pinMode (joystick2_x, INPUT);                     
  pinMode (joystick2_y, INPUT);

  // potentiometer 1

  pinMode (pot1, INPUT);

  // potentiometer 2

  pinMode (pot2, INPUT);

}


// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

void loop() {

  // reading analog data

  // tData[0] = float(analogRead(joystick1_x))/5.115 ;  
  // tData[1] = float(analogRead (joystick1_y))/5.115  ; 

  // tData[2] = float(analogRead (joystick2_x)) ;  
  // tData[3] = float(analogRead (joystick2_y)) ; 

  // tData[4] = float(analogRead (pot1)) ;

  // tData[5] = float(analogRead (pot2)) ;

  // tData[6] = mode; 

  
  

  if(inputMode == 1){
    bareRpmRight = ((float(analogRead(joystick1_y))/5.115) - 100) * 2 ;
    bareRpmLeft = ((float(analogRead(joystick2_y))/5.115) - 100) * 2 ;

    if(bareRpmRight > -10 && bareRpmRight < 10){rpmRight = 0;}
    else if(bareRpmRight > 197){rpmRight = 200;}
    else{rpmRight = bareRpmRight;}


    if(bareRpmLeft > -10 && bareRpmLeft < 10){rpmLeft = 0;}
    else if(bareRpmLeft > 197){rpmLeft = 200;}
    else{rpmLeft = bareRpmLeft;}
  }


  tData[0] = rpmRight;
  tData[1] = rpmLeft;
  tData[2] = mode;

  Serial.print("Sending: RPM Right: "); 
  Serial.print(tData[0]);
  Serial.print(", RPM Left: ");
  Serial.print(tData[1]);
  Serial.print(", Mode: ");
  if(tData[2] == 1){
    Serial.println("Regular Drive");
  }
  
  


  
  // Send a message to the DESTINATION!
  if (rf69_manager.sendtoWait((uint8_t*)&tData, sizeof(tData), DEST_ADDRESS)) {
    
    
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      buf[len] = 0; // zero out remaining string 
      if(millis() - lastConnection > 3800){
        buzzer(buzzerPin, 120, 150, 2);
      }
      lastConnection = millis();   
    } 
    else {
      lcd.home();
      lcd.clear();
      lcd.setCursor(0,1);
      lcd.print("No reply");
    }
  }
  
  else {
    lcd.home();
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Sending Failed");
    
  }





  // connection status

  if (millis() - lastConnection == 0){
    connectionLedOn();
    lcd.home();
    lcd.clear();
    lcd.setCursor(0,2);
    lcd.print("Connected");
  } 

  else if(millis() - lastConnection < 3500 ){
    lcd.home();
    lcd.clear();
    lcd.setCursor(0,2);
    lcd.print("No Connection");
    connectionLedOff();
    buzzer(buzzerPin, 80,70, 3);
  }
  else{
    connectionLedOff();
  }
  




  // battery status led

  float measuredvbat = analogRead(VBATPIN);

  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  if(measuredvbat > 3.9){
    // rgbColor(0,255,0);
    // rgbColor(255,166,0);
    rgbColor(255,0,0);
  }
  else if(measuredvbat > 3.7){
    rgbColor(255,166,0);
  }
  else if(measuredvbat <= 3.7){
    rgbColor(255,0,0);
  }
  

  
	
  delay(50);


}
