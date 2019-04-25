#include <SPI.h>
#include "src/MFRC522.h"
#include "channels.h"
#include "IRLine.h"
#include "proj_types.h"


 
byte UsingSimulator;

channels_t serial_channels;
byte go;

// RFID pins
#define SS_PIN  7
#define RST_PIN 8
#define time_turn_r 600
#define time_turn_l 600
 
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class

MFRC522::MIFARE_Key key; 

// Init array that will store new NUID 
byte nuidPICC[4];

IRLine_t IRLine;

// Motor Pins
#define MOTOR_AIN1 6
#define MOTOR_AIN2 5
#define MOTOR_PWMA 9

  
#define MOTOR_BIN1 4
#define MOTOR_BIN2 3
#define MOTOR_PWMB 10

// Solenoid Pins
#define SOLENOID 2

// Touch Switch
#define TOUCHSW 11

byte RFID_read;

void setSolenoidState(byte state)
{
  digitalWrite(SOLENOID, state);
}

byte readTouchSwicth(void)
{
  byte ret;
  pinMode(TOUCHSW, INPUT_PULLUP);
  ret = !digitalRead(TOUCHSW);
  pinMode(TOUCHSW, OUTPUT);
  return ret;
}

void setMotorsVoltage(int v1, int v2)
{
  if (v1 >= 0) {
    digitalWrite(MOTOR_AIN1, 1);
    digitalWrite(MOTOR_AIN2, 0);
    if (v1 > 255) v1 = 255;
    analogWrite(MOTOR_PWMA, v1);
  } else {
    digitalWrite(MOTOR_AIN1, 0);
    digitalWrite(MOTOR_AIN2, 1);
    v1 = - v1;
    if (v1 > 255) v1 = 255;
    analogWrite(MOTOR_PWMA, v1);
  }

  if (v2 >= 0) {
    digitalWrite(MOTOR_BIN1, 1);
    digitalWrite(MOTOR_BIN2, 0);
    if (v2 > 255) v2 = 255;
    analogWrite(MOTOR_PWMB, v2);
  } else {
    digitalWrite(MOTOR_BIN1, 0);
    digitalWrite(MOTOR_BIN2, 1);
    v2 = - v2;
    if (v2 > 255) v2 = 255;
    analogWrite(MOTOR_PWMB, v2);
  }
  
}


void serial_write(uint8_t b)
{
  Serial.write(b);
}



void setup() 
{
  // Motors I/O Setup
  digitalWrite(MOTOR_AIN1, 0);
  digitalWrite(MOTOR_AIN2, 0);
  digitalWrite(MOTOR_PWMA, 0);
  digitalWrite(MOTOR_BIN1, 0);
  digitalWrite(MOTOR_BIN2, 0);
  digitalWrite(MOTOR_PWMB, 0);
  
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_BIN1, OUTPUT);
  pinMode(MOTOR_BIN2, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);

  // Solenoid I/O Setup
  digitalWrite(SOLENOID, 0);
  pinMode(SOLENOID, OUTPUT);

  // Faster ADC - http://forum.arduino.cc/index.php/topic,6549.0.html
  // set prescaler to 16
  // sbi(ADCSRA,ADPS2); // cbi(ADCSRA,ADPS1); // cbi(ADCSRA,ADPS0);
  ADCSRA = (ADCSRA | (1 << ADPS2)) & ~((1 << ADPS1) | (1 << ADPS0));          

  
  Serial.begin(115200);
  serial_channels.init(process_serial_packet, serial_write);
  
  // RFID setup
  SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522 

  UsingSimulator = 1;

}

void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}



void readRFID(void) 
{
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);

  // Look for new cards

  //if (rfid.PICC_IsNewCardPresent()) {
  if (rfid.PICC_WakeupA(bufferATQA, &bufferSize) == 0) {
    // Verify if the NUID has been readed
    if (rfid.PICC_ReadCardSerial()) {
      
      Serial.print(F("PICC type: "));
      MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
      Serial.print(rfid.PICC_GetTypeName(piccType));

      // Store NUID into nuidPICC array
      for (byte i = 0; i < 4; i++) {
        nuidPICC[i] = rfid.uid.uidByte[i];
      }
      
      Serial.println(F("The NUID tag is:"));
      Serial.print(F("In hex: "));
      printHex(rfid.uid.uidByte, rfid.uid.size);
      
      // Halt PICC
      rfid.PICC_HaltA();
      
      // Stop encryption on PCD
      rfid.PCD_StopCrypto1();
    }
  } 
}  



void readIRSensors(void)
{
  byte c;
  for (c = 0; c < 5; c++) {
    IRLine.IR_values[c] = 1023 - analogRead(A0 + 4 - c);
  }      
}

uint32_t tis;
uint32_t current, previous, interval = 40000UL;
uint8_t cont_ini=0, RFID=1, MA1=0, aux=0, MA2=0, cont_arm=0;

robot_t robot;

byte TouchSwitch, LastTouchSwitch;


void process_serial_packet(char channel, uint32_t value, channels_t& obj)
{
 byte c;


 if (channel == 'r') {           // RFID tag
   for (c = 0; c < 4; c++) {
     nuidPICC[c] = (value >> (c * 8)) & 0xFF;
   }
   
 } else if (channel == 'i')  {   // IR Sensors + Touch
   for (c = 0; c < 5; c++) {
    IRLine.IR_values[c] = 16 * ((value >> (c * 6)) & 0x3F);
   } 
   TouchSwitch = ((value >> 31) & 1);  
 
 } else if (channel == 'g')  {  // Control
  // Calc control
  go = 1;
 } else if (channel == 's')  {  // Set new state
  robot.state = value;

 } else if (channel == 'p')  { // Ping
   obj.send(channel, value + 1);
   Serial.println(value + 1);
 }
}

void followLineRight(float Vnom, float K)
{
  robot.v = Vnom;
  robot.w = K * IRLine.pos_right;
}


void followLineLeft(float Vnom, float K)
{
  robot.v = Vnom;
  robot.w = K * IRLine.pos_left;
}

void followLine(float Vnom, float K)
{
  float pos;
  
  if (fabs(IRLine.pos_left) < fabs(IRLine.pos_right)) {
    pos = IRLine.pos_left;
  } else {
    pos = IRLine.pos_right;
  }
  
  robot.v = Vnom;
  robot.w = K * pos;
}


void moveRobot(float Vnom, float Wnom)
{
  robot.v = Vnom;
  robot.w = Wnom;
}


void setState(byte new_state)
{
  tis = 0;
  robot.state = new_state;
  IRLine.crosses=0;
}

#define Varm 30
#define Vmed 100
#define kapa -4.0

void control(void)
{
  
  
    if (robot.state==1 && TouchSwitch && cont_ini==0){ //avança (pela esquerda) até detetor fim de curso
      setState(2);
      
    }
    else if (robot.state==1 && cont_ini>0){  
      setState(9);
      
    }
    else if (robot.state==2 && IRLine.crosses >=1){  //recua ate detetar a direita
      if (RFID==1) {setState(3); cont_ini++;}
      else if(RFID==0) {setState(19); cont_ini++;}
    }
    else if(robot.state == 3 && tis > 1600 && IRLine.total > 1500) { //vira 180 e avança pela esquerda
      if(MA1==0) setState(4);
      else if(MA1==1) setState(28);
     }
     else if(robot.state == 4 && IRLine.crosses>=1 && tis>4000) {  
      setState(6);
     }
     else if(robot.state == 6 && tis > 1600) { //avança durante alguns segundos para deixar a peça na maquina, incrementa MA1
      setState(7);
     }
     else if(robot.state == 7 && IRLine.crosses >= 1) { //recua ate chegar ao cruzamento para a maquina 
      MA1=1;
      setState(8);
     }  
     else if(robot.state == 8 && tis>1000) { //segue pela esquerda
      setState(1);
     } 
     else if(robot.state == 9 && (cont_ini+1)== IRLine.crosses) {
      setState(10);
      cont_ini++;
     }   
      else if(robot.state == 10 && TouchSwitch) { 
      setState(11);  
     }
      else if(robot.state == 11 && IRLine.crosses>=1 && RFID==0) { 
      setState(12);  
     }
     else if(robot.state == 12 && IRLine.crosses>=(5-(cont_ini-1))) { 
      setState(13);  
     }
     else if(robot.state == 13 && tis > 1100) { 
      setState(14);  
     }
     else if(robot.state == 14 && IRLine.crosses>=(3+cont_ini-1)) { 
      setState(15);  
     }
     else if(robot.state == 15 && tis > 300) { 
      setState(16);  
     }
     else if(robot.state == 16 && tis > 750 && IRLine.total > 1500) { 
      setState(21);  
     }
     else if(robot.state == 11 && IRLine.crosses>=1 && RFID==1) { 
      setState(17);  
     }
     else if(robot.state == 17 && IRLine.crosses>=cont_ini && MA1==0) { 
      setState(6);  
     }
      else if(robot.state == 17 && IRLine.crosses>=cont_ini && MA1==1) { 
      setState(18);  
     }
      else if(robot.state == 18 && IRLine.crosses>=2) { 
      setState(29);  
     }
      else if(robot.state == 28 && IRLine.crosses>=2) { 
      setState(29);  
     }
      else if(robot.state == 29 && tis > 1600) { 
      setState(30);  
     }
      else if(robot.state == 30 && IRLine.crosses>=1) { 
      MA2=1;
      setState(31);  
     }
      else if(robot.state == 31 && tis > time_turn_r) { 
      setState(32);  
     }
      else if(robot.state == 32 && IRLine.crosses>=3 && MA2==1) { 
      setState(33);  
     }
      else if(robot.state == 33 && tis > 1600) { 
      MA2=0;
      setState(34);  
     }
      else if(robot.state == 34 && IRLine.crosses>=1) { 
      setState(35);  
     }
      else if(robot.state == 35 && IRLine.crosses>=(2+cont_arm)) { 
      setState(39);  
     }
      else if(robot.state == 39 && tis > 1600) { 
      cont_arm++;
      setState(40);  
     }
      else if(robot.state == 40 && IRLine.crosses>=1) { 
      setState(41);  
     }
      else if(robot.state == 41 && IRLine.crosses>=cont_arm) { 
        if(MA1==1) setState(42);
        else if(MA1==0) setState(24);   
     }
     else if(robot.state == 42 && IRLine.crosses>=2){
      setState(36);
     }
//     else if(robot.state == 43 && IRLine.crosses>=1){
//      setState(44);
//     }
//     else if(robot.state == 44 && tis > time_turn_r){
//      setState(45);
//     }
//     else if(robot.state == 45 && IRLine.crosses>=3){
//      setState(1);
//     }
     else if(robot.state == 32 && IRLine.crosses>=4 && MA1==1 && MA2==0){
      setState(36);
     }
     else if(robot.state == 36 && tis > 1600){
      MA1=0;
      setState(37);
     }
     else if(robot.state == 37 && IRLine.crosses>=1){
      setState(38);
     }
     else if(robot.state == 38 && IRLine.crosses>=(3+cont_arm)){
      setState(39);
     }
     else if(robot.state == 19 && tis > 1600 && IRLine.total > 1500){
      setState(20);
     }
     else if(robot.state == 20 && IRLine.crosses>=5){
      setState(21);
     }
     else if(robot.state == 21 && ((cont_arm==0 && tis > 3000) || (cont_arm > 0 && tis > 1700))){
      setState(22);
     }
     else if(robot.state == 22 && IRLine.crosses>=1){
      cont_arm++;
      setState(23);
     }
     else if(robot.state == 23 && IRLine.crosses>=cont_arm){
      if(MA1==1 || MA2==1) setState(27);
      else if(MA1==0 && MA2==0) setState(24);
     }
     else if(robot.state == 27 && IRLine.crosses>=2 && MA1==1 && MA2==0){
      setState(36);
     }
     else if(robot.state == 27 && IRLine.crosses>=1 && MA2==1){
      setState(33);
     }
     else if(robot.state == 24 && IRLine.crosses>=1){
      setState(25);
     }
     else if(robot.state == 25 && tis > 1100){
      setState(26);
     }
     else if(robot.state == 26 && IRLine.crosses>=(6-cont_ini)){
      setState(43);
     }
     else if(robot.state == 43 && TouchSwitch){
      setState(11);
     }
     
     

//      if (robot.state == 1 && TouchSwitch) {
//      setState(2);
//    } else if(robot.state == 2 && tis > 400) { //agarra a caixa
//      setState(3);
//    } else if(robot.state == 3 && tis > 1600) { //recua
//      setState(4);
//    } else if(robot.state == 4 && tis > 1600 && IRLine.total > 1500) {
//      setState(5);
//      IRLine.crosses = 0;
//    } else if(robot.state == 5 && IRLine.crosses >= 5) {
//      setState(6);
//    } else if(robot.state == 6 && tis > 1500 && IRLine.total > 1500) {
//      setState(7);
//    } else if(robot.state == 7 && tis > 2000) {
//      IRLine.crosses = 0;
//      setState(8);
//    }




    if (robot.state == 0) {         // Robot Stoped            
      robot.solenoid_state = 0;
      moveRobot(0, 0);
    
    } else if (robot.state == 1) {  // Go: Get first box
      robot.solenoid_state = 1;
      followLineLeft(Vmed, kapa);   
    } else if (robot.state == 2) {  // Go back with the box 23w
      robot.solenoid_state = 1;
      moveRobot(-60, 0);   
    } else if (robot.state == 3 || robot.state == 19) {  // Turn 180 degrees
      robot.solenoid_state = 1;
      moveRobot(0, 50);
    } else if (robot.state == 4) {  // segue pela esquerda com iman
      robot.solenoid_state = 1;
      followLineLeft(Vmed, kapa);      
    } else if (robot.state == 6) {  // avança durante alguns segundos 
      robot.solenoid_state = 1;
      moveRobot(60, 0); 
    } else if (robot.state == 7 || robot.state == 22 || robot.state == 30 || robot.state == 40) {  //recua sem iman
      robot.solenoid_state = 0;
      moveRobot(-60, 0);
    } else if (robot.state == 8 || robot.state == 26 || robot.state == 32) { //segue a linha pela esquerda sem iman
      robot.solenoid_state = 0;
      followLineLeft(Vmed, kapa);
    } else if (robot.state == 9 || robot.state == 23 || robot.state == 27 || robot.state == 24 || robot.state == 31 || robot.state == 41 || robot.state == 42) { //segue a linha pela direita sem iman
      robot.solenoid_state = 0;
      followLineRight(Vmed, kapa);
    } else if(robot.state ==10){
      robot.solenoid_state = 1;
      followLineLeft(Vmed, kapa);
    } else if(robot.state ==11 || robot.state == 34 || robot.state == 37){ //recuar com iman
      robot.solenoid_state = 1;
      moveRobot(-60, 0);
    } else if(robot.state == 12 || robot.state == 39 || robot.state == 43){ //direita com iman
      robot.solenoid_state = 1;
      followLineRight(Vmed, kapa);
    } else if(robot.state == 13 || robot.state == 15){
      robot.solenoid_state = 1;
      moveRobot(Vmed, 0);
    } else if(robot.state == 14){
      robot.solenoid_state = 1;
      followLineLeft(Vmed-40*(IRLine.crosses==6), kapa);
    } else if(robot.state == 16){
      robot.solenoid_state = 1;
      moveRobot(0, -50);
    } else if(robot.state == 17 || robot.state == 29){
      robot.solenoid_state = 1;
      followLineLeft(Vmed, kapa);
    } else if(robot.state == 18 || robot.state == 20 || robot.state == 21 || robot.state == 28 || robot.state == 33 || robot.state == 35 || robot.state == 36 || robot.state == 38){ //esquerda com iman
      robot.solenoid_state = 1;
      followLineRight(Vmed, kapa);
    } else if(robot.state == 25){
      robot.solenoid_state = 0;
      moveRobot(Vmed, 0);
    } 
    
}

void loop(void)
{
  if (UsingSimulator) {
    sim_loop();
  } else {
    real_loop();
  }
}


void sim_loop(void)
{
  byte b;
  if (Serial.available()) {
    b = Serial.read();
    serial_channels.StateMachine(b);
  }

  if (go) {
    tis = tis + interval / 1000;
    IRLine.calcIRLineEdgeLeft();
    IRLine.calcIRLineEdgeRight();
    IRLine.calcCrosses();
    control();
    go = 0;

    serial_channels.send('S',  robot.state);
    serial_channels.send('V',  round(robot.v * 1000));
    serial_channels.send('W',  round(robot.w * 1000));
    serial_channels.send('M',  robot.solenoid_state);
    serial_channels.send('X',  IRLine.crosses);
    //serial_channels.send('Y',  IRLine.total);
    //serial_channels.send('Z',  IRLine.cross_count);

    

  }
}


void real_loop(void) 
{  
  uint32_t t;
  byte b;
  if (Serial.available()) {
    b = Serial.read();
    if (b == '+') robot.solenoid_state = 1; //setSolenoidState(1);
    if (b == '-') robot.solenoid_state = 0; //setSolenoidState(0);
    if (b == '(') {robot.v = 50; robot.w =  0;} //setMotorsVoltage(50, 50) ;
    if (b == '/') {robot.v =  0; robot.w = 50;} //setMotorsVoltage(-200, 200) ;
    if (b == '=') {robot.v =  0; robot.w =-50;} //setMotorsVoltage(200, -200) ;
    if (b == ')') {robot.v =-50; robot.w = 0;} //setMotorsVoltage(-200, -200) ;
    if (b == '?') {robot.v =  0; robot.w = 0;} //setMotorsVoltage(0, 0) ;
    if (b == '\\') robot.state = 0;
    if (b == '*') robot.state = 1;
    if (b == '!') UsingSimulator = 0;
    if (b == '#') UsingSimulator = 1;
    serial_channels.StateMachine(b);
  }

  current = micros();
  if (current - previous >= interval) {
    previous = current;
    tis = tis + interval / 1000;
    
    t = micros();
    readRFID();
    t = micros() - t;

    t = micros();
    readIRSensors();
    t = micros() - t;

    LastTouchSwitch = TouchSwitch;
    TouchSwitch = readTouchSwicth();
    if (robot.state == 0 && LastTouchSwitch && !TouchSwitch) robot.state = 1;

    IRLine.calcIRLineEdgeLeft();
    IRLine.calcIRLineEdgeRight();
    IRLine.calcCrosses();

    control();
    
    setSolenoidState(robot.solenoid_state);
    setMotorsVoltage(robot.v + robot.w, robot.v - robot.w);
    
    byte c;
    for (c = 0; c < 5; c++) {
       Serial.print(IRLine.IR_values[c]);
       Serial.print(" ");
    }
    Serial.print(" PosR: ");
    Serial.print(IRLine.pos_right);

    Serial.print(" PosL: ");
    Serial.print(IRLine.pos_left);

    Serial.print(" Touch: ");
    Serial.print(TouchSwitch);
    
    Serial.print(" state: ");
    Serial.print(robot.state);

    Serial.print(" tis: ");
    Serial.print(tis);
     
    Serial.println();
  }
  
}
