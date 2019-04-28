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

void setSolenoidState(byte state)
{
  digitalWrite(SOLENOID, state);
}

byte readTouchSwicth(void)
{
  byte ret;
  pinMode(TOUCHSW, INPUT_PULLUP);
  ret = digitalRead(TOUCHSW);
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

  UsingSimulator = 0;
  
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

void followLineRight(float Vnom, float K, float Ke)
{
  int Vrot=Vnom+Ke*fabs(IRLine.pos_right);
  if(Vrot>0) Vnom=Vrot; 
  else if(Vrot<=0 || fabs(IRLine.pos_right)==32) Vnom=0;
  robot.v = Vnom;
  robot.w = K * IRLine.pos_right;
}


void followLineLeft(float Vnom, float K, float Ke)
{
  int Vrot=Vnom+Ke*fabs(IRLine.pos_left);
  if(Vrot>0) Vnom=Vrot; 
  else if(Vrot<=0 || fabs(IRLine.pos_left)==32) Vnom=0;
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
uint8_t cont_arm=0;
#define Vmed 80
#define Varm 60
#define kapa -2.0 
#define kaparm -1.0
#define ke -2.3 
#define time_cruz 650
void control(void)
{
    /*if(robot.state==0 && TouchSwitch) setState(1);
    if (robot.state == 1 && TouchSwitch && tis>500) {
      setState(2);
    } else if(robot.state == 2 && tis > 2500) {
      setState(3);
    } else if(robot.state == 3 && tis>500) setState(4);
    else if(robot.state == 4 && tis>500) setState(0);

    if (robot.state == 0) {         // Robot Stoped            
      robot.solenoid_state = 0;
      moveRobot(0, 0);
    
    } else if (robot.state == 1) {  // Go: Get first box
      robot.solenoid_state = 0;
      moveRobot(60, 0);

    } else if (robot.state == 2) { // Turn Solenoid On and Get the Box
      robot.solenoid_state = 1;
      moveRobot(0, -50);
    
    } else if (robot.state == 3) {  // Go back with the box
      robot.solenoid_state = 1;
      moveRobot(-60, 0);
      
    } else if (robot.state == 4) {  // Turn 180 degrees
      robot.solenoid_state = 1;
      moveRobot(0, 50);
    } */
      //esta com problemas em contar cruzamentos
      
      if(robot.state==0 && TouchSwitch) setState(1);
      else if (robot.state==1 && TouchSwitch && tis>500) setState(17);
      else if (robot.state==17 && tis>500) setState(2);
      else if (robot.state==2 && tis>1000) setState(3); //volta para trás até detetar a direita
      else if (robot.state==3 && tis > 1500 && IRLine.total > 1500) setState(4); //estar centrado
      else if (robot.state==4 && (IRLine.crosses>=5||tis>13000)) setState(5); //speed até à curva
      else if (robot.state==5 && (tis>3000 ||tis>10000)) setState(6); //entrar no armazem
      else if (robot.state==6 && IRLine.crosses==1){  //recua até detetar o cruzamento
        setState(7); 
        cont_arm++;
      } else if (robot.state==7 && cont_arm==4) setState(0); //termino
      else if (robot.state==7 && IRLine.crosses==(1+cont_arm) && cont_arm<4) setState(8); //antes do cruzamento
      else if (robot.state==8 && tis>=time_cruz) setState(9); //cruzamento
      else if (robot.state==9 && IRLine.crosses==(6-cont_arm)) setState(10);
      else if (robot.state==10 && TouchSwitch) setState(18);
      else if (robot.state==18 && tis>500) setState(11);
      else if (robot.state==11 && IRLine.crosses==1) setState(12);
      else if (robot.state==12 && IRLine.crosses==(5-cont_arm) && cont_arm<=2) setState(13); //andar até o cruzamento 
      else if (robot.state==12 && IRLine.crosses==(4-cont_arm) && cont_arm==3) setState(15);
      else if (robot.state==13 && tis>=time_cruz) setState(14); //ultrapassar o cruzamento
      else if (robot.state==14 && IRLine.crosses==3+cont_arm) setState(5); //andar até ao armazem certo | quando cont_arm=3 posso passar state=5 quando crosses==5


      
      
      
      
      if (robot.state == 0) {         // Robot Stoped            
        robot.solenoid_state = 0;
        moveRobot(0, 0);
      } else if (robot.state == 1||robot.state==17) {
        robot.solenoid_state = 1;
        followLineLeft(Vmed, kapa, ke);
      } else if (robot.state == 2 || robot.state==11) {
        robot.solenoid_state = 1;
        moveRobot(-Vmed+20, 0);
      } else if (robot.state == 3) {
        robot.solenoid_state = 1;
        moveRobot(0, 50); 
      } else if (robot.state == 4) {
        robot.solenoid_state = 1;
        followLineRight(Vmed-10*(IRLine.crosses>=4), kapa, ke);
      } else if (robot.state == 5) {
        robot.solenoid_state = 1;
        followLineRight(Vmed-20, kapa,ke);
      } else if (robot.state == 6) {
        robot.solenoid_state = 0;
        moveRobot(-Vmed+20, 0);
      } else if (robot.state == 7) {
        robot.solenoid_state = 0;
        followLineRight(Vmed, kapa,ke);
      } else if (robot.state == 8) {
        robot.solenoid_state = 0;
        moveRobot(Vmed, 0);
      } else if (robot.state == 9) {
        robot.solenoid_state = 0;
        followLineLeft(Vmed, kapa,ke);
      } else if (robot.state == 10||robot.state==18) {
        robot.solenoid_state = 1;
        followLineRight(Vmed, kapa,ke);
      } else if (robot.state == 12) {
        robot.solenoid_state = 1;
        followLineRight(Vmed, kapa,ke);
      } else if (robot.state == 13 ) {
        robot.solenoid_state = 1;
        moveRobot(Vmed, 0);
      } else if (robot.state == 14) {
        robot.solenoid_state = 1;
        followLineLeft(Vmed-10*(IRLine.crosses>=2+cont_arm), kapa,ke);
      } else if(robot.state==16){
        robot.solenoid_state = 1;
        moveRobot(0, -40);
      } else if (robot.state == 15 ) {
        robot.solenoid_state = 1;
        followLineLeft(Vmed, kapa,ke);
      } 
      //ultima tent
      /*if(robot.state==0 && TouchSwitch) setState(1); //clicar no botao para iniciar
      else if (robot.state==1 && ((TouchSwitch && tis>500)||tis>10000)) setState(2); //andar em frente até ir contra a caixa
      else if (robot.state==2 && tis>500) setState(3); //forçar o movimento contra a caixa
      else if (robot.state==3 && tis>1500) setState(4); //voltar para trás 
      else if (robot.state==4 && tis > 1500 && IRLine.total > 1500) setState(5); //estar centrado
      else if (robot.state==5 && IRLine.crosses==3) setState(100); //seguir pela direita
      else if (robot.state==100 && tis>500) setState(6); //avançar mais um pouco
      else if (robot.state==6 && tis > 750 && IRLine.total > 1500) setState(7); //rodar para a esquerda
      else if (robot.state==7 && IRLine.crosses==2) setState(101); //seguir pela direita
      else if (robot.state==101 && tis>500) setState(8); //avançar mais um pouco
      else if (robot.state==8 && tis > 750 && IRLine.total > 1500) setState(9); //rodar para a direita
      else if (robot.state==9 && tis > 3000 ) setState(10); //avançar devagar 


       if (robot.state == 0) {         // Robot Stoped            
        robot.solenoid_state = 0;
        moveRobot(0, 0);
      } else if (robot.state == 1||robot.state == 2) {
        robot.solenoid_state = 1;
        followLineLeft(Vmed, kapa);
      } else if (robot.state == 3) {
        robot.solenoid_state = 1;
        moveRobot(-Vmed, 0);
      } else if (robot.state == 4 || robot.state == 6) {
        robot.solenoid_state = 1;
        moveRobot(0, 50); 
      } else if (robot.state == 5 || robot.state == 7) {
        robot.solenoid_state = 1;
        followLineRight(Vmed, kapa);
      } else if (robot.state == 8) {
        robot.solenoid_state = 1;
        moveRobot(0, -50);
      } else if (robot.state == 9) {
        robot.solenoid_state = 1;
        followLine(Vmed, kaparm);
      } else if (robot.state == 100 || robot.state == 101) {
        robot.solenoid_state = 1;
        moveRobot(Vmed, 0); 
      }*/
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
    //readRFID();
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
    Serial.print(" Crosses: ");
    Serial.print(IRLine.crosses);

    Serial.print(" Total: ");
    Serial.print(IRLine.total);
     
    Serial.println();
  }
  
}
