#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>



#define DIR_ESQ PD5 //Dir1: controls the direction of the left wheel.
//If PD2=0 it goes backwards, if PD2=1 it moves forward
#define DIR_DIR PD7 //Dir2: controls the direction of the right wheel.
//If PD5=0 it goes backwards, if PD5=1 it moves forward
#define Ki 0
#define Kp 80
#define Kd 0
#define M_ESQ OCR1A //value of the left motor/wheel (PB1)
#define M_DIR OCR1B //value of the right motor/wheel (PB2)
#define motor_esq_vel_med 30000  //average left motor/wheel value
#define motor_dir_vel_med 30000  //average left motor/wheel value
#define IGNORAR_DIR 1990
#define IGNORAR_ESQ 2010
#define IGNORAR_CRUZ 2000
#define temp_ms 10000
#define FIM_CURSO 3
#define ELETROIMAN 4
#define POS_DIR 1300


byte dir = 0, esq = 0, cruz=0, center=0, flagdir = 0, flagesq = 0, flagcruz = 0;
uint16_t max = 980, min = 255; //variaveis globais
uint8_t levantado = 0, contdir = 0, contesq = 0, contcruz = 0, cont_arm = 0;
//uint32_t proportional, derivative, integral, last_proportional, power_error, M_ESQ_V, M_DIR_V;

void adc_init(void)
{
  ADMUX = (1 << REFS0);     //select AVCC
  //and set everything else to 0
  ADCSRA = (1 << ADEN) | 7; //enable the ADC and select a prescale = 128
  //(16MHz/128 = 125kHz the closest to 200kHz)

}

int readAdc(char chan) //we use a char variable to have 8 bits
{
  uint32_t norm, norm_f, valor;
  ADMUX = (1 << REFS0) | (chan & 0x0f);     //channel selection
  ADCSRA |= (1 << ADSC);        //start the conversion
  while (ADCSRA & (1 << ADSC)); //wait for end of conversion

  valor = ADCW;                 //ADCW puts together ADCL and ADCH

  if (valor < min) min = ADCW;  //since we couldn't predict the values max and min
  //we check for them every time a value is read
  else if (ADCW > max) max = ADCW;
  norm = (ADCW - min);          //due to overflow issues we can't do all this operations at once
  norm_f = (norm * 1000) / (max - min);

  // Serial.print("Valor sens ");
  // Serial.print(valor);



  return norm_f; // we return the normalized value
}

int deteta_pos() {
  int i, sens[5];
  uint32_t soma = 0;
  float pos_med_1, pos_med_2, pos_med_3, pos_med_4, pos;
  for (i = 0; i < 5; i++) {
    sens[i] = readAdc(i);
    soma = soma + sens[i];

  }

  pos_med_1 = ((float)1000 / (float)soma) * (sens[1]); //Weighted average:
  pos_med_2 = ((float)2000 / (float)soma) * (sens[2]); //due to overflow issues
  pos_med_3 = ((float)3000 / (float)soma) * (sens[3]); //these operations were necessary
  pos_med_4 = ((float)4000 / (float)soma) * (sens[4]);
  pos = pos_med_1 + pos_med_2 + pos_med_3 + pos_med_4;


  if ((sens[0] < 150) && (sens[1] < 150) && (sens[2] < 150) && (sens[3] < 150) && (sens[4] < 150)) levantado = 1; //se o carro for levantado, os motores desligam
  //when the car was lifted, the sensors read values under 150

  else levantado = 0;

  if ( (sens[0] >800) && (sens[2] <500) && (sens[3] <500) && (sens[4] < 500 )) { //deteta interseção (tende a virar a direita) && (sens[1] >800) && (sens[2] <500)
    dir = 1;
    esq = 0;
    cruz=0;
    center = 0;
  }
  else if ((sens[0] < 500) && (sens[1] <500) && (sens[2] <500) && (sens[4] >800 )) { //deteta interseção (tende a virar a esq)
    dir = 0;
    esq = 1;
    cruz=0;
    center = 0;
  }
  else if ((sens[0] < 500 ) && (sens[1] < 500 ) && (sens[2] < 500 ) && (sens[3] < 500) && (sens[4] < 500)) { //deteta cruzamento
    cruz=1;
    dir=0;
    esq=0;
    center = 0;
  }
  else if ((sens[0] >750 ) && (sens[1] >750 ) && (sens[2] < 530) && (sens[3] >750 ) && (sens[4] >750 )) { //esta centrado
    dir = 0;
    esq = 0;
    cruz=0;
    center = 1;
  }

  else{
    dir = 0;
    esq = 0;
    cruz=0;
    center = 0;
  }
  

  


  Serial.print(" pos: ");
  Serial.print(pos);
//  Serial.print("esq: ");
//  Serial.print(esq );
//  Serial.print(" dir: ");
//  Serial.print(dir);
//  Serial.print(" cruz: ");
//  Serial.print(cruz);
//  Serial.print(" center: ");
//  Serial.print(center);
//    
  return (int)pos;
}


void funcao_avancar(int pos, int pos_ideal) {
  
  
int32_t proportional, derivative, integral, last_proportional, power_error, M_ESQ_V, M_DIR_V;
  int dir, esq;

 /* proportional = pos - pos_ideal; 
  derivative = proportional - last_proportional; 
  integral += proportional; 

  last_proportional = proportional;

  power_error = proportional * Kp + derivative * Kd + integral * Ki;


    M_ESQ_V =  motor_esq_vel_med - power_error; //update the velocity of each wheel given the
    M_DIR_V =  motor_dir_vel_med + power_error; //error (how far the car is from the black line)

    //if the update value exceeds the maximum (2^16), we equal the value to the maximum
    if (M_ESQ_V > 65536) {
      M_ESQ = 65530;
    }
    if (M_DIR_V > 65536) {
      M_DIR = 65530;
    }
  if (M_ESQ_V <0) {
      M_ESQ = 30000;
    }
    if (M_DIR_V <0) {
      M_DIR = 30000;
    }
    else {
      PORTD |= (1 << DIR_DIR); //The right motor/wheel runs forward so DIR_DIR=1
      PORTD |= (1 << DIR_ESQ); //The left motor/wheel runs forward so DIR_ESQ=1
      M_ESQ = M_ESQ_V; //updates with the value resulting from the PID
      M_DIR = M_DIR_V; //updates with the value resulting from the PID

    }

 }*/

 proportional = pos - pos_ideal; //Ideally, the central position is 2000
                             //So we get the position error:
                             //positive number means the car is on the right of the black line,
                             //negative number means the car is on the left of the black line.
  derivative = proportional - last_proportional; //Reflects the response speed of the car.
                                                 //The large derivative value means the fast response speed

  integral += proportional; //When the absolution value is large the error accumulation is large too,
                            //which means the car go far away from the route

  last_proportional = proportional;

  power_error = proportional * Kp + derivative * Kd + integral * Ki;

  

  if (levantado == 1) { //if the car is lifted then the motors turn off
    M_ESQ = 0;
    M_DIR = 0;


  }

   else {

    M_ESQ_V =  motor_esq_vel_med - power_error; //update the velocity of each wheel given the
    M_DIR_V =  motor_dir_vel_med + power_error; //error (how far the car is from the black line)

//if the update value exceeds the maximum (2^16), we equal the value to the maximum
    if (M_ESQ_V > 65536) {
      M_ESQ = 65530;
    }
    if (M_DIR_V > 65536) {
      M_DIR = 65530;
    }

    if (M_ESQ_V <20000) {
      M_ESQ = 20000;
    }
    if (M_DIR_V <20000) {
      M_DIR = 20000;
    }

//if the update value is negative it means that it should run backwards-
//in this case we update the value of DIR_ESQ or DIR_DIR

    if (M_DIR_V < 0) {
      M_ESQ = M_ESQ_V; //update with a positive value resulting from the PID
      M_DIR = motor_dir_vel_med; //update with the average value
      PORTD |= (1 << DIR_ESQ); //The left motor/wheel runs forward so DIR_ESQ=1
      PORTD = PORTD & ~(1 << DIR_DIR); //The right motor/wheel runs backwards so DIR_DIR=0
    }
    else if (M_ESQ_V < 0) {
      M_DIR = M_DIR_V; //update with a positive value resulting from the PID
      M_ESQ = motor_esq_vel_med; //update with the average value
      PORTD |= (1 << DIR_DIR); //The right motor/wheel runs forward so DIR_DIR=1
      PORTD = PORTD & ~(1 << DIR_ESQ); //The left motor/wheel runs backwards so DIR_ESQ=0
    }
    else {
      PORTD |= (1 << DIR_DIR); //The right motor/wheel runs forward so DIR_DIR=1
      PORTD |= (1 << DIR_ESQ); //The left motor/wheel runs forward so DIR_ESQ=1
      M_ESQ = M_ESQ_V; //updates with the value resulting from the PID
      M_DIR = M_DIR_V; //updates with the value resulting from the PID

    }

  }

}


void funcao_recuar(int pos) {
  int32_t proportional, derivative, integral, last_proportional, power_error, M_ESQ_V, M_DIR_V;
  int dir, esq;


  /*

  proportional = pos - 2000; 
  derivative = proportional - last_proportional; 

  integral += proportional; 

  last_proportional = proportional;

  power_error = proportional * Kp + derivative * Kd + integral * Ki;

  
    M_ESQ_V =  motor_esq_vel_med - power_error; //update the velocity of each wheel given the
    M_DIR_V =  motor_dir_vel_med + power_error; //error (how far the car is from the black line)

    //if the update value exceeds the maximum (2^16), we equal the value to the maximum
    if (M_ESQ_V > 65536) {
      M_ESQ = 65530;
    }
    if (M_DIR_V > 65536) {
      M_DIR = 65530;
    }
  if (M_ESQ_V <0) {
      M_ESQ = 30000;
    }
    if (M_DIR_V <0) {
      M_DIR = 30000;
    }

    else {
        PORTD = PORTD & ~(1 << DIR_ESQ); //The left motor/wheel runs backwards so DIR_ESQ=0
        PORTD = PORTD & ~(1 << DIR_DIR); //The right motor/wheel runs backwards so DIR_DIR=0
        M_ESQ = M_ESQ_V; //updates with the value resulting from the PID
        M_DIR = M_DIR_V; //updates with the value resulting from the PID
  
      }
 }*/




  //Firstly, we adjust Kp; set the Ki and Kd to 0, and adjust the value of Kp
  //to make the car run along the black line.
  //Then, we adjust Ki and Kd so that the car run smoothly

  /*proportional = pos - 2000; //Ideally, the central position is 2000
                             //So we get the position error:
                             //positive number means the car is on the right of the black line,
                             //negative number means the car is on the left of the black line.
  derivative = proportional - last_proportional; //Reflects the response speed of the car.
                                                 //The large derivative value means the fast response speed

  integral += proportional; //When the absolution value is large the error accumulation is large too,
                            //which means the car go far away from the route

  last_proportional = proportional;

  power_error = proportional * Kp + derivative * Kd + integral * Ki;

  

    M_ESQ_V =  motor_esq_vel_med - power_error; //update the velocity of each wheel given the
    M_DIR_V =  motor_dir_vel_med + power_error; //error (how far the car is from the black line)

//if the update value exceeds the maximum (2^16), we equal the value to the maximum
    if (M_ESQ_V > 65536) {
      M_ESQ = 65530;
    }
    if (M_DIR_V > 65536) {
      M_DIR = 65530;
    }

  PORTD = PORTD & ~(1 << DIR_DIR);
  PORTD = PORTD & ~(1 << DIR_ESQ);

    if (M_DIR_V < 0) {
      M_ESQ = M_ESQ_V; //update with a positive value resulting from the PID
      M_DIR = motor_dir_vel_med; //update with the average value
     
    }
    else if (M_ESQ_V < 0) {
      M_DIR = M_DIR_V; //update with a positive value resulting from the PID
      M_ESQ = motor_esq_vel_med; //update with the average value
      
    }
    else {
      
      M_ESQ = M_ESQ_V; //updates with the value resulting from the PID
      M_DIR = M_DIR_V; //updates with the value resulting from the PID

    }*/

    PORTD = PORTD & ~(1 << DIR_DIR);
    PORTD = PORTD & ~(1 << DIR_ESQ);
     M_DIR = motor_esq_vel_med+10000; //update with a positive value resulting from the PID
     M_ESQ = motor_esq_vel_med; 

  

}




void funcao_dir() {
int32_t proportional, derivative, integral, last_proportional, power_error, M_ESQ_V, M_DIR_V;
  int dir, esq;

 

      M_DIR = 10000; //update with a positive value resulting from the PID
      M_ESQ = motor_esq_vel_med; //update with the average value
      PORTD |= (1 << DIR_ESQ); //The left motor/wheel runs forward so DIR_ESQ=1
      PORTD |= (1 << DIR_DIR);
      //PORTD = PORTD & ~(1 << DIR_DIR);

    
 }

 void funcao_dir2() {
int32_t proportional, derivative, integral, last_proportional, power_error, M_ESQ_V, M_DIR_V;
  int dir, esq;

 

      M_DIR = 0; //update with a positive value resulting from the PID
      M_ESQ = motor_esq_vel_med; //update with the average value
      PORTD |= (1 << DIR_ESQ); //The left motor/wheel runs forward so DIR_ESQ=1
      PORTD |= (1 << DIR_DIR);
      //PORTD = PORTD & ~(1 << DIR_DIR);

    
 }


void funcao_esq() {
int32_t proportional, derivative, integral, last_proportional, power_error, M_ESQ_V, M_DIR_V;
  int dir, esq;

   
      M_DIR = motor_esq_vel_med; //update with a positive value resulting from the PID
      M_ESQ = 10000; //update with the average value
      PORTD |= (1 << DIR_DIR);
      PORTD |= (1 << DIR_ESQ);
      
   
 }

void motores_pwm() {

  DDRD = DDRD | (1 << DIR_DIR); //PD5 e PD7 outputs
  DDRD = DDRD | (1 << DIR_ESQ);
  DDRB |= (1 << DDB1) | (1 << DDB2); // PB1 and PB2 is now an output

  ICR1 = 0xFFFF; // set TOP to 16bit

  OCR1A = 0; // set PWM bottom value=0 on motor esq

  OCR1B = 0; // set PWM bottom value=0 on motor dir

  TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // Clear OC1A/OC1B on Compare Match
  //(set OC1A/OC1B at BOTTOM) (non-inverting mode)

  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  // set Fast PWM mode using ICR1 as TOP

  TCCR1B |= (1 << CS10); // START the timer with no prescaler
}



void timer_ir() {
  TCCR2A = 0; //choose normal working mode
  TCNT2 = 0; //start timer with the value 0
  TCCR2B = (7 << CS20); //choose TP=1024
}

/*void int0_init() {
  EICRA = 0b11;//The rising edge of INT0 generates an interrupt request.
  DDRD &= ~(1 << IR_PIN); //IR_PIN as input
  EIMSK |= (1 << INT0); // INT0: External Interrupt Request 0 Enable
  sei();
}*/



  int main() {
    int  pos, state = 0, fimdecurso=1, conta_temp, pos_ideal;
    unsigned long current_temp=0, old_temp=0;
    dir = 0;
    esq = 0;
    cruz=0;
    center=0;
    flagdir = 0;
    flagesq = 0;
    flagcruz = 0;
    contdir = 0;
    contesq = 0;
    contcruz = 0;
    cont_arm = 0;
    state=0;
   Serial.begin(9600);

    pinMode(FIM_CURSO, INPUT_PULLUP);
    pinMode(ELETROIMAN, OUTPUT);
    
    //timer_ir();
    motores_pwm();
    adc_init();
    while (1) {
      
      fimdecurso = digitalRead(FIM_CURSO);

      pos = deteta_pos();
      
      
      
      switch (state) {
        
        case (0):

//           Serial.print("Contdir: ");
//           Serial.print(contdir);
//
//              Serial.print("Contarm: ");
//              Serial.print(cont_arm);

          if ((state==0) && (flagdir == 0) && (dir == 1)) { //contar interseção à direita
            contdir++;
            flagdir = 1; // variavel auxiliar
          }
          else if ((state==0) && (flagdir == 1) && (dir == 0)) {
            flagdir = 0;
          }
          if ((state==0) && (contdir == 3)) {
            if ((state==0) && (cont_arm == 0)) { //tem de ir buscar a 1 peça (ou seja não vira à direita)- 1 vez
              state = 1;
              flagdir = 0;
              contdir = 0;
            }
            else if ((state==0) && (cont_arm > 0)) {
//              Serial.print("Contarm: ");
//              Serial.print(cont_arm);
              state = 20;
              flagdir = 0;
              contdir = 0;
            }
          }

          break;


        case (1):

          if ((fimdecurso == 1)) state = 2;

          break;

        case (2):

       
        if ((((dir == 1) && (cont_arm == 0)) || ((cruz == 1) && (cont_arm > 0)))) { //volta para trás até detetar a direita
           state = 3;
          }

          break;

        case (3):

          if ((state==3) && (center == 1)) state = 4; //estar centrado

          break;

        case (4):
          contesq=0;
          flagesq = 0;
          while ((state==4) && (contesq < (4 - (cont_arm + 1)))) {

            pos = deteta_pos();
            
            if ((flagesq == 0) && (esq == 1)) { //contar interseção à direita
              
              pos_ideal= 2500 ;
              flagesq = 1; // variavel auxiliar
            }
            else if ((flagesq == 1) && (esq == 0)){
              contesq++;
              flagesq = 0;
              pos_ideal=2000;
            }
              
            
            
          funcao_avancar(pos, pos_ideal);

          Serial.print("Contesq: ");
          Serial.println(contesq);
          Serial.print(" Cont_Arm: ");
          Serial.println(cont_arm);
            
          }

          if ((state==4) && (contesq == (4 - (cont_arm + 1)))) {
            state = 5;
          }
          

          break;
          
        case (5):

          if ((state==5) && (dir == 1)) state = 6;

          break;

        case (6):

          if ((state==6) && (center == 1)) state = 7;

          break;

        case (7):

          if ((state==7) && (esq == 1)) state = 8; //deteta a esquerda
          break;

        case (8):

          if ((state==8) && (cruz == 1)) state = 9; //deteta o cruzamento
          break;

        case (9):

          if ((state==9) && (dir == 1)) state = 10; //deteta direita
          break;

        case (10):

          if ((state==10) && (cruz == 1)) state = 11; //deteta outro cruzamento
          break;

        case (11):

          if ((state==11) && (center == 1)) state = 12;
          break;

        case (12):

          contdir=0;
          
          while ((state==12) && (contdir < (cont_arm + 1))) {
            pos = deteta_pos();
            
            if ((state==12) && (flagdir == 0) && (dir == 1)) { //contar interseção à direita
              contdir++;
              flagdir = 1; // variavel auxiliar
            }
            else if ((state==12) && (flagdir == 1) && (dir == 0)) {
              flagdir = 0;
            }
          }

          if ((state==12) && (contdir == (cont_arm + 1))) state = 13;
          break;

        case (13):

          if ((state==13) && (center == 1)) state = 14;
          break;

        case (14):
          
          if ((state==14) && (conta_temp == temp_ms)) state = 15;
          break;

        case (15):

          if ((state==15) && ((cruz == 1) || ((dir == 1) && (cont_arm == 3)))) state = 16; //deteta cruzamento
          break;

        case (16):

           
          if ((state==16) && (center == 1)){
            cont_arm++;
            state = 17;
          }

          break;

        case (17):

          contesq=0;
          
          while ((state==17) && (contesq < (cont_arm - 1))) {
            pos = deteta_pos();
           // Serial.print("cont_arm:");
           // Serial.println(cont_arm);
            if ((flagesq == 0) && (esq == 1)) { //contar interseção à direita
              contesq++;
              flagesq = 1; // variavel auxiliar
            }
            else if ((state==17) && (flagesq == 1) && (esq == 0)) {
              flagesq = 0;
            }
          }

          if ((state==17) && (contesq == (cont_arm - 1))) state = 18;
          break;

        case (18):

          if ((state==18) && (dir == 1)) 
            {
              state = 19;
              flagdir=1;
            }
          break;

        case (19):

          if ((flagdir == 1) && (dir == 0)) { //deteta falling edge
            state = 0;
            contdir=0;
          }
          break;

        case (20):

            //  Serial.print("Contarm: ");
            //  Serial.print(cont_arm);


          if ((state==20) && (center == 1)) state = 21;

          break;

        case (21):
          contesq=0;
          
          while ((state==21) && (contesq < (cont_arm))) {
            pos = deteta_pos();
            if ((state==21) && (flagesq == 0) && (esq == 1)) { //contar interseção à direita
              contesq++;
              flagesq = 1; // variavel auxiliar
            }
            else if ((state==21) && (flagesq == 1) && (esq == 0)) {
              flagesq = 0;
            }
          }

          if ((state==21) && (contesq == cont_arm)) state = 22;
          break;

        case (22):

          if ((state==22) && (center == 1)) state = 23;
          break;

        case (23):

          if ((state==23) && (fimdecurso == 1)) state = 2;
          break;

      }

      if (state == 0) {
        
         
         if(dir==1) pos_ideal= POS_DIR;
         else
          pos_ideal=2000;
       
         
         funcao_avancar(pos, pos_ideal);
      }

      else if (state == 1) {
        if(dir==1) pos_ideal= POS_DIR;
         else
          pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
        digitalWrite(ELETROIMAN,1);
      }

      else if (state == 2) {
        funcao_recuar(pos);
      }

      else if (state == 3) {

        funcao_dir();
      }

      else if (state == 5) {
        
        if(esq==1) pos_ideal= 2400 ;
         else
          pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);

      
      }

     /* else if (state == 5) {

        pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
      }*/

      else if (state == 6) {
        funcao_dir2();
      }

      else if (state == 7) {
        pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
      }

      else if (state == 8) {
         if(esq==1) pos_ideal=2500 ;
         else
          pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
      }

      else if (state == 9) {
         if(cruz==1) pos_ideal=1930;
         else
          pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
      }

      else if (state == 10) {
         if(dir==1) pos_ideal= 1300;
         else
          pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
      }

      else if (state == 11) {
        funcao_esq();
      }

      else if (state == 12) {
         if(dir==1) pos_ideal=1300;
         else
          pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
      }
      
      else if (state == 13) {
        
        funcao_dir();
      }

      else if (state == 14) {
        conta_temp=0;
      
        while(conta_temp<temp_ms){
          conta_temp++;
          funcao_avancar(pos, pos_ideal);
        }
        digitalWrite(ELETROIMAN,0);
      }

      else if (state == 15) {
        funcao_recuar(pos);
      }

      else if (state == 16) {
        funcao_dir();
      }

      else if (state == 17) {
        if(esq==1) pos_ideal=2500 ;
         else
          pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
        
      }

      else if (state == 18) {
        if(dir==1) pos_ideal=1300;
         else
          pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
      }

      else if (state == 19) {
        if(dir==1) pos_ideal=1300;
         else
          pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
      }

      else if (state == 20) {
        funcao_dir();
      }

      else if (state == 21) {
        if(esq==1) pos_ideal=2500 ;
         else
          pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
      }

      else if (state == 22) {
        funcao_esq();
      }

      else if (state == 23) {
        pos_ideal=2000;
        funcao_avancar(pos, pos_ideal);
        digitalWrite(ELETROIMAN,1);
      }

      if (levantado == 1) {
        M_ESQ = 0;
        M_DIR = 0;
      }

    Serial.print("State");
    Serial.println(state);
    }
  }
