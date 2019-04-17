#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define DIR_ESQ PD5 //Dir1: controls the direction of the left wheel.
//If PD5=0 it goes backwards, if PD5=1 it moves forward
#define DIR_DIR PD7 //Dir2: controls the direction of the right wheel.
//If PD7=0 it goes backwards, if PD7=1 it moves forward
#define Ki 5
#define Kp 130
#define Kd 25
#define M_ESQ OCR1A //value of the left motor/wheel (PB1)
#define M_DIR OCR1B //value of the right motor/wheel (PB2)
#define motor_esq_vel_med 32000  //average left motor/wheel value
#define motor_dir_vel_med 32000  //average left motor/wheel value
#define IGNORAR_DIR 1
#define IGNORAR_ESQ 1
#define IGNORAR_CRUZ 1
#define temp_ms 1000


byte dir = 0, esq = 0, cruz=0, center=0, flagdir = 0, flagesq = 0, flagcruz = 0;
uint16_t max = 980, min = 255; //variaveis globais
uint8_t levantado = 0, contdir = 0, contesq = 0, contcruz = 0, cont_arm = 0;
uint32_t proportional, derivative, integral, last_proportional, power_error, M_ESQ_V, M_DIR_V;

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

  if ( (sens[0] >900) && (sens[1] >900 ) && (sens[2] <600 ) && (sens[3] < 600) && (sens[4] < 600 )) { //deteta interseção (tende a virar a direita)
    dir = 1;
    esq = 0;
    cruz=0;
    center = 0;
  }
  else if ((sens[0] < 500) && (sens[1] < 500 ) && (sens[2] < 500) && (sens[3] >900) && (sens[4] >900 )) { //deteta interseção (tende a virar a esq)
    dir = 0;
    esq = 1;
    cruz=0;
    center = 0;
  }
  else if ((sens[0] < 600 ) && (sens[1] < 600 ) && (sens[2] < 600 ) && (sens[3] < 600) && (sens[4] < 600)) { //deteta cruzamento
    cruz=1;
    dir=0;
    esq=0;
    center = 0;
  }
  else if ((sens[0] >900 ) && (sens[1] >900 ) && (sens[2] < 450) && (sens[3] >900 ) && (sens[4] >900 )) { //esta centrado
    dir = 0;
    esq = 0;
    cruz=0;
    center = 1;
  }


  //Serial.print(" pos: ");
  //Serial.println(pos);

  return (int)pos;
}


void funcao_avancar(int pos) {
  
  //Firstly, we adjust Kp; set the Ki and Kd to 0, and adjust the value of Kp
  //to make the car run along the black line.
  //Then, we adjust Ki and Kd so that the car run smoothly

  proportional = pos - 2000; //Ideally, the central position is 2000
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

    else {
      PORTD |= (1 << DIR_DIR); //The right motor/wheel runs forward so DIR_DIR=1
      PORTD |= (1 << DIR_ESQ); //The left motor/wheel runs forward so DIR_ESQ=1
      M_ESQ = M_ESQ_V; //updates with the value resulting from the PID
      M_DIR = M_DIR_V; //updates with the value resulting from the PID

    }

 }

void funcao_recuar(int pos) {

  //Firstly, we adjust Kp; set the Ki and Kd to 0, and adjust the value of Kp
  //to make the car run along the black line.
  //Then, we adjust Ki and Kd so that the car run smoothly

  proportional = pos - 2000; //Ideally, the central position is 2000
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

    else {
        PORTD = PORTD & ~(1 << DIR_ESQ); //The left motor/wheel runs backwards so DIR_ESQ=0
        PORTD = PORTD & ~(1 << DIR_DIR); //The right motor/wheel runs backwards so DIR_DIR=0
        M_ESQ = M_ESQ_V; //updates with the value resulting from the PID
        M_DIR = M_DIR_V; //updates with the value resulting from the PID
  
      }
 }


void funcao_dir(int pos) {

/*    if(cruz==1) pos=pos-VALOR_SEM_VAL_ESQ_E_FRENTE;
    if(dir==1) pos=pos-VALOR_SEM_VAL_ESQ;*/

  //Firstly, we adjust Kp; set the Ki and Kd to 0, and adjust the value of Kp
  //to make the car run along the black line.
  //Then, we adjust Ki and Kd so that the car run smoothly

  proportional = pos - 2000; //Ideally, the central position is 2000
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


void funcao_esq(int pos) {

/*    if(cruz==1) pos=pos-VALOR_SEM_VAL_DIR_E_FRENTE;
    if(esq==1) pos=pos-VALOR_SEM_VAL_DIR; */

  //Firstly, we adjust Kp; set the Ki and Kd to 0, and adjust the value of Kp
  //to make the car run along the black line.
  //Then, we adjust Ki and Kd so that the car run smoothly

  proportional = pos - 2000; //Ideally, the central position is 2000
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
    int  pos, state = 0, fimdecurso;
    unsigned long current_temp=0, old_temp=0;

    Serial.begin(9600);
    
    timer_ir();
    motores_pwm();
    adc_init();
    while (1) {

      pos = deteta_pos();

      switch (state) {
        
        case (0):

          if (flagdir == 0 && dir == 1) { //contar interseção à direita
            contdir++;
            flagdir = 1; // variavel auxiliar
          }
          else if (flagdir == 1 && dir == 0) {
            flagdir = 0;
          }
          if (contdir == 3) {
            if (cont_arm == 0) { //tem de ir buscar a 1 peça (ou seja não vira à direita)- 1 vez
              state = 1;
              flagdir = 0;
              contdir = 0;
            }
            else if (cont_arm > 0) {
              state = 20;
              flagdir = 0;
              contdir = 0;
            }
          }


        case (1):

          if (fimdecurso == 1 ) state = 2;


        case (2):

          if ((dir == 1 && cont_arm == 0) || (cruz == 1 && cont_arm > 0)) { //volta para trás até detetar a direita
            state = 3;
          }

        case (3):

          if (center == 1) state == 4; //estar centrado

        case (4):

          while (contesq < (4 - (cont_arm + 1))) {

            if (flagesq == 0 && esq == 1) { //contar interseção à direita
              contesq++;
              flagesq = 1; // variavel auxiliar
            }
            else if (flagesq == 1 && esq == 0) {
              flagesq = 0;
            }
          }

          if (contesq == (4 - (cont_arm + 1))) {
            state = 5;
          }

        case (5):

          if (dir == 1) state = 6;

        case (6):

          if (center == 1) state == 7;

        case (7):

          if (esq == 1) state = 8; //deteta a esquerda

        case (8):

          if (cruz == 1) state = 9; //deteta o cruzamento

        case (9):

          if (dir == 1) state = 10; //deteta direita

        case (10):

          if (cruz == 1) state = 11; //deteta outro cruzamento

        case (11):

          if (center == 1) state == 12;

        case (12):

          while (contdir < (cont_arm + 1)) {
            if (flagdir == 0 && dir == 1) { //contar interseção à direita
              contdir++;
              flagdir = 1; // variavel auxiliar
            }
            else if (flagdir == 1 && dir == 0) {
              flagdir = 0;
            }
          }

          if (contdir == (cont_arm + 1)) state = 13;

        case (13):

          if (center == 1) state == 14;

        case (14):

          if (current_temp == temp_ms) state = 15;

        case (15):

          if ((cruz == 1) || (dir == 1 && cont_arm == 3)) state = 16; //deteta cruzamento

        case (16):

          cont_arm++;
          if (center == 1) state == 17;

        case (17):

          while (contesq < (cont_arm - 1)) {
            if (flagesq == 0 && esq == 1) { //contar interseção à direita
              contesq++;
              flagesq = 1; // variavel auxiliar
            }
            else if (flagesq == 1 && esq == 0) {
              flagesq = 0;
            }
          }

          if (contesq == (cont_arm - 1)) state = 18;

        case (18):

          if (dir == 1) state == 19;

        case (19):

          if (flagdir == 1 && dir == 0) { //deteta falling edge
            state = 0;
          }

        case (20):

          if (center == 1) state = 21;

        case (21):

          while (contesq < (cont_arm)) {
            if (flagesq == 0 && esq == 1) { //contar interseção à direita
              contesq++;
              flagesq = 1; // variavel auxiliar
            }
            else if (flagesq == 1 && esq == 0) {
              flagesq = 0;
            }
          }

          if (contesq == cont_arm) state = 22;

        case (22):

          if (center == 1) state = 23;

        case (23):

          if (fimdecurso == 1) state = 2;

      }

      if (state == 0) {
        funcao_avancar(pos);
      }

      if (state == 1) {
        //if (dir == 1) pos = IGNORAR_DIR;
        funcao_avancar(pos);
      }

      if (state == 2) {
        funcao_recuar(pos);
      }

      if (state == 3) {
        //if (esq== 1) pos = IGNORAR_ESQ;
        funcao_dir(pos);
      }

      if (state == 4) {
        if (esq == 1) pos = IGNORAR_ESQ;
        funcao_avancar(pos);
      }

      if (state == 5) {
        funcao_avancar(pos);
      }

      if (state == 6) {
        funcao_dir(pos);
      }

      if (state == 7) {
        funcao_avancar(pos);
      }

      if (state == 8) {
        if (esq == 1) pos = IGNORAR_ESQ;
        funcao_avancar(pos);
      }

      if (state == 9) {
        if (cruz == 1) pos = IGNORAR_CRUZ;
        funcao_avancar(pos);
      }

      if (state == 10) {
        if (dir == 1) pos = IGNORAR_DIR;
        funcao_avancar(pos);
      }

      if (state == 11) {
        funcao_esq(pos);
      }

      if (state == 12) {
        funcao_avancar(pos);
      }
      
      if (state == 13) {
        funcao_dir(pos);
      }

      if (state == 14) {
        old_temp= millis(); 
        while(current_temp!=temp_ms){
          funcao_avancar(pos);
          current_temp=millis()-old_temp;
        }
      }

      if (state == 15) {
        funcao_recuar(pos);
      }

      if (state == 16) {
        funcao_dir(pos);
      }

      if (state == 17) {
        if (esq == 1) pos = IGNORAR_ESQ;
        funcao_avancar(pos);
      }

      if (state == 18) {
        funcao_avancar(pos);
      }

      if (state == 19) {
        if (dir == 1) pos = IGNORAR_DIR;
        funcao_avancar(pos);
      }

      if (state == 20) {
        funcao_esq(pos);
      }

      if (state == 21) {
        if (esq == 1) pos = IGNORAR_ESQ;
        funcao_avancar(pos);
      }

      if (state == 22) {
        funcao_esq(pos);
      }

      if (state == 23) {
        funcao_avancar(pos);
      }

      if (levantado == 1) {
        M_ESQ = 0;
        M_DIR = 0;
      }

    }
  }





