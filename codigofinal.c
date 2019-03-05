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
#define LED_AZUL PB0
#define LED_VERDE PD3
#define LED_VERM PD4
#define BUZZER PD6
#define IR_PIN PD2 //IR receiver pin
#define VAL_ON 4294959390 //value that corresponds to the button we use to "lock" and "unlock the car


uint16_t max = 980, min = 255; //variaveis globais
uint8_t levantado = 0, bit_pos = 0, sinal_acabou;

uint32_t volatile sinal, aux = 0;

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
  ADMUX|= (chan & 0x0f);        //channel selection
  ADCSRA |= (1 << ADSC);        //start the conversion
  while (ADCSRA & (1 << ADSC)); //wait for end of conversion

  valor = ADCW;                 //ADCW puts together ADCL and ADCH

  if (valor < min) min = ADCW;  //since we couldn't predict the values max and min
  	  	  	  	  	  	  	  	//we check for them every time a value is read
  else if (ADCW > max) max = ADCW;
  norm = (ADCW - min);          //due to overflow issues we can't do all this operations at once
  norm_f = (norm * 1000) / (max - min);

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

  pos_med_1 = ((float)1000 / (float)soma) * (sens[1]); //Weighted average
  pos_med_2 = ((float)2000 / (float)soma) * (sens[2]); //due to overflow issues
  pos_med_3 = ((float)3000 / (float)soma) * (sens[3]); //this operations were necessary
  pos_med_4 = ((float)4000 / (float)soma) * (sens[4]);
  pos = pos_med_1 + pos_med_2 + pos_med_3 + pos_med_4;


  if ((sens[0] < 150) && (sens[1] < 150) && (sens[2] < 150) && (sens[3] < 150) && (sens[4] < 150)) //se o carro for levantado, os motores desligam
  {
    levantado = 1; //when the car was lifted the sensors read values under 150
  }
  else levantado = 0;

   return (int)pos;
}

void controlo_pos(int pos) {
  int32_t proportional, derivative, integral, last_proportional, power_error, M_ESQ_V, M_DIR_V;
  int dir, esq;

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
void motores_pwm() {

  DDRD = DDRD | (1 << DIR_DIR); //PD5 and PD7 as outputs
  DDRD = DDRD | (1 << DIR_ESQ);
  DDRB |= (1 << DDB1) | (1 << DDB2); // PB1 and PB2 as outputs

  ICR1 = 0xFFFF; // set TOP to 2^16

  OCR1A = 0; // set OCR1A=0 so the left motor won't start

  OCR1B = 0; // set OCR1B=0 so the right motor won't start

  TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // Clear OC1A/OC1B on Compare Match
                                           //(set OC1A/OC1B at BOTTOM) (non-inverting mode)

  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  // set Fast PWM mode using ICR1 as TOP

  TCCR1B |= (1 << CS10); // START the timer with no prescaling
}


void timer_ir() {
  TCCR2B = 0; //stop timer
  TCCR2A = 0; //choose normal working mode
  TCNT2 = 0; //start timer with the value 0
  TCCR2B = (7<<CA20) //choose TP=1024
}

void int0_init() {
  EICRA = 0b11; //The rising edge of INT0 generates an interrupt request.
  DDRD &= ~(1 << IR_PIN); //IR_PIN as input
  EIMSK |= (1 << INT0); // INT0: External Interrupt Request 0 Enable
  sei();
}

ISR(INT0_vect) {
  if (!sinal_acabou) { //if there is nothing to read
    TCNT2 = 0;
    while ((PIND & (1 << IR_PIN))) { //waits for the end of the reading
    	if (TCNT2 >= 254) {
        break; //if the timer overflows it means that the signal ended
        bit_pos = 0;
      }
    }
    bit_pos--; //we write in the next position (31 to 0)

    if (TCNT2 > 60 && TCNT2 < 80) { //the beginning of the signal is longer, if the timer is between these values it means we have a new signal
      bit_pos = 31; //we start to write in the 31 position
    }
    if (TCNT2 > 15 && TCNT2 < 30) { //if the input is at high this time we know it's a bit=1
      aux |= (1 <<  bit_pos); //we write 1 in the respective position (bit_pos)
    }
    if (bit_pos == 0) { //If tcnt2>254, the last bit arrived
      sinal_acabou = 1;
      sinal = aux;
      aux = 0;
    }
  }
}


int main() {
  int  pos, state = 0;
  //to make sure the output isn't on when it is defined
  PORTD = PORTD & ~((1 << BUZZER));
  PORTD = PORTD & ~((1 << LED_VERDE));
  PORTD = PORTD & ~((1 << LED_VERM));
  PORTB = PORTB & ~((1 << LED_AZUL));
  DDRB = DDRB | (1 << LED_AZUL);
  DDRD = DDRD | (1 << LED_VERDE);
  DDRD = DDRD | (1 << LED_VERM);
  DDRB = DDRB | (1 << BUZZER);
  int0_init();
  timer_ir();
  motores_pwm();
  adc_init();
  while (1) {

    pos = deteta_pos();
    controlo_pos(pos);

    switch (state) {
      case (0):
        if (VAL_ON == sinal  && (0 == levantado)) {
          state = 1;
          dados = 0;
        }
        if ((sinal != VAL_ON) && (1 == levantado)) {
          state = 2;
        }
      case (1):
        if (VAL_ON == sinal ) {
          state = 0;
          dados = 0;
        }
      case (2):
        if (VAL_ON == sinal ) {
          state = 0;
          dados = 0;
        }
    }

    if (0 == state) {
      PORTD = PORTD & ~((1 << BUZZER)); //turn off buzzer
      PORTD = PORTD & ~((1 << LED_VERDE)); //turn off green LED
      PORTD |= (1 << LED_VERM); //turn on red LED

      if (levantado == 1) {
        PORTD = PORTD & ~((1 << LED_VERM)); //turn off red LED
        PORTB |= (1 << LED_AZUL); //turn on blue LED
      }

      else {
        PORTB = PORTB & ~((1 << LED_AZUL)); //turn off blue LED
      }

      M_ESQ = 0;
      M_DIR = 0;

    }
    if (1 == state) {
      PORTD = PORTD & ~((1 << BUZZER)); //turn off the buzzer
      PORTD = PORTD & ~((1 << LED_VERM)); //turn off the red LED
      if (levantado == 1) {
        PORTD = PORTD & ~((1 << LED_VERDE));
        PORTB |= (1 << LED_AZUL); //turn on the blue LED
      }

      else {
        PORTB = PORTB & ~((1 << LED_AZUL)); //turn off the blue LED
        PORTD |= (1 << LED_VERDE); //turn on the green LED
      }

    }
    if (2 == state) {
      PORTD |= (1 << BUZZER); //turn on the buzzer
      PORTD = PORTD & ~((1 << LED_VERM)); //turn off the red LED
      PORTD = PORTD & ~((1 << LED_VERDE)); //turn off the green LED

      if (levantado == 1) {
        PORTB |= (1 << LED_AZUL); //turn on the blue LED
        M_ESQ = 0;
        M_DIR = 0;
      }
      else {
        PORTB = PORTB & ~((1 << LED_AZUL)); //turn off the blue LED
        M_ESQ = 0;
        M_DIR = 0;

      }
    }
    sinal_acabou = 0; //ready to read a new signal from the remote control



  }

}

