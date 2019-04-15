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
#define IGNORAR_DIR
#define IGNORAR_ESQ
#define IGNORAR_CRUZ

byte dir=0, esq=0, flagdir=0, flagesq=0, flagcruz=0;
uint16_t max = 980, min = 255; //variaveis globais
uint8_t levantado = 0, perdido = 0, bit_IR = 0, ir_acabou_q, contdir=0, contesq=0, contcruz=0;
uint32_t volatile dados, buff_ir = 0;

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

  pos_med_1 = ((float)1000 / (float)soma) * (sens[1]); //Weighted average
  pos_med_2 = ((float)2000 / (float)soma) * (sens[2]); //due to overflow issues
  pos_med_3 = ((float)3000 / (float)soma) * (sens[3]); //this operations were necessary
  pos_med_4 = ((float)4000 / (float)soma) * (sens[4]);
  pos = pos_med_1 + pos_med_2 + pos_med_3 + pos_med_4;


  if ((sens[0] < 150) && (sens[1] < 150) && (sens[2] < 150) && (sens[3] < 150) && (sens[4] < 150)) levantado = 1; //se o carro for levantado, os motores desligam
                                                                                                              //when the car was lifted the sensors read values under 150
  
  else levantado = 0;
  
  if( (sens[0] < ) && (sens[1] < ) && (sens[2] < ) && (sens[3] < ) && (sens[4] < )) { //deteta interseção (tende a virar a direita)
    dir=1;
    esq=0;
  }
  else if((sens[0] < ) && (sens[1] < ) && (sens[2] < ) && (sens[3] < ) && (sens[4] < )){ //deteta interseção (tende a virar a esq)
    dir=0;
    esq=1;  
  }
  else if((sens[0] < ) && (sens[1] < ) && (sens[2] < ) && (sens[3] < ) && (sens[4] < )){ //deteta cruzamento
    dir=1;
    esq=1;
  }
  else{
    dir=0;
    esq=0;
  }
     

  
//Serial.print(" pos: ");
//Serial.println(pos);

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
  TCCR2B = (7<<CS20); //choose TP=1024
}

void int0_init() {
  EICRA = 0b11;//The rising edge of INT0 generates an interrupt request.
  DDRD &= ~(1 << IR_PIN); //IR_PIN as input
  EIMSK |= (1 << INT0); // INT0: External Interrupt Request 0 Enable
  sei();
}


ISR(INT0_vect) {
  if (!ir_acabou_q) { //se nao houver uma leitura por ler
    TCNT2 = 0;
    while ((PIND & (1 << IR_PIN))) { //espera que acabe a componente positiva do sinal, se houver overflow é porque o sinal chegou ao fim
      if (TCNT2 >= 254) {
        break;
        bit_IR = 0;
      }
    }
    bit_IR--;

    if (TCNT2 > 60 && TCNT2 < 80) { //o inicio do sinal é mais longo
      bit_IR = 31;
    }
    if (TCNT2 > 15 && TCNT2 < 30) { //bit 1
      buff_ir |= (1 <<  bit_IR);
    }
    if (bit_IR == 0) { //Se tcnt2 for maior que 254 o ultimo pulso chegou e a variável dados guarda a informacao enviada pelo comando
      ir_acabou_q = 1;
      dados = buff_ir;
      
      buff_ir = 0;
    }
  }
}


int main() {
  int  pos, state = 0;
  DDRB = DDRB | (1 << LED_AZUL);
  DDRD = DDRD | (1 << LED_VERDE);
  DDRD = DDRD | (1 << LED_VERM);
  DDRB = DDRB | (1 << BUZZER);
  Serial.begin(9600);
  int0_init();
  timer_ir();
  motores_pwm();
  adc_init();
  while (1) {
    
   pos = deteta_pos();  
   if(flagdir=1){
    pos=IGNORAR_DIR;
   }
   else if(flagesq=1){
    pos=IGNORAR_ESQ;
   }
   else if(flagcruz=1){
    pos=IGNORAR_CRUZ;
   }
   switch (state) {
      case (0):
        if(flagdir=0 && dir=1){ //contar interseção à direita (enquanto for verdade tem de avançar, ignorando os valores dos sensores)
          contdir++;
          flagdir=1;
        }
        else if(flagdir=1 && dir=0){
          flagdir=0;
        }
        if (contdir==3) {
          if(cont==0){//tem de ir buscar a 1 peça ou seja não vira à direita
            state=1;
            flagdir=0;
            contdir=0;
          }  
          if(cont>0){
            state=13;
            flagdir=0;
            contdir=0;
          }
              
        }   
                
      case (1):
        if (VAL_ON == dados ) {
          state = 0;
          dados = 0;
        }
        if (1 == levantado) {
          state = 2;
          dados = 0;
        }
      case (2):
        if (VAL_ON == dados ) {
          state = 0;
          dados = 0;
        }
        if (0 == levantado) {
          state = 1;
          dados = 0;
        }
}

    controlo_pos(pos);
    if (levantado == 1) {
        M_ESQ = 0;
        M_DIR = 0;
      }

  }

}
