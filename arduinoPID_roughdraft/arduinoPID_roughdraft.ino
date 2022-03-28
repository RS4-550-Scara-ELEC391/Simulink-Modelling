//#include <TimerInterrupt.h>
//#include <TimerInterrupt.hpp>
//#include <ISR_Timer.h>
//#include <ISR_Timer.hpp>

// declare io pins
int clk1 = 4;           // clk output to shift reg1
int clk2 = 5;           // clk output to shift reg2
int serialData1 = 6;    // encoder 1 data 
int serialData2 = 7;    // encoder 2 data 
int PWM1 = 2;           // PWM output to joint 1
int PWM2 = 3;           // PWM output to joint 2
int reset1 = 8;         // homing pulse 1
int reset2 = 9;        // homing pulse 2

// PID constants (Joint 1)
const double K1 = 1;
const double Kp1 = K1*1;
const double Ki1 = K1*1;
const double Kd1 = K1*1;

// PID constants (Joint 1)
const double K2 = 1;
const double Kp2 = K2*1;
const double Ki2 = K2*1;
const double Kd2 = K2*1;

// timing constants
const int CF = 1163;          // ISR frequency [Hz]
const double Ts = 1/CF;       // ISR run time [s]
const int N = 100;            // filter constant
const double a = N*Ts/(1+N*Ts);   // used for derivative eqn

// step references
const double ref1 = -3.14/4;  
const double ref2 = 3.14/2;


  // initialize global variables
  double filtered_err_old1 = 0;
  double I1 = 0;
  
  double filtered_err_old2 = 0;
  double I2 = 0;

  int duty1 = 127;
  int duty2 = 127;


void setup() {
  // baud rate
  Serial.begin(9600);

  // setup io pins
  pinMode(clk1, OUTPUT);
  pinMode(clk2, OUTPUT);
  pinMode(serialData1, INPUT);
  pinMode(serialData2, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(reset1, INPUT);
  pinMode(reset2, INPUT);

  // Homing Routine:
    // rotate joint 1 back slowly until homing pulse is triggered
    while(digitalRead(reset1) == 0) {
    analogWrite(PWM1, 102);   ////40% duty (-1V)
    }
    //rotate joint 2 back slowly until homing pulse is triggered
    while(digitalRead(reset2) == 0) {
    analogWrite(PWM2, 102);   ////40% duty (-1V)
    }
  

  // --- initialize timer1 ---
  noInterrupts();           // disable all interrupts

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 13757;            // CPU freq/prescaler/desired freq = 16MHz/1163Hz
  TCCR1B |= (1 << WGM12);   // CTC mode (clear timer on compare)
  TCCR1B |= (1 << CS10);    // no prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  interrupts();             // enable all interrupts

}


void loop() {
  // loop
}

// interrupt service routine
ISR(TIMER1_COMPA_vect){  

 // ------------ Motor 1 ------------


  // get position from encoder
  int pose1 = readEncoder(serialData1, clk1);

  // calculate error
  int error1 = ref1 - pose1;

  // calculate integral
  I1 = Ki1 * error1 * Ts * + I1;

  // calculate derivative with filtered input (Ns / s+N)
  double filtered_err1 = (1-a)*filtered_err_old1 + a*error1;
  double D1 = (a/Ts)*(error1 - filtered_err_old1);

  filtered_err_old1 = filtered_err1;

  // PID value = Kp*error + Ki*integral + Kd*rate
  int PID1 = (Kp1 * error1) + (Ki1 * I1) + (Kd1 * D1);


  // set PWM duty: mapping (-5 to 5V) -> (0 to 255)
  PID1 = constrain(PID1, -5, 5);
  duty1 = 25.5*PID1 + 127.5;
  analogWrite(PWM1, duty1);

 // ------------ Motor 2 ------------

  // get position from encoder
  int pose2 = readEncoder(serialData2, clk2);

  // calculate error
  int error2 = ref2 - pose2;

  // calculate integral
  I2 = Ki2 * error2 * Ts * + I2;

  // calculate derivative with filtered input (Ns / s+N)
  double filtered_err2 = (1-a)*filtered_err_old2 + a*error2;
  double D2 = (a/Ts)*(error2 - filtered_err_old2);

  filtered_err_old2 = filtered_err2;

  // PID value = Kp*error + Ki*integral + Kd*rate
  int PID2 = (Kp2 * error2) + (Ki2 * I2) + (Kd2 * D2);


  // set PWM duty: mapping (-5 to 5V) -> (0 to 255)
  PID2 = constrain(PID2, -5, 5);    
  duty2 = 25.5*PID2 + 127.5;
  analogWrite(PWM2, duty2);

  
}// end ISR



// ------- Read Encoder function -------
int readEncoder(int serialData, int clk){

  int8_t value = 0;
  int8_t tempbit = 0;
  digitalWrite(clk, 0);       // clk low
  digitalWrite(clk, 1);       // clk high, load data

  // read in 8 bits:
  for(int i = 0; i < 8; i++){
    tempbit = digitalRead(serialData);
    value = value + (tempbit << i); // shift bit and add it
    digitalWrite(clk,0);          // clk low
    digitalWrite(clk,1);          // clk high to get next bit
  }

  // cast int8_t to int:
  int encoder_value = (int8_t) value; // signed -> negative if backwards from home, positive if forwards from home
  
  return encoder_value;
  
}//end readEncoder
