
// Declare io pins
int clk1 = 4;           // clk output to shift reg1
int clk2 = 5;           // clk output to shift reg2
int serialData1 = 6;    // encoder 1 data 
int serialData2 = 7;    // encoder 2 data 
int PWM1 = 9;           // PWM output to joint 1
int PWM2 = 10;           // PWM output to joint 2
int reset1 = 8;         // homing pulse 1
int reset2 = 12;        // homing pulse 2

// PID constants (Joint 1)
const double K1 = 50;
const double Kp1 = K1*0.19;
const double Ki1 = K1*1;
const double Kd1 = K1*0.009;

// PID constants (Joint 2)
const double K2 = 100;
const double Kp2 = K2*0.1329;
const double Ki2 = K2*1;
const double Kd2 = K2*0.0044;

// timing constants
const int CF = 1163;          // ISR frequency [Hz]
const double Ts = 1/CF;       // ISR run time [s]
const int N = 100;            // filter constant
const double a = N*Ts/(1+N*Ts);   // used for derivative eqn

// position references
double ref1 = 0;      // Joint 1 desired angle - assigned in "patharray" function
double ref2 = 0;      // Joint 2 desired angle - assigned in "patharray" function
const double encoderRes = 6.283/1024;   //encoder resolution 2pi/1024


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
ref1 = patharray(1, millis());
ref2 = patharray(2, millis());
  
}

// interrupt service routine
ISR(TIMER1_COMPA_vect){  

 // ------------ Motor 1 ------------


  // get position from encoder
  int pose1 = readEncoder(serialData1, clk1);

  // calculate error
  double error1 = ref1 - encoderRes*pose1;

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
  double error2 = ref2 - encoderRes*pose2;

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

//------------- Return pre-calculated desired angle for current time ------------

double patharray(int joint, int currenttime){

  // pre-calculated using trajectories.m (Joint1: 0 to -pi/4, Joint2: 0 to pi/2)
  const double joint1path[201] = {0,-0.00016027,-0.00064108,-0.00144242,-0.0025643,-0.00400672,-0.00576968,-0.00785318,-0.01025722,-0.01298179,-0.0160269,-0.01939255,-0.02307873,-0.02708546,-0.03141272,-0.03606052,-0.04102886,-0.04631774,-0.05192715,-0.0578571,-0.06410759,-0.07067862,-0.07757019,-0.08478229,-0.09231494,-0.10016812,-0.10834183,-0.11683609,-0.12565088,-0.13478622,-0.14424209,-0.1540185,-0.16411544,-0.17453293,-0.18527095,-0.19632951,-0.20770861,-0.21940824,-0.23142842,-0.24376913,-0.25643038,-0.26941216,-0.28271449,-0.29633735,-0.31028076,-0.3245447,-0.33912917,-0.35403419,-0.36925974,-0.38480583,-0.40051219,-0.41605829,-0.43128384,-0.44618886,-0.46077333,-0.47503727,-0.48898067,-0.50260354,-0.51590586,-0.52888765,-0.5415489,-0.55388961,-0.56590979,-0.57760942,-0.58898852,-0.60004708,-0.6107851,-0.62120259,-0.63129953,-0.64107594,-0.65053181,-0.65966714,-0.66848194,-0.67697619,-0.68514991,-0.69300309,-0.70053574,-0.70774784,-0.71463941,-0.72121043,-0.72746093,-0.73339088,-0.73900029,-0.74428917,-0.74925751,-0.75390531,-0.75823257,-0.76223929,-0.76592548,-0.76929113,-0.77233624,-0.77506081,-0.77746485,-0.77954835,-0.7813113,-0.78275373,-0.78387561,-0.78467695,-0.78515776,-0.78531803};
  const double joint2path[201] = {0,0.00032054,0.00128215,0.00288484,0.00512861,0.00801345,0.01153937,0.01570636,0.02051443,0.02596358,0.0320538,0.03878509,0.04615747,0.05417092,0.06282544,0.07212104,0.08205772,0.09263547,0.1038543,0.11571421,0.12821519,0.14135725,0.15514038,0.16956459,0.18462987,0.20033623,0.21668367,0.23367218,0.25130177,0.26957243,0.28848417,0.30803699,0.32823088,0.34906585,0.37054189,0.39265901,0.41541721,0.43881648,0.46285683,0.48753825,0.51286075,0.53882433,0.56542898,0.59267471,0.62056151,0.64908939,0.67825835,0.70806838,0.73851949,0.76961167,0.80102439,0.83211657,0.86256768,0.89237771,0.92154667,0.95007455,0.97796135,1.00520708,1.03181173,1.0577753,1.0830978,1.10777923,1.13181958,1.15521885,1.17797704,1.20009416,1.22157021,1.24240518,1.26259907,1.28215188,1.30106362,1.31933429,1.33696388,1.35395239,1.37029983,1.38600619,1.40107147,1.41549568,1.42927881,1.44242087,1.45492185,1.46678176,1.47800058,1.48857834,1.49851501,1.50781062,1.51646514,1.52447859,1.53185096,1.53858226,1.54467248,1.55012163,1.5549297,1.55909669,1.56262261,1.56550745,1.56775122,1.56935391,1.57031552,1.57063606};
    
  
  // return desired position for current time
  if(currenttime < 2010){
    if (joint == 1)
    return joint1path[currenttime/10];
    else
    return joint2path[currenttime/10];
  }
  
  // after runtime, hold final position
  else {
    if (joint == 1)
    return joint1path[201];
    else
    return joint2path[201];
  }


}//patharray
