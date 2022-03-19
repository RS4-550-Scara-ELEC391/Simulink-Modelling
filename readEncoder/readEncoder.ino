// declare pins (these can be changed)
int parallel = 4;    
int clk = 5;
int serialData = 6;

void setup() {
  // baud rate
  Serial.begin(9600);
  
  // setup io pins
  pinMode(parallel, OUTPUT);
  pinMode(clk, OUTPUT);
  pinMode(serialData, INPUT);

}

void loop() {
  // call function
  int encoder_num = readEncoder();

}


// --- Read from encoder function: ---
int readEncoder(){

  int8_t value = 0;
  int8_t tempbit = 0;
  digitalWrite(parallel, 0);  // enable parallel inputs
  digitalWrite(clk, 0);       // clk low
  digitalWrite(clk, 1);       // clk high, load data
  digitalWrite(parallel, 1);  // disable parallel inputs 

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
