int parallel = 1;  // pin #
int clk = 2;       // pin #
int dataout = 3;   // pin #



byte switcharray = 0;

void setup() {
  pinMode(parallel, OUTPUT);
  pinMode(clk, OUTPUT);
  pinMode(dataout, INPUT);

  Serial.begin(9600);   // baud rate

}

void loop() {

  byte value = 0;
  byte tempbit = 0;
  digitalWrite(parallel, 0);  // enable parallel inputs
  digitalWrite(clk, 0);       // clk low
  digitalWrite(clk, 1);       // clk high, load data
  digitalWrite(parallel, 1);  // disable parallel inputs
  

    for(int j = 0; j<8; j++) {
      tempbit = digitalRead(dataOut);
      value = value + (tempbit << j);  // shift bit and add it
      digitalWrite(clk, 0);     // clk low 
      digitalWrite(clk, 1);     // clk high to get next bit
    }

 
  

}l
