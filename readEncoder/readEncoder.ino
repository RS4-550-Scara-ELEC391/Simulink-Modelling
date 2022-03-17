int ioSelect = 2; // SR pin 15
int clk = 3       // SR pin 7
int dataout = 4   // SR pin 13



byte switcharray = 0;

void setup() {
  pinMode(ioSelect, OUTPUT);
  pinMode(clk, OUTPUT);
  pinMode(dataout, INPUT);

  Serial.begin(9600);   // baud rate

}

void loop() {

  byte value = 0;
  byte tempbit = 0;
  byte datain = 0;
  digitalWrite(ioselect, 0);  // enable parallel inputs
  digitalWrite(clk, 0);       // clk low
  digitalWrite(clk, 1);       // clk high, load data
  digitalWrite(ioSelect, 1);  // disable parallel inputs
  

    for(int j = 0; j<8; j++) {
      tempbit = digitalRead(dataOut);
      value = value + (tempbit << j);  // shift bit and add it
      digitalWrite(clk, 0);     // clk low 
      digitalWrite(clk, 1);     // clk high to get next bit
    }

 
  

}l
