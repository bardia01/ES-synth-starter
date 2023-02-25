#include <Arduino.h>
#include <U8g2lib.h>

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

volatile uint32_t currentStepSize = 0;

void sampleISR(){
  static uint32_t phaseAcc=0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout+128);
}
void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);
  
  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
}

uint8_t readCols(){
  // write a function to read the 4 columns and return a byte with the 4 bits set
  // according to the state of the columns
  uint8_t result = 0;
  result |= digitalRead(C0_PIN) << 0;
  result |= digitalRead(C1_PIN) << 1;
  result |= digitalRead(C2_PIN) << 2;
  result |= digitalRead(C3_PIN) << 3;
  return result;
}

void setRow(uint8_t rowIdx){
  // write a function to set the row select lines to the value of rowIdx
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}


constexpr int stepSizes [12] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};

constexpr uint8_t key_num_to_char[12] = {'C','C','D','D','E','F','F','G','G','A','A','B'};
constexpr bool key_num_to_sharp[12] = {false,true,false,true,false,false,true,false,true,false,true,false};


void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  static uint8_t keyArray[7];
  uint32_t localstepsize =0;

  if (millis() > next) {
    char currentKey[2];
    for(uint8_t rowIdx = 0; rowIdx < 3; rowIdx++){
      setRow(rowIdx);
      delayMicroseconds(3);
      keyArray[rowIdx] = readCols();

      for(uint8_t i = 0; i<4; i++){
        if(!(keyArray[rowIdx] & (1<<i))){
          localstepsize = stepSizes[rowIdx*4+i];
          currentKey[0] = key_num_to_char[rowIdx*4+i];
          currentKey[1] = key_num_to_sharp[rowIdx*4+i] ? '#' : ' ';
        }
      }
      __atomic_store_n(&currentStepSize, localstepsize, __ATOMIC_RELAXED);
      Serial.println(currentStepSize, DEC);
    }
    char rows[3];
    int n = sprintf(rows, "%X%X%X", keyArray[0], keyArray[1], keyArray[2]);
    const char *row2 =  rows;
    const char *row3 = currentKey;
    next += interval;
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory 

    u8g2.drawStr(2,20, row2);
    
    u8g2.drawStr(2,30, row3);
    //Toggle LED
    digitalToggle(LED_BUILTIN);

    //Task 1 - Read columns and display on serial
    // u8g2.setCursor(2,30);
    // u8g2.print((keyArray[0])|(keyArray[1]<<1)|(keyArray[2]<<2), HEX);
    u8g2.sendBuffer();          // transfer internal memory to the display
  }
}