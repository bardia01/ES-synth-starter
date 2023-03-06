#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
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
volatile int8_t rotation_variable = 0;

volatile int8_t prev_change=0;


#define increase 1
#define decrease -1
#define no_change 0
#define intermediate 0
#define impossible 3

const int8_t rotato[16] = { no_change, increase, intermediate, impossible, decrease, no_change, 
                           impossible, intermediate, intermediate, impossible, no_change,
                           decrease, impossible, intermediate, increase, no_change}; 

int8_t local_rotation_variable=0;
void decode_knob3(bool prev_a, bool prev_b, bool curr_a, bool curr_b){
  uint8_t concat_ab = (prev_b << 3) | (prev_a << 2) | (curr_b << 1) | curr_a;
  int8_t change = rotato[concat_ab];
  
  if(change == impossible){
    local_rotation_variable += prev_change;
    prev_change = 0;
  }
  else{
    local_rotation_variable += change;
    prev_change = change;
  }
  local_rotation_variable = local_rotation_variable > 8 ? 8 : local_rotation_variable < 0 ? 0 : local_rotation_variable;

  __atomic_store_n(&rotation_variable, local_rotation_variable, __ATOMIC_ACQ_REL); 
}

void sampleISR(){
  static uint32_t phaseAcc=0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8-rotation_variable);
  analogWrite(OUTR_PIN, Vout+128);
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

volatile uint8_t keyArray[7];
char currentKey[2];
  // write a function to scan the keys and update the currentStepSize variable
  // with the step size for the selected key
  // (hint: use the stepSizes array and the readCols function

constexpr int stepSizes [12] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};

constexpr uint8_t key_num_to_char[12] = {'C','C','D','D','E','F','F','G','G','A','A','B'};
constexpr bool key_num_to_sharp[12] = {false,true,false,true,false,false,true,false,true,false,true,false};

SemaphoreHandle_t keyArrayMutex;

uint8_t rotation_it = 0;
void scanKeysTask(void *pvParameters){
  const TickType_t xFrequency = 30/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  bool prev_knob3_a = 0;
  bool prev_knob3_b = 0;
  
  bool curr_knob3_a = 0;
  bool curr_knob3_b = 0;

  uint8_t local_rotation_it = rotation_it & 0x03;

  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    uint32_t localstepsize = 0;

    // the issue that makes the knobs change notes is when u increase rowidx to 4
    for(uint8_t rowIdx = 0; rowIdx < 4; rowIdx++){
      setRow(rowIdx);
      delayMicroseconds(3);
      keyArray[rowIdx] = readCols();
      if(rowIdx == 3){
        //Reading knob 2, 3
        prev_knob3_a = curr_knob3_a;
        prev_knob3_b = curr_knob3_b;
        curr_knob3_a = keyArray[3] & 0x01;
        curr_knob3_b = keyArray[3] & 0x02;
        decode_knob3(prev_knob3_a, prev_knob3_b, curr_knob3_a, curr_knob3_b);
      }
      else{
        Serial.println(rowIdx, DEC);
        for(uint8_t i = 0; i<4; i++){
          if(!(keyArray[rowIdx] & (1<<i))){
            if(rowIdx != 3){
              localstepsize = stepSizes[rowIdx*4+i];
              currentKey[0] = key_num_to_char[rowIdx*4+i];
              currentKey[1] = key_num_to_sharp[rowIdx*4+i] ? '#' : ' ';
            }
          }
        }
      }
      __atomic_store_n(&currentStepSize, localstepsize, __ATOMIC_SEQ_CST);
      Serial.println(currentStepSize, DEC);
    }
    xSemaphoreGive(keyArrayMutex);
  }
}
void displayUpdateTask(void *pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    char rows[3];
    int n = sprintf(rows, "%X%X%X", keyArray[0], keyArray[1], keyArray[2]);
    xSemaphoreGive(keyArrayMutex);
    const char *row2 =  rows;
    const char *row3 = currentKey;
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory 
    u8g2.drawStr(2,20, row2);
    u8g2.drawStr(2,30, row3);
    u8g2.setCursor(20,30);
    u8g2.print(rotation_variable, DEC);
    u8g2.setCursor(50,30);
    u8g2.print(prev_change, DEC);
    digitalToggle(LED_BUILTIN);
    u8g2.sendBuffer();          // transfer internal memory to the display  
  }
}

void setup() {

  keyArrayMutex = xSemaphoreCreateMutex();
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
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &scanKeysHandle 
  );  /* Pointer to store the task handle */
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
    "displayUpdate",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &displayUpdateHandle 
  );  /* Pointer to store the task handle */
  vTaskStartScheduler();
}


void loop() {
}