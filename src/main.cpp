#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <string.h>
#include <ES_CAN.h>

SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t rxmsgMutex;
SemaphoreHandle_t CAN_TX_Semaphore;

//Constants
  const uint32_t interval = 100; //Display update interval
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;
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

//Lab 1 section 1, reading a single row of the key matrix
volatile int32_t currentStepSize;
volatile uint8_t keyArray[7];
uint8_t readCols() {
  uint8_t cols = 0;
  cols |= digitalRead(C0_PIN) << 0; // 8'b0 bitor one pin << 0 
  cols |= digitalRead(C1_PIN) << 1;
  cols |= digitalRead(C2_PIN) << 2;
  cols |= digitalRead(C3_PIN) << 3; // like 00000000 or 00000000 or 00000001 or 00000010 or 00000100
  return cols;
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN,HIGH);
}
const int32_t stepSizes [] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538}; // = 2^32 * f / Fs = 2^32 * 440 / 22k
//[64299564.93684364, 68124038.08814545, 72174973.15141818, 76469940.44741818, 81077269.0013091, 85899345.92, 91006452.48651637]
std::string keyMap[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};


class Knob {

  public:
    int upperLimit;
    int lowerLimit;
    int8_t a = 0;
    int8_t previnc = 0;
    volatile int8_t knobrotation;

    Knob(int upper, int lower){
      upperLimit = upper;
      lowerLimit = lower;
      knobrotation = (upper+lower) >> 1;
    }

    void getValue(int8_t an){
      if((a==0 && an ==1) || (a==3 && an==2)) {knobrotation+=1; previnc=1;}
      else if((a==1 && an==0) || (a==2 && an==3)) {knobrotation-=1;previnc=-1;}
      else if((a==0 && an==3) || (a==2 && an==1) || (a==3 && an ==0)) knobrotation+=previnc;
      knobrotation = min(upperLimit, max(lowerLimit, int(knobrotation)));
      a = an;
    }

};

Knob knob3(8, 0);
Knob knob2(8, 0);

uint16_t keys_pressed;
int32_t cVout = 0;
int32_t Vout[12] = {0};

// Add a local variable to store last cvout then print that
int32_t frund = 0;
uint32_t ID;
uint8_t RX_Message[8]={0};

volatile bool octave_up = false;

void sampleISR() {
  static int32_t phaseAcc[12] = {0};
  int32_t cVout = 0;
  for(int i=0; i<12;i++){
    if((keys_pressed & (1<<i)) != 0){
      if(octave_up){
        phaseAcc[i] += stepSizes[RX_Message[2]] << (RX_Message[1] - 4);
      }
      else{
        phaseAcc[i] += stepSizes[RX_Message[2]] >> (4 - RX_Message[1]);
      }
      Vout[i] = (phaseAcc[i] >> 24); 
      Vout[i] = Vout[i] >> (8 - knob3.knobrotation);
      cVout += Vout[i];
    }
  }
  cVout = max(-128, min(127, (int)cVout));
  frund = cVout;
  analogWrite(OUTR_PIN, (cVout + 128));
}

volatile int8_t press = -1;

void writetx(uint8_t totx[]){
  totx[0] = press==-1?0x52:0x50;
  totx[1] = knob2.knobrotation;
  totx[2] = press!=-1?press:totx[2];
}


void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static int8_t an3,an2 =0;
  uint8_t TX_Message[8] = {0};
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.clearBuffer();         // clear the internal memory
    uint32_t localCurrentStepSize = 0;
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    //Access keyArray here
    for(uint8_t i = 0; i < 7; i++){
      setRow(i);
      delayMicroseconds(3);
      keyArray[i] = readCols();
    }
    uint8_t keyArrayCopy[7]; 
    memcpy(keyArrayCopy,(void*)keyArray, sizeof keyArray);
    xSemaphoreGive(keyArrayMutex);
      uint8_t keyArraycopyy[7];
      uint16_t keys_pressed_copy = 0;
      memcpy(&keyArraycopyy, (void*)&keyArray, sizeof keyArray);
      xSemaphoreGive(keyArrayMutex);
      for(uint8_t i = 0; i < 3; i++){
        for(uint8_t j = 0; j < 4; j++)
        {
          if(!(keyArrayCopy[i] & (1 << j)))
          {
            keys_pressed_copy |= (1 << (i*4 + j));
            // pressed = i*4 + j;
            // localCurrentStepSize = stepSizes[pressed];
          }
        }
      }
      __atomic_store(&keys_pressed, &keys_pressed_copy, __ATOMIC_RELAXED);
      writetx(TX_Message);
      
      //__atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
      // u8g2.sendBuffer();          // transfer internal memory to the display
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      knob3.getValue(an3);
      knob2.getValue(an2);
      xSemaphoreGive(keyArrayMutex);
      an3 = ((keyArrayCopy[3]) & 0x03);
      an2 = (((keyArrayCopy[3]) & 0x0C) >> 2);
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
}
void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    xSemaphoreTake(rxmsgMutex, portMAX_DELAY);
    while (CAN_CheckRXLevel())
	    CAN_RX(ID, RX_Message);
    xSemaphoreGive(rxmsgMutex);
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    
      // write something to the internal memory
    // if(tmp != 0)
    //   u8g2.drawStr(2,10,keyMap[tmp].c_str());
    // tmp = 0;

    // if(pressed != -1)
    //   u8g2.drawStr(2,10,keyMap[pressed].c_str());
    // pressed = -1;
    u8g2.setCursor(2,30);
    u8g2.print(frund, DEC);
    u8g2.setCursor(2,20);
    u8g2.print(keyArray[0],HEX);
    u8g2.setCursor(22,20);
    u8g2.print(keyArray[1],HEX); 
    u8g2.setCursor(42,20);
    u8g2.print(keyArray[2],HEX);

    u8g2.setCursor(62,20);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    
    u8g2.print(knob3.knobrotation,DEC);
    u8g2.setCursor(82,20);
    u8g2.print(knob2.knobrotation,DEC);
    xSemaphoreGive(keyArrayMutex);
    u8g2.setCursor(66,30);
    u8g2.print((char) RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);
    
    // setRow(2);
    // uint8_t keys = readCols();
    // u8g2.setCursor(2,20);
    // u8g2.print(keys,HEX);
    u8g2.sendBuffer();          // transfer internal memory to the display
    
  }
}


void CANSendTask(void * pvParameters){
  uint8_t msgOut[8];
	while (1) {
	xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void CANDecodeTask(void * pvParameters){ 
  bool local_octave_up;;
  uint32_t localCurrentStepSize = 0;
  uint8_t localRX_Message[8];
  while(1){
    xQueueReceive(msgInQ, localRX_Message, portMAX_DELAY);
    if(localRX_Message[0] == 0x52){
      __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    }
    else if(RX_Message[0] == 0x50){
      if(RX_Message[1] >4){
        localCurrentStepSize = stepSizes[localRX_Message[2]] << (localRX_Message[1] - 4);
        local_octave_up = true;
      }
      else {
        local_octave_up = false;
        localCurrentStepSize = stepSizes[localRX_Message[2]] >> (4 - localRX_Message[1]);
      }
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
      __atomic_store_n(&octave_up, local_octave_up, __ATOMIC_RELAXED);
    }
    xSemaphoreTake(rxmsgMutex, portMAX_DELAY);
    memcpy(RX_Message, localRX_Message, sizeof RX_Message);
    xSemaphoreGive(rxmsgMutex);
  }
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}



void setup() {
  // put your setup code here, to run once:
  //initialise queue
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
  //semaphore

  CAN_Init(true);8,
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  setCANFilter(0x123,0x7ff);
  CAN_Start();
  keyArrayMutex = xSemaphoreCreateMutex();
  rxmsgMutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);
  //initialise the RTOS scheduler
  TaskHandle_t scanKeysHandle = NULL;

  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &scanKeysHandle );

  TaskHandle_t candecode = NULL;
  xTaskCreate(
    CANDecodeTask,		/* Function that implements the task */
    "CANDecode",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &candecode );	/* Pointer to store the task handle */

  TaskHandle_t canSend = NULL;
  xTaskCreate(
    CANSendTask,		/* Function that implements the task */
    "CANSend",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &canSend );	/* Pointer to store the task handle */
  //Timer setup
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
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

  vTaskStartScheduler();
}


void loop() {
  // put your main code here, to run repeatedly:
  // static uint32_t next = millis();
  // static uint32_t count = 0;
  // Serial.println(RX_Message[2]);
  Serial.println(cVout, DEC);

  // if (millis() > next) {

  //   //Serial.println(dog);
  //   next += interval;
  //   //Update display
    
  //   //Toggle LED
  //   digitalToggle(LED_BUILTIN);
  // }
}