#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <string.h>
#include <ES_CAN.h>
#include <math.h>

//#define TEST_SCANKEYS 0


SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t rxmsgMutex;
SemaphoreHandle_t CAN_TX_Semaphore;
uint32_t joyX;
uint32_t joyY;
float filter_coeffs [] = {0.1, 0.2,0.3, 0.4, 0.5};
float delay_filter [5] = {0, 0, 0, 0, 0};
const uint16_t DEGREES_IN_CIRCLE = 360;
const uint16_t TABLE_SIZE = 1000;
float sine_table[TABLE_SIZE];
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

// FIR filter
double apply_fir_filter(double input)
{
  // Shift the delay line
  for (int i = 4; i > 0; i--)
  {
    delay_filter[i] = delay_filter[i-1];
  }

  // Add the new sample to the delay line
  delay_filter[0] = input;

  // Compute the output of the FIR filter
  double output = 0;
  for (int i = 0; i < 5; i++)
  {
    output += filter_coeffs[i] * delay_filter[i];
  }

  return output;
}

// Generate Sine Table
void init_sine_table() {
    for (int i = 0; i < TABLE_SIZE; i++) {
        double angle = i * DEGREES_IN_CIRCLE / static_cast<float>(TABLE_SIZE);
        sine_table[i] = sin(angle);
    }
}

double get_sine_value(double frequency, double sample_rate) {
    // Convert the frequency to an angle in degrees
    double angle = frequency * DEGREES_IN_CIRCLE / sample_rate;
    int index = static_cast<int>(angle * TABLE_SIZE / DEGREES_IN_CIRCLE) % TABLE_SIZE;
    double sine_value = sine_table[index];
    return sine_value;
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
int32_t dog = 0;
volatile uint8_t knob3rotation = 6;

void ScanJoystickTask(void * pvParameters){
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    joyX = analogRead(JOYX_PIN);
    joyY = analogRead(JOYY_PIN);
    Serial.print("Joy X: ");
    Serial.println(joyX);
    Serial.println(joyY);
  }
}

void sampleISR() {
  static int32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  uint8_t A = 5;

  // Vout =
  int32_t Vout = phaseAcc >> 24;
  Vout = Vout >> (8 - knob3rotation);
  Vout = apply_fir_filter(Vout);
  analogWrite(OUTR_PIN, (Vout + 128));
}

volatile int8_t tmp = 0;
volatile int8_t pressed = -1;

void writetx(uint8_t totx[]){
  if(tmp == pressed){
    totx[0] = 0x52;
  }
  else {
    totx[0]= 0x50;
  }
  totx[1] = 4;
  if(pressed == -1){
    totx[2] = 0;
  }
  else
  totx[2] = pressed;
}

uint32_t ID;
uint8_t RX_Message[8]={0};

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static uint8_t a,an,previnc =0;
  uint8_t TX_Message[8] = {0};
  tmp = pressed;
  // #define FIRST_RUN = true;
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.clearBuffer();         // clear the internal memory
    uint32_t localCurrentStepSize = 0;
    // Serial.println("We are in the scan keys task");
    // ScanJoystickTask(NULL);
    // Serial.println(joyX);
    // Serial.println(joyY);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    //Access keyArray here
    for(uint8_t i = 0; i < 4; i++){
      setRow(i);
      delayMicroseconds(3);
      keyArray[i] = readCols();
    }
    xSemaphoreGive(keyArrayMutex);
      uint8_t keyArraycopyy[7] = {keyArray[0], keyArray[1], keyArray[2], keyArray[3], keyArray[4], keyArray[5], keyArray[6]};
      //memcpy(&keyArraycopyy, (void*)&keyArray, sizeof keyArray);

      xSemaphoreGive(keyArrayMutex);
      for(uint8_t i = 0; i < 3; i++){
        for(uint8_t j = 0; j < 4; j++)
        {
          if(!(keyArraycopyy[i] & (1 << j)))
          {
            pressed = i*4 + j;
            localCurrentStepSize = stepSizes[pressed];
          }
        }
      }
      if (pressed == -1)
      {
        localCurrentStepSize = 0;
      }
      writetx(TX_Message);

      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
      // u8g2.sendBuffer();          // transfer internal memory to the display
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);

      if((a == 0) && (an == 1))
      {
        knob3rotation += 1;
        previnc = 1;
      }
      else if(a==1 && an == 0)
      {
        knob3rotation -= 1;
        previnc = -1;
      }
      else if(a==2 && an == 3)
      {
        knob3rotation -= 1;
        previnc = -1;
      }
      else if(a==3 && an == 2)
      {
        knob3rotation += 1;
        previnc = 1;
      }
      else if(a==0 && an == 3){
        knob3rotation += previnc;
      }
      else if(a==2 && an ==1){
        knob3rotation += previnc;
      }
      else if(a==3 && an ==0){
        knob3rotation += previnc;
      }
      if(knob3rotation > 8){
        knob3rotation = 8;
      }
      else if(knob3rotation < 0){
        knob3rotation = 0;
      }
      xSemaphoreGive(keyArrayMutex);
      a = an;
      an = ((keyArraycopyy[3]) & 0x03);

      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      #define FIRST_RUN = false;

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

    if(pressed != -1)
      u8g2.drawStr(2,10,keyMap[pressed].c_str());
    pressed = -1;

    u8g2.setCursor(2,20);
    u8g2.print(keyArray[0],HEX);
    u8g2.setCursor(22,20);
    u8g2.print(keyArray[1],HEX);
    u8g2.setCursor(42,20);
    u8g2.print(keyArray[2],HEX);

    u8g2.setCursor(62,20);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);

    u8g2.print(knob3rotation,DEC);
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
  uint32_t localCurrentStepSize = 0;
  uint8_t localRX_Message[8];
  xSemaphoreTake(rxmsgMutex, portMAX_DELAY);
  memcpy(&localRX_Message, (void*)&RX_Message, sizeof RX_Message);
  xSemaphoreGive(rxmsgMutex);
  while(1){
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    if(RX_Message[0] == 0x52){
      __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    }
    else if(RX_Message[0] == 0x50){
      localCurrentStepSize = stepSizes[RX_Message[2]] << (RX_Message[1] - 4);
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    }

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
  msgOutQ = xQueueCreate(384,8);
  //semaphore

  CAN_Init(true);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  setCANFilter(0x123,0x7ff);
  CAN_Start();
  keyArrayMutex = xSemaphoreCreateMutex();
  rxmsgMutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);
  //initialise the RTOS scheduler
  TaskHandle_t scanKeysHandle = NULL;
  #ifndef DISABLE_THREADS
    xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &scanKeysHandle );	/* Pointer to store the task handle */
  #endif
  TaskHandle_t displayUpdateHandle = NULL;
  #ifndef DISABLE_THREADS
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &scanKeysHandle );
  #endif
  TaskHandle_t candecode = NULL;
  #ifndef DISABLE_THREADS
  xTaskCreate(
    CANDecodeTask,		/* Function that implements the task */
    "CANDecode",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority */
    &candecode );	/* Pointer to store the task handle */
  #endif
  TaskHandle_t canSend = NULL;
  #ifndef DISABLE_THREADS
  xTaskCreate(
    CANSendTask,		/* Function that implements the task */
    "CANSend",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &canSend );	/* Pointer to store the task handle */
  #endif
  //Timer setup
  TaskHandle_t JoystickHandle = NULL;
  #ifndef DISABLE_THREADS
  xTaskCreate(
    ScanJoystickTask,		/* Function that implements the task */
    "Joystick",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &JoystickHandle );	/* Pointer to store the task handle */
  #endif
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  #ifndef DISABLE_THREADS
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  #endif
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
  Serial.println("before task start");
  #ifndef DISABLE_THREADS
  vTaskStartScheduler();
  #endif
  Serial.println("after task start");
  #ifdef TEST_SCANKEYS
    uint32_t startTime = micros();
    for (int iter = 0; iter < 32; iter++) {
      //Serial.print("itteration: ");
      //Serial.println(iter);
      scanKeysTask(NULL);
    }
    Serial.println(micros()-startTime);
    while(1);
  #endif

}


void loop() {
  // put your main code here, to run repeatedly:
  // static uint32_t next = millis();
  // static uint32_t count = 0;
  //Serial.println(RX_Message[2]);

  // if (millis() > next) {

  //   //Serial.println(dog);
  //   next += interval;
  //   //Update display

  //   //Toggle LED
  //   digitalToggle(LED_BUILTIN);
  // }
}
