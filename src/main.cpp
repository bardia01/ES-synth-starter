#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <string.h>
#include <math.h>
#include <ES_CAN.h>
#include <sinwave.h>

#define SAMPLE_BUFFER_SIZE 128
#define LFO_STEP_SIZE 976128.9309
#define JOYSTICK_MAX 920
#define JOYSTICK_HYSTERISIS_THRESHOLD 200
#define JOYSTICK_HYSTERISIS_THRESHOLD_SMALL 60

uint32_t joystick_neutral_x = 0;
uint32_t joystick_neutral_y = 0;

SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t rxmsgMutex;
SemaphoreHandle_t CAN_TX_Semaphore;
SemaphoreHandle_t sampleBufferSemaphore;
#define receiver
// RECEIVER

SemaphoreHandle_t handshakemutex;


bool g_initial_handshake = false;
volatile uint8_t g_handshake_msg[8] = {0};
volatile bool g_handshake_received = 0;

#ifndef receiver
  uint8_t MY_ID = 0; 
#endif

#define HANDSHAKE_MSG_ID 255

#ifdef receiver
  uint8_t MY_ID = 8;
#endif
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

// void gensin(){
//   float step = 2*3.14159265358979323846 / 1024;
//   float phase = 0;
//   for(uint32_t i = 0; i<1024; i++){
//     //Serial.println(sin(phase));
//     sinwave[i] = (int)(127.0*sin(phase));
//     //Serial.println(sinwave[i]);
//     phase += step;
//   }
// }

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
volatile uint8_t loctave_1 = 0;
volatile uint8_t loctave_2 = 0;

volatile uint8_t uoctave_1 = 0;
volatile uint8_t uoctave_2 = 0;


//Lab 1 section 1, reading a single row of the key matrix
volatile int32_t currentStepSize = 0;
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

volatile uint8_t max_id;

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
Knob knob1(3,0);
Knob knob0(3,0);

volatile uint8_t g_keys_pressed_p1;
volatile uint8_t g_keys_pressed_p2;

volatile uint8_t g_HSEast = 0;
volatile uint8_t g_HSWest = 0;



// Add a local variable to store last cvout then print that
int32_t frund = 0;
uint32_t ID;
uint8_t RX_Message[8]={0};

uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE];
uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE];
volatile bool writeBuffer1 = false;


volatile uint32_t g_joyx = 0;
volatile uint32_t g_joyy = 0;
volatile uint32_t g_joyx_prev = 0;
volatile uint32_t g_joyy_prev = 0;

volatile bool sinsound = 0;
inline void create_sawtooth(int32_t &cVout, uint16_t &count, int32_t vout_arr[], uint32_t phase_arr[], uint8_t keyint_0, uint8_t keyint_1, int8_t octave){
  for(int i=0; i<8;i++){
    if((keyint_0 & (1<<i)) != 0){ 
      count++;
      if(knob2.knobrotation + octave > 4){
        phase_arr[i] += stepSizes[i] << (knob2.knobrotation - 4 + octave);
      }
      else{
        phase_arr[i] += stepSizes[i] >> (-octave + 4 - knob2.knobrotation);
      }
      vout_arr[i] = (phase_arr[i] >> 24) - 128; 
      vout_arr[i] = vout_arr[i] >> (8 - knob3.knobrotation);
      cVout += vout_arr[i];
    }
  }
  for(int i=0; i<4;i++){
    if((keyint_1 & (1<<i)) != 0){
      count++;
      if(knob2.knobrotation + octave > 4){
        phase_arr[i+8] += stepSizes[i+8] << (knob2.knobrotation - 4 + octave);
      }
      else{
        phase_arr[i+8] += stepSizes[i+8] >> (-octave + 4 - knob2.knobrotation);
      }
      vout_arr[i+8] = (phase_arr[i+8] >> 24) - 128; 
      vout_arr[i+8] = vout_arr[i+8] >> (8 - knob3.knobrotation);
      cVout += vout_arr[i+8];
    }
  }
}
inline void create_square(int32_t &cVout, uint16_t &count, int32_t vout_arr[], uint32_t phase_arr[], uint8_t keyint_0, uint8_t keyint_1, int8_t octave){
  for(int i=0; i<8;i++){
    if((keyint_0 & (1<<i)) != 0){ 
      count++;
      if(knob2.knobrotation + octave > 4){
        phase_arr[i] += stepSizes[i] << (knob2.knobrotation - 4 + octave);
      }
      else{
        phase_arr[i] += stepSizes[i] >> (-octave + 4 - knob2.knobrotation);
      }
      int32_t d = (phase_arr[i] >> 24) - 128; 
      vout_arr[i] = (d>0) ? 127 : -128;
      vout_arr[i] = vout_arr[i] >> (8 - knob3.knobrotation);
      cVout += vout_arr[i];
    }
  }
  for(int i=0; i<4;i++){
    if((keyint_1 & (1<<i)) != 0){
      count++;
      if(knob2.knobrotation + octave > 4){
        phase_arr[i+8] += stepSizes[i+8] << (knob2.knobrotation - 4 + octave);
      }
      else{
        phase_arr[i+8] += stepSizes[i+8] >> (-octave + 4 - knob2.knobrotation);
      }
      int32_t d = (phase_arr[i+8] >> 24) - 128; 
      vout_arr[i+8] = (d>0) ? 127 : -128;
      vout_arr[i+8] = vout_arr[i+8] >> (8 - knob3.knobrotation);
      cVout += vout_arr[i+8];
    }
  }
}
inline void create_sin(int32_t &cVout, uint16_t &count, int32_t vout_arr[], uint32_t phase_arr[], uint8_t keyint_0, uint8_t keyint_1, int8_t octave){
  for(int i=0; i<8;i++){
    if((keyint_0 & (1<<i)) != 0){ 
      count++;
      if(knob2.knobrotation + octave > 4){
        phase_arr[i] += stepSizes[i] << (knob2.knobrotation - 4 + octave);
      }
      else{
        // knob2.knobrotation is [0,3]
        phase_arr[i] += stepSizes[i] >> (-octave + 4 - knob2.knobrotation);
      }
      int32_t d = (phase_arr[i] >> 22); 
      vout_arr[i] = sinwave[d];
      vout_arr[i] = vout_arr[i] >> (8 - knob3.knobrotation);
      cVout += vout_arr[i];
    }
  }
  for(int i=0; i<4;i++){
    if((keyint_1 & (1<<i)) != 0){
      count++;
      if(knob2.knobrotation + octave > 4){
        phase_arr[i+8] += stepSizes[i+8] << (knob2.knobrotation - 4 + octave);
      }
      else{
        phase_arr[i+8] += stepSizes[i+8] >> (-octave + 4 - knob2.knobrotation);
      }
      int32_t d = phase_arr[i+8] >> 22; 
      vout_arr[i+8] = sinwave[d];
      vout_arr[i+8] = vout_arr[i+8] >> (8 - knob3.knobrotation);
      cVout += vout_arr[i+8];
    }
  }
}
inline void create_triangle(int32_t &cVout, uint16_t &count, int32_t vout_arr[], uint32_t phase_arr[], uint8_t keyint_0, uint8_t keyint_1, int8_t octave){
  for(int i=0; i<8;i++){
    if((keyint_0 & (1<<i)) != 0){ 
      count++;
      if(knob2.knobrotation + octave > 4){
        phase_arr[i] += stepSizes[i] << (knob2.knobrotation - 4 + octave);
      }
      else{
        phase_arr[i] += stepSizes[i] >> (-octave + 4 - knob2.knobrotation);
      }
      int32_t d = (phase_arr[i] >> 24) - 128; 
      vout_arr[i] = (d < 0 ) ? (d << 1) + 127 : 127 - (d << 1);
      vout_arr[i] = vout_arr[i] >> (8 - knob3.knobrotation);
      cVout += vout_arr[i];
    }
  }
  for(int i=0; i<4;i++){
    if((keyint_1 & (1<<i)) != 0){
      count++;
      if(knob2.knobrotation + octave > 4){
        phase_arr[i+8] += stepSizes[i+8] << (knob2.knobrotation - 4 + octave);
      }
      else{
        phase_arr[i+8] += stepSizes[i+8] >> (-octave + 4 - knob2.knobrotation);
      }
      int32_t d = (phase_arr[i+8] >> 24) - 128; 
      vout_arr[i+8] = (d < 0 ) ? (d << 1) + 127 : 127 - (d << 1);
      vout_arr[i+8] = vout_arr[i+8] >> (8 - knob3.knobrotation);
      cVout += vout_arr[i+8];
    }
  }
}
inline void create_custom(int32_t &cVout, uint16_t &count, int32_t vout_arr, uint32_t phase_arr){
  //joystick neutral = 522, 477
  count++;
  phase_arr += stepSizes[0] * (2*(g_joyx- 522)/JOYSTICK_MAX);
  vout_arr = (phase_arr >> 24) - 128;
  vout_arr = vout_arr >> (8 - knob3.knobrotation);
  cVout += vout_arr;
}


void sampleISR() {
  static uint32_t readCtr = 0;
    if (readCtr == SAMPLE_BUFFER_SIZE){
    readCtr = 0;
    writeBuffer1 = !writeBuffer1;
    xSemaphoreGiveFromISR(sampleBufferSemaphore, NULL);
  }
  if (writeBuffer1)
    analogWrite(OUTR_PIN, sampleBuffer0[readCtr++]);
  else
    analogWrite(OUTR_PIN, sampleBuffer1[readCtr++]);
}

volatile bool press = 0;
uint8_t g_myPos = 0;

void writetx(uint8_t totx[], bool is_handshake=0){
  if(!is_handshake) totx[0] = MY_ID;
  else totx[0] = HANDSHAKE_MSG_ID;
  totx[1] = (knob2.knobrotation << 4) | knob3.knobrotation;
  totx[2] = press?g_keys_pressed_p1:totx[2];
  totx[3] = press?g_keys_pressed_p2:totx[3];
  totx[4] = g_myPos;
  totx[5] = knob1.knobrotation;
}

bool g_outBits[7] = {true, true, true, true, true, true, true};
bool tmp_outBits[7] = {true, true, true, true, true, true, true};

void handshaketask(void * pvParameters) {
  const TickType_t xFrequency = 80/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
   vTaskDelayUntil( &xLastWakeTime, xFrequency );
   xSemaphoreTake(handshakemutex, portMAX_DELAY);
   xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
   uint8_t keyarraytmp[2];
   uint8_t l_myPos;
   for(uint8_t i = 5; i < 7; i++){
      setRow(i);
      digitalWrite(REN_PIN,1);   
      delayMicroseconds(3);
      keyarraytmp[i] = readCols();
      digitalWrite(REN_PIN,0);     
    }
    uint8_t l_HSEast = (((keyarraytmp[6]) & 0x08) >> 3);
    uint8_t l_HSWest = (((keyarraytmp[5]) & 0x08) >> 3);
    if(g_handshake_received == 1 || g_initial_handshake == 1){
      if(l_HSWest == 1 && g_myPos ==0 && l_HSEast == 0){ 
        bool l_outBits[7] = {true, true, true, true, true, false, false};
        memcpy(g_outBits, l_outBits, sizeof(l_outBits));
        l_myPos = max((int)g_myPos, g_handshake_msg[4] + 1);
        __atomic_store(&g_myPos, &l_myPos, __ATOMIC_RELAXED);
        uint8_t handshake_msg[8] = {0};
        writetx(handshake_msg, true);
        xQueueSend(msgOutQ, handshake_msg, portMAX_DELAY);
        g_initial_handshake = false;
        for(uint8_t i = 0; i < 7; i++){
          setRow(i);
          digitalWrite(OUT_PIN,l_outBits[i]); //Set value to latch in DFF
          digitalWrite(REN_PIN,1); 
          digitalWrite(REN_PIN,0);     
        }
      }
      __atomic_store_n(&g_handshake_received, false, __ATOMIC_RELAXED);
    }    
    xSemaphoreGive(keyArrayMutex);
    xSemaphoreGive(handshakemutex);
  }
}


int8_t g_toggle_joyx = 0;
int8_t g_toggle_joyy = 0;
uint8_t g_control_from_joystick = 0;
/*
              X    Y
FULL LEFT =  926   ~479
FULL RIGHT = 144   ~479
FULL UP =   ~479    131
FULL DOWN = ~479    890

*/
void joysticktask(void * pvParameters){
  const TickType_t xFrequency = 300/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static int8_t l_toggle_joyx = 0;
  static int8_t l_toggle_joyy = 0;
  static uint8_t l_control_from_joystick = 0;
  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // Store prev values 
    __atomic_store(&g_joyx_prev, &g_joyx, __ATOMIC_RELAXED);
    __atomic_store(&g_joyy_prev, &g_joyy, __ATOMIC_RELAXED);

    // Read joystick values
    uint32_t l_joyx = analogRead(JOYX_PIN);
    uint32_t l_joyy = analogRead(JOYY_PIN);
    __atomic_store(&g_joyx, &l_joyx, __ATOMIC_RELAXED);
    __atomic_store(&g_joyy, &l_joyy, __ATOMIC_RELAXED);

    // Is the joystick not at the centre?
    bool active = (g_joyx < joystick_neutral_x - JOYSTICK_HYSTERISIS_THRESHOLD) || (g_joyx > joystick_neutral_x + JOYSTICK_HYSTERISIS_THRESHOLD)
                || (g_joyy < joystick_neutral_y - JOYSTICK_HYSTERISIS_THRESHOLD) || (g_joyy > joystick_neutral_y + JOYSTICK_HYSTERISIS_THRESHOLD);
  
    
    if(active && (g_toggle_joyx == 0)){
      //joy stick has moved right
      if(g_joyx < g_joyx_prev - JOYSTICK_HYSTERISIS_THRESHOLD_SMALL){
        l_toggle_joyx = 1; // 1 indicates right
      }
      //joy stick has moved left
      else if(g_joyx > g_joyx_prev + JOYSTICK_HYSTERISIS_THRESHOLD_SMALL){
        l_toggle_joyx = -1; // -1 indicates left
      }
    }
    else{
      l_toggle_joyx = 0;
    }
    __atomic_store(&g_toggle_joyx, &l_toggle_joyx, __ATOMIC_RELAXED);
    // l_control_from_joystick = min(3, max(0, int(g_control_from_joystick)+l_toggle_joyx));
    if(l_control_from_joystick + l_toggle_joyx < 0){
      l_control_from_joystick = 3;
    }
    else if(l_control_from_joystick + l_toggle_joyx > 3){
      l_control_from_joystick = 0;
    }
    else{
      l_control_from_joystick += l_toggle_joyx;
    }
    __atomic_store(&g_control_from_joystick, &l_control_from_joystick, __ATOMIC_RELAXED);
  }
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 40/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static uint8_t an3,an2,an1,an0 =0;
  uint8_t TX_Message[8] = {0};
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    uint32_t localCurrentStepSize = 0;
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    //Access keyArray here
    for(uint8_t i = 0; i < 7; i++){
      setRow(i);
      digitalWrite(OUT_PIN,g_outBits[i]); //Set value to latch in DFF
      digitalWrite(REN_PIN,1);   
      delayMicroseconds(3);
      keyArray[i] = readCols();
      digitalWrite(REN_PIN,0);   
    }
    uint8_t keyArrayCopy[7]; 
    memcpy(keyArrayCopy,(void*)keyArray, sizeof keyArray);
    xSemaphoreGive(keyArrayMutex);
      uint16_t keys_pressed_copy = 0;
      xSemaphoreGive(keyArrayMutex);
      for(uint8_t i = 0; i < 3; i++){
        for(uint8_t j = 0; j < 4; j++)
        {
          if(!(keyArrayCopy[i] & (1 << j)))
          { 
            localCurrentStepSize = (i*4 + j);
            press = 1;
            keys_pressed_copy |= (1 << (i*4 + j));
          }
        }
      }
      uint8_t keys_pressed_p1 = keys_pressed_copy & 0xff;
      uint8_t keys_pressed_p2 = (keys_pressed_copy >> 8) & 0xff;
      __atomic_store(&loctave_1, &keys_pressed_p1, __ATOMIC_RELAXED);
      __atomic_store(&loctave_2, &keys_pressed_p2, __ATOMIC_RELAXED);
      writetx(TX_Message);
      
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

      // Read knob values
      an3 = ((keyArrayCopy[3]) & 0x03);
      an2 = (((keyArrayCopy[3]) & 0x0C) >> 2);
      an1 = ((keyArrayCopy[4])& 0x03);
      an0 = ((keyArrayCopy[4])& 0x0C) >> 2;

      // Assign knob values to knob objects
      knob3.getValue(an3);
      knob2.getValue(an2);
      knob1.getValue(an1);
      knob0.getValue(an0);
      

      uint8_t l_HSEast = (((keyArrayCopy[6]) & 0x08) >> 3);
      uint8_t l_HSWest = (((keyArrayCopy[5]) & 0x08) >> 3);
      __atomic_store(&g_HSEast, &l_HSEast, __ATOMIC_RELAXED);
      __atomic_store(&g_HSWest, &l_HSWest, __ATOMIC_RELAXED);

      if(g_myPos == 0){ xQueueSend(msgOutQ, TX_Message, 0);}
      else xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
}
void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 40/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    u8g2.setFont(u8g2_font_Georgia7px_te);
    u8g2.clearBuffer();         // clear the internal memory
    xSemaphoreTake(rxmsgMutex, portMAX_DELAY);
    #ifdef receiver
    while (CAN_CheckRXLevel())
	    CAN_RX(ID, RX_Message);
    #endif
    xSemaphoreGive(rxmsgMutex);
  
    u8g2.drawStr(95,10, "Pos:");
    u8g2.setCursor(120,10);
    u8g2.print(g_myPos, DEC);
    #ifdef receiver
      u8g2.drawStr(120,30, "R");
    #endif
    u8g2.drawStr(5,10, "Note:");
    if(loctave_1 || loctave_2) u8g2.drawStr(30,10,keyMap[currentStepSize].c_str());
    u8g2.setCursor(50,10);
    u8g2.print(g_joyx, DEC);
    u8g2.setCursor(70,10);
    u8g2.print(g_joyy, DEC);
    u8g2.drawStr(5,20, "Volume:");
    u8g2.setCursor(50,20);
    u8g2.print(knob3.knobrotation,DEC);
    std::string wave_type;
    if(g_control_from_joystick == 2) wave_type = "Sine";
    else if(g_control_from_joystick == 0) wave_type = "Sawtooth";
    else if(g_control_from_joystick == 1) wave_type = "Square";
    else if(g_control_from_joystick == 3) wave_type = "Triangle";
    u8g2.drawStr(65,20, wave_type.c_str()); 
    u8g2.drawStr(5,30, "Octave:");
    u8g2.setCursor(42,30);
    u8g2.print(knob2.knobrotation,DEC);
    
    u8g2.drawStr(52,30, "AM LFO:");
    u8g2.setCursor(105,30);
    u8g2.print(knob0.knobrotation,DEC); 

    u8g2.sendBuffer();          // transfer internal memory to the display
    digitalToggle(LED_BUILTIN);
  }
}

uint8_t g_msgOut[8];
void CANSendTask(void * pvParameters){
  uint8_t msgOut[8];
	while (1) {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    g_msgOut[2] = msgOut[2];
    g_msgOut[3] = msgOut[3];
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

volatile uint16_t lfo_index = 0;

constexpr float gaussian_kernel[7] = {0.005977, 0.060598, 0.241732, 0.492446, 0.241732, 0.060598, 0.005977};
constexpr float gaussian_kernel_big[21] = {0.0005,	0.0015,	0.0039,	0.0089,	0.0183,	0.0334,	0.0549,	0.0807,	0.1063,	0.1253,	0.1324,	0.1253,	0.1063,	0.0807,	0.0549,	0.0334,	0.0183,	0.0089,	0.0039,	0.0015,	0.0005};
inline int32_t gauss_filter(int32_t input){
  static int32_t fir_buffer[21] = {0};
  int32_t output = 0;
  for(uint8_t i = 20; i > 0; i--){
    fir_buffer[i] = fir_buffer[i-1];
  }
  fir_buffer[0] = input;
  for(uint8_t i = 0; i < 21; i++){
    output += fir_buffer[i] * gaussian_kernel_big[i];
  }
  return output;
}

inline int32_t avg_filter(int32_t input){
  static int32_t lpf_buffer[4] = {0};
  int32_t output = 0;
  static uint8_t count = 0;
  lpf_buffer[count] = input;
  count = (count+1) & 0b11; 
  for(uint8_t i = 0; i<4; i++){
    output += lpf_buffer[i] >> 2;
  }
  return output;
}


void sampleBufferTask(void* pvParameters){
  // const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  // TickType_t xLastWakeTime = xTaskGetTickCount();
  static uint32_t phaseAccLO[12] = {0};
  static uint32_t phaseAccUO[12] = {0};
  static uint32_t phaseAccR[12] = {0};
  int32_t RVout[12] = {0};
  int32_t LVout[12] = {0};
  int32_t UVout[12] = {0};
  uint32_t js_phase = 0;
  int32_t js_vout = 0;  
  bool alone;
  while(1){
    // vTaskDelayUntil( &xLastWakeTime, xFrequency );
    alone = (g_myPos == 0);
    xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
    for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE; writeCtr++) {
      if(lfo_index > 4400) lfo_index = 0;
      // Serial.println("got here\n");
      int32_t cVout = 0;
      uint16_t count=0;
      // uint8_t RX_Message[8]={0};
      // xSemaphoreTake(rxmsgMutex, portMAX_DELAY);
      // memcpy(RX_Message, RX_Message, 8);
      // xSemaphoreGive(rxmsgMutex);

      if(g_control_from_joystick == 0){
        //sawtooth
        //time to try lfo sine wave modulation ting
        create_sawtooth(cVout, count, LVout, phaseAccLO, loctave_1, loctave_2, alone-1);
        create_sawtooth(cVout, count, UVout, phaseAccUO, uoctave_1, uoctave_2, 1);
        create_sawtooth(cVout, count, RVout, phaseAccR, g_keys_pressed_p1, g_keys_pressed_p2, 0);
      }
      else if(g_control_from_joystick == 1){
        // square wave
        create_square(cVout, count, LVout, phaseAccLO, loctave_1, loctave_2, alone-1);
        create_square(cVout, count, UVout, phaseAccUO, uoctave_1, uoctave_2, 1);
        create_square(cVout, count, RVout, phaseAccR, g_keys_pressed_p1, g_keys_pressed_p2, 0);
      }
      else if(g_control_from_joystick == 2){
        //sinwave
      create_sin(cVout, count, LVout, phaseAccLO, loctave_1, loctave_2, alone-1);
      create_sin(cVout, count, UVout, phaseAccUO, uoctave_1, uoctave_2, 1);
      create_sin(cVout, count, RVout, phaseAccR, g_keys_pressed_p1, g_keys_pressed_p2, 0);
      }
      else if(g_control_from_joystick == 3){
        //triangle wave
      create_triangle(cVout, count, LVout, phaseAccLO, loctave_1, loctave_2, alone-1);
      create_triangle(cVout, count, UVout, phaseAccUO, uoctave_1, uoctave_2, 1);
      create_triangle(cVout, count, RVout, phaseAccR, g_keys_pressed_p1, g_keys_pressed_p2, 0);
      }
      // else if(knob1.knobrotation == 3){
      //   create_custom(cVout, count, js_vout, js_phase);
      // }
      
      // Dynamic scaling of cVout, appropriate since now in a task rather than the ISR
      cVout = (float)cVout / (float)count;

      // Filter the scaled cVout and return the result back into the same variable
      cVout = avg_filter(cVout);

      // Apply the LFO AM modulation if the knob is not on 0
      float multiplier = (knob0.knobrotation) ? lfowave[lfo_index] : 1;

      // Increment the LFO index by the knob rotation value
      lfo_index += (knob0.knobrotation);

      // Apply the multiplier to the cVout and clamp the result to the 8-bit range
      cVout = max(-128, min(127, (int)(cVout*multiplier)));

      // frund is your friend that helps you debuff
      frund = cVout;

      // Write the filtered and clamped result to the output buffer (normalise the range)
      if (writeBuffer1)
        sampleBuffer1[writeCtr] = cVout + 128;
      else
        sampleBuffer0[writeCtr] = cVout + 128;
    }
  }
}

void CANDecodeTask(void * pvParameters){ 
  bool local_octave_up;;
  uint32_t localCurrentStepSize = 0;
  uint8_t localRX_Message[8];
  uint8_t l_my_id;
  const TickType_t xFrequency = 30/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    // vTaskDelayUntil(&xLastWakeTime, xFrequency);
    xQueueReceive(msgInQ, localRX_Message, portMAX_DELAY);
    xSemaphoreTake(handshakemutex, portMAX_DELAY);
    if(localRX_Message[0] == HANDSHAKE_MSG_ID){
      memcpy((void*)g_handshake_msg, localRX_Message, sizeof g_handshake_msg);
      g_handshake_received = true;
    }
    else{
      memcpy(RX_Message, localRX_Message, sizeof RX_Message);
      if(localRX_Message[4] == 2){
        __atomic_store_n(&g_keys_pressed_p1, localRX_Message[2], __ATOMIC_RELAXED);
        __atomic_store_n(&g_keys_pressed_p2, localRX_Message[3], __ATOMIC_RELAXED);
      }
      else if(localRX_Message[4] == 3){
        __atomic_store_n(&uoctave_1, localRX_Message[2], __ATOMIC_RELAXED);
        __atomic_store_n(&uoctave_2, localRX_Message[3], __ATOMIC_RELAXED);
      }
    }
    xSemaphoreGive(handshakemutex);
    xSemaphoreGive(rxmsgMutex);
  }
}


#ifdef receiver
  void CAN_RX_ISR (void) {
    uint8_t RX_Message_ISR[8];
    uint32_t ID;
    CAN_RX(ID, RX_Message_ISR);
    xQueueSendFromISR(msgInQ,RX_Message_ISR, NULL);
  }
#endif

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void setup() {
  // put your setup code here, to run once:
  //initialise queue
  sampleBufferSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sampleBufferSemaphore);
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
  //semaphore

  CAN_Init(false);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  TaskHandle_t candecode = NULL;
  xTaskCreate(
    CANDecodeTask,		/* Function that implements the task */
    "CANDecode",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &candecode);	/* Pointer to store the task handle */

  CAN_RegisterTX_ISR(CAN_TX_ISR);
 
  setCANFilter(0x123,0x7ff);

  CAN_Start();
  keyArrayMutex = xSemaphoreCreateMutex();
  rxmsgMutex = xSemaphoreCreateMutex();
  handshakemutex = xSemaphoreCreateMutex();

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

  TaskHandle_t handshakehandle = NULL;
  xTaskCreate(
    handshaketask,		/* Function that implements the task */
    "handshake",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &handshakehandle);
  TaskHandle_t joystickhandle = NULL;
  xTaskCreate(
    joysticktask,		/* Function that implements the task */
    "joystick",		/* Text name for the task */
    32,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &joystickhandle);

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
    "displayUpdate",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &displayUpdateHandle );

  TaskHandle_t canSend = NULL;
  xTaskCreate(
    CANSendTask,		/* Function that implements the task */
    "CANSend",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &canSend );	/* Pointer to store the task handle */

  TaskHandle_t sampleBuffer = NULL;
  xTaskCreate(
    sampleBufferTask,		/* Function that implements the tsask */
    "sampleBuffer",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &sampleBuffer );	/* Pointer to store the task handle */

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

  //Initialise UARTF
  Serial.begin(9600);
  Serial.println("Hello World");
  joystick_neutral_x = analogRead(JOYX_PIN);
  joystick_neutral_y = analogRead(JOYY_PIN);
  g_initial_handshake = true;
  knob0.knobrotation = 0;
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  //   digitalToggle(LED_BUILTIN);
  // }
}