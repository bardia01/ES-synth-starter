#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <string.h>
#include <math.h>
#include <ES_CAN.h>
#include <sinwave.h>


uint8_t g_handshake_msg[8] = {0};

#define HANDSHAKE_MSG_ID 255
#define SAMPLE_BUFFER_SIZE 128
#define STEREO_MSG_ID_0 254
#define STEREO_MSG_ID_1 253

SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t handshakemutex;
SemaphoreHandle_t rxmsgMutex;
SemaphoreHandle_t CAN_TX_Semaphore;
SemaphoreHandle_t sampleBufferSemaphore;

volatile bool g_initial_handshake;
bool g_outBits[7] = {true, true, true, true, true, true, true};

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

// Globals for the audio output
volatile uint8_t loctave_1 = 0;
volatile uint8_t loctave_2 = 0;
volatile uint8_t uoctave_1 = 0;
volatile uint8_t uoctave_2 = 0;
volatile uint8_t g_control_from_joystick = 0;

//Lab 1 section 1, reading a single row of the key matrix
volatile uint8_t currentStepSize;
uint8_t keyArray[7];
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
constexpr uint32_t stepSizes [] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};
// = 2^32 * f / Fs = 2^32 * 440 / 22k

// For displaying notes
std::string keyMap[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

// Knob class
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

// Globals for audio output
volatile uint8_t g_stereo_keys_p1;
volatile uint8_t g_stereo_keys_p2;
volatile uint8_t g_keys_pressed_p1;
volatile uint8_t g_keys_pressed_p2;

volatile uint8_t g_HSEast = 0;
volatile uint8_t g_HSWest = 0;
volatile uint8_t g_myPos = 0;

uint8_t RX_Message[8]={0};
volatile uint8_t g_volume = 0;
volatile uint8_t g_octave = 0;

// Functions to create waveforms
inline void create_sawtooth(int32_t &cVout, uint8_t &count, int16_t vout_arr[], uint32_t phase_arr[], uint8_t keyint_0, uint8_t keyint_1, int8_t octave){
  for(int i=0; i<8;i++){
    if((keyint_0 & (1<<i)) != 0){ 
      count++;
      if(g_octave + octave > 4){
        phase_arr[i] += stepSizes[i] << (g_octave - 4 + octave);
      }
      else{
        phase_arr[i] += stepSizes[i] >> (-octave + 4 - g_octave);
      }
      vout_arr[i] = (phase_arr[i] >> 24) - 128; 
      vout_arr[i] = vout_arr[i] >> (8 - g_volume);
      cVout += vout_arr[i];
    }
  }
  for(int i=0; i<4;i++){
    if((keyint_1 & (1<<i)) != 0){
      count++;
      if(g_octave + octave > 4){
        phase_arr[i+8] += stepSizes[i+8] << (g_octave - 4 + octave);
      }
      else{
        phase_arr[i+8] += stepSizes[i+8] >> (-octave + 4 - g_octave);
      }
      vout_arr[i+8] = (phase_arr[i+8] >> 24) - 128; 
      vout_arr[i+8] = vout_arr[i+8] >> (8 - g_volume);
      cVout += vout_arr[i+8];
    }
  }
}
inline void create_square(int32_t &cVout, uint8_t &count, int16_t vout_arr[], uint32_t phase_arr[], uint8_t keyint_0, uint8_t keyint_1, int8_t octave){
  for(int i=0; i<8;i++){
    if((keyint_0 & (1<<i)) != 0){ 
      count++;
      if(g_octave + octave > 4){
        phase_arr[i] += stepSizes[i] << (g_octave - 4 + octave);
      }
      else{
        phase_arr[i] += stepSizes[i] >> (-octave + 4 - g_octave);
      }
      int32_t d = (phase_arr[i] >> 24) - 128; 
      vout_arr[i] = (d>0) ? 127 : -128;
      vout_arr[i] = vout_arr[i] >> (8 - g_volume);
      cVout += vout_arr[i];
    }
  }
  for(int i=0; i<4;i++){
    if((keyint_1 & (1<<i)) != 0){
      count++;
      if(g_octave + octave > 4){
        phase_arr[i+8] += stepSizes[i+8] << (g_octave - 4 + octave);
      }
      else{
        phase_arr[i+8] += stepSizes[i+8] >> (-octave + 4 - g_octave);
      }
      int32_t d = (phase_arr[i+8] >> 24) - 128; 
      vout_arr[i+8] = (d>0) ? 127 : -128;
      vout_arr[i+8] = vout_arr[i+8] >> (8 - g_volume);
      cVout += vout_arr[i+8];
    }
  }
}
inline void create_sin(int32_t &cVout, uint8_t &count, int16_t vout_arr[], uint32_t phase_arr[], uint8_t keyint_0, uint8_t keyint_1, int8_t octave){
  for(int i=0; i<8;i++){
    if((keyint_0 & (1<<i)) != 0){ 
      count++;
      if(g_octave + octave > 4){
        phase_arr[i] += stepSizes[i] << (g_octave - 4 + octave);
      }
      else{
        phase_arr[i] += stepSizes[i] >> (-octave + 4 - g_octave);
      }
      int32_t d = (phase_arr[i] >> 22); 
      vout_arr[i] = sinwave[d];
      vout_arr[i] = vout_arr[i] >> (8 - g_volume);
      cVout += vout_arr[i];
    }
  }
  for(int i=0; i<4;i++){
    if((keyint_1 & (1<<i)) != 0){
      count++;
      if(g_octave + octave > 4){
        phase_arr[i+8] += stepSizes[i+8] << (g_octave - 4 + octave);
      }
      else{
        phase_arr[i+8] += stepSizes[i+8] >> (-octave + 4 - g_octave);
      }
      int32_t d = phase_arr[i+8] >> 22; 
      vout_arr[i+8] = sinwave[d];
      vout_arr[i+8] = vout_arr[i+8] >> (8 - g_volume);
      cVout += vout_arr[i+8];
    }
  }
}
inline void create_triangle(int32_t &cVout, uint8_t &count, int16_t vout_arr[], uint32_t phase_arr[], uint8_t keyint_0, uint8_t keyint_1, int8_t octave){
  for(int i=0; i<8;i++){
    if((keyint_0 & (1<<i)) != 0){ 
      count++;
      if(g_octave + octave > 4){
        phase_arr[i] += stepSizes[i] << (g_octave - 4 + octave);
      }
      else{
        phase_arr[i] += stepSizes[i] >> (-octave + 4 - g_octave);
      }
      int32_t d = (phase_arr[i] >> 24) - 128; 
      vout_arr[i] = (d < 0 ) ? (d << 1) + 127 : 127 - (d << 1);
      vout_arr[i] = vout_arr[i] >> (8 - g_volume);
      cVout += vout_arr[i];
    }
  }
  for(int i=0; i<4;i++){
    if((keyint_1 & (1<<i)) != 0){
      count++;
      if(g_octave + octave > 4){
        phase_arr[i+8] += stepSizes[i+8] << (g_octave - 4 + octave);
      }
      else{
        phase_arr[i+8] += stepSizes[i+8] >> (-octave + 4 - g_octave);
      }
      int32_t d = (phase_arr[i+8] >> 24) - 128; 
      vout_arr[i+8] = (d < 0 ) ? (d << 1) + 127 : 127 - (d << 1);
      vout_arr[i+8] = vout_arr[i+8] >> (8 - g_volume);
      cVout += vout_arr[i+8];
    }
  }
}

// Global arrays for samplebuffers
uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE];
uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE];
volatile bool writeBuffer1 = false;

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

// writetx function to fill an array with a message packet
void writetx(uint8_t totx[], bool is_handshake=0){
  if(!is_handshake) totx[0] = 0;
  else totx[0] = HANDSHAKE_MSG_ID;
  totx[1] = g_octave;
  totx[2] = g_keys_pressed_p1;
  totx[3] = g_keys_pressed_p2;
  totx[4] = g_myPos;
}

// Task to read the keys
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static int8_t an3,an2 =0;
  // Variable to store the key
  static uint8_t localCurrentStepSize = 0;
  uint8_t TX_Message[8] = {0};
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    // Access keyArray here
    for(uint8_t i = 0; i < 7; i++){
      setRow(i);
      digitalWrite(OUT_PIN,g_outBits[i]); //Set value to latch in DFF
      digitalWrite(REN_PIN,1);   
      delayMicroseconds(3);
      keyArray[i] = readCols();
      digitalWrite(REN_PIN,0);     
    }
    uint8_t keyArrayCopy[7]; 
    memcpy(keyArrayCopy,keyArray, sizeof keyArray);
    uint16_t keys_pressed_copy = 0;
    for(uint8_t i = 0; i < 3; i++){
      for(uint8_t j = 0; j < 4; j++){
        if(!(keyArrayCopy[i] & (1 << j))){ 
          localCurrentStepSize = i*4 + j;
          keys_pressed_copy |= (1 << (i*4 + j));
          }
        }
      }
      uint8_t keys_pressed_p1 = keys_pressed_copy & 0xff;
      uint8_t keys_pressed_p2 = (keys_pressed_copy >> 8) & 0xff;
      g_keys_pressed_p1 = keys_pressed_p1;
      g_keys_pressed_p2 = keys_pressed_p2;
      writetx(TX_Message, false);
      currentStepSize = localCurrentStepSize;
      xSemaphoreGive(keyArrayMutex);

      uint8_t l_HSEast = (((keyArrayCopy[6]) & 0x08) >> 3);
      uint8_t l_HSWest = (((keyArrayCopy[5]) & 0x08) >> 3);
      __atomic_store(&g_HSEast, &l_HSEast, __ATOMIC_RELAXED);
      __atomic_store(&g_HSWest, &l_HSWest, __ATOMIC_RELAXED);
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
}

// Task to update the display
void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 80/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    u8g2.setFont(u8g2_font_Georgia7px_te);
    u8g2.clearBuffer();         // clear the internal memory
   #ifdef receiver
    u8g2.drawStr(113,10, "R");
    u8g2.setCursor(120,10);
    u8g2.print(g_myPos, DEC);
  #else
    u8g2.drawStr(113,10, "S");
    u8g2.setCursor(120,10);
    u8g2.print(g_myPos, DEC);
  #endif
  // 1st row
  u8g2.drawStr(5,10, "Note:");
  if(g_keys_pressed_p1 || g_keys_pressed_p2) u8g2.drawStr(32,10,keyMap[currentStepSize].c_str());
  // 2nd row
  u8g2.drawStr(5,20, "Vol:");
  u8g2.setCursor(30,20);
  u8g2.print(g_volume,DEC);
  std::string wave_type;
  if(g_control_from_joystick == 2) wave_type = "Sine";
  else if(g_control_from_joystick == 0)wave_type = "Sawtooth";
  else if(g_control_from_joystick == 1) wave_type = "Square";
  else if(g_control_from_joystick == 3) wave_type = "Triangle";
  u8g2.drawStr(55,10, wave_type.c_str()); 
  // 3rd row
  u8g2.drawStr(5,30, "Oct:");
  u8g2.setCursor(30,30);
  u8g2.print(g_octave,DEC);
  u8g2.sendBuffer();          // transfer internal memory to the display
  digitalToggle(LED_BUILTIN);
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


// Global for handshaking 
volatile bool g_handshake_received = 0;

void handshaketask(void * pvParameters) {
  const TickType_t xFrequency = 80/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
   vTaskDelayUntil( &xLastWakeTime, xFrequency );
   xSemaphoreTake(handshakemutex, portMAX_DELAY);
   xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
   uint8_t keyarraytmp[8] = {0};
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

    // If the handshake msg has been received
    if(g_handshake_received == 1){
      if(l_HSWest == 1 && g_myPos ==0){
        l_myPos = max((int)g_myPos, g_handshake_msg[4] + 1);
        bool l_outBits[7] = {true, true, true, true, true, false, false};
        g_myPos = l_myPos;
        uint8_t handshake_msg[8] = {0};
        writetx(handshake_msg, true);
        xQueueSend(msgOutQ, handshake_msg, portMAX_DELAY);
        g_initial_handshake = false;
        for(uint8_t i = 5; i < 7; i++){
          setRow(i);
          digitalWrite(OUT_PIN,l_outBits[i]);
          digitalWrite(REN_PIN,1);   
          digitalWrite(REN_PIN,0);     
        }
        memcpy(g_outBits, l_outBits, sizeof g_outBits);
      }
      g_handshake_received = false;
    }
    xSemaphoreGive(keyArrayMutex);
    xSemaphoreGive(handshakemutex);
  }
}

// Averaging filter
int32_t avg_filter(int32_t input){
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
  static uint32_t phaseAccLO[12] = {0};
  static uint32_t phaseAccUO[12] = {0};
  static uint32_t phaseAccR[12] = {0};
  int16_t RVout[12] = {0};
  int16_t LVout[12] = {0};
  int16_t UVout[12] = {0};
  bool alone;
  while(1){
    alone = (g_myPos == 0);
    xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
    for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE; writeCtr++) {
      int32_t cVout = 0;
      uint8_t count=0;

      if(g_control_from_joystick == 0){
        //sawtooth
        //time to try lfo sine wave modulation ting
        create_sawtooth(cVout, count, LVout, phaseAccLO, loctave_1, loctave_2, alone-1);
        create_sawtooth(cVout, count, UVout, phaseAccUO, uoctave_1, uoctave_2, 1);
        create_sawtooth(cVout, count, RVout, phaseAccR, g_stereo_keys_p1, g_stereo_keys_p2, 0);
      }
      else if(g_control_from_joystick == 1){
        // square wave
        create_square(cVout, count, LVout, phaseAccLO, loctave_1, loctave_2, alone-1);
        create_square(cVout, count, UVout, phaseAccUO, uoctave_1, uoctave_2, 1);
        create_square(cVout, count, RVout, phaseAccR, g_stereo_keys_p1, g_stereo_keys_p2, 0);
      }
      else if(g_control_from_joystick == 2){
        //sinwave
      create_sin(cVout, count, LVout, phaseAccLO, loctave_1, loctave_2, alone-1);
      create_sin(cVout, count, UVout, phaseAccUO, uoctave_1, uoctave_2, 1);
      create_sin(cVout, count, RVout, phaseAccR, g_stereo_keys_p1, g_stereo_keys_p2, 0);
      }
      else if(g_control_from_joystick == 3){
        //triangle wave
      create_triangle(cVout, count, LVout, phaseAccLO, loctave_1, loctave_2, alone-1);
      create_triangle(cVout, count, UVout, phaseAccUO, uoctave_1, uoctave_2, 1);
      create_triangle(cVout, count, RVout, phaseAccR, g_stereo_keys_p1, g_stereo_keys_p2, 0);
      }
      
      // Dynamic scaling of cVout, appropriate since now in a task rather than the ISR
      cVout = (float)cVout / (float)count;

      // Filter the scaled cVout and return the result back into the same variable
      cVout = avg_filter(cVout);

      // Apply the multiplier to the cVout and clamp the result to the 8-bit range
      cVout = max(-128, min(127, (int)(cVout)));

      // Write the filtered and clamped result to the output buffer (normalise the range)
      if (writeBuffer1)
        sampleBuffer1[writeCtr] = cVout + 128;
      else
        sampleBuffer0[writeCtr] = cVout + 128;
    }
  }
}

void CANDecodeTask(void * pvParameters){
  uint8_t localRX_Message[8];
  uint8_t localRX_Message1[8] = {0};
  uint8_t l_my_id;
  const TickType_t xFrequency = 40/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    xQueueReceive(msgInQ, localRX_Message, portMAX_DELAY);
    xSemaphoreTake(handshakemutex, portMAX_DELAY);
    if(localRX_Message[0] == HANDSHAKE_MSG_ID){
      memcpy((void*)g_handshake_msg, localRX_Message, sizeof g_handshake_msg);
      g_handshake_received = true;
    }
    else if(localRX_Message[0]==STEREO_MSG_ID_1){
      uoctave_1 = localRX_Message[2];
      uoctave_2 = localRX_Message[3];
      g_control_from_joystick = localRX_Message[5];
      g_volume = localRX_Message[1] & 0x0f;
      g_octave = localRX_Message[1] >> 4;
    }
    else if(localRX_Message[0]==STEREO_MSG_ID_0){
      g_stereo_keys_p1 = localRX_Message[2];
      g_stereo_keys_p2 = localRX_Message[3];
      loctave_1 = localRX_Message[5];
      loctave_2 = localRX_Message[6];
      g_volume = localRX_Message[1] & 0x0f;
      g_octave = localRX_Message[1] >> 4;
    }
    xSemaphoreGive(handshakemutex);
    xSemaphoreGive(rxmsgMutex);
  }
}


void CAN_RX_ISR (void) {
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ,RX_Message_ISR, NULL);
}
  
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
  2,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t handshakehandle = NULL;
  xTaskCreate(
  handshaketask,		/* Function that implements the task */
  "handshake",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &handshakehandle);

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

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
  g_initial_handshake = false;
  vTaskStartScheduler();
}


void loop() {
}
