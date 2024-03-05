#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <knob.hpp>

//Constants
  const uint32_t interval = 100; //Display update interval
  const uint32_t stepSizes[] = {51076057, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007187, 96418756, 0}; //Shared  

  volatile uint32_t currentStepSize;
  volatile int rotation[] = {16,16,16,16};

  struct {
  std::bitset<32> inputs;  
  SemaphoreHandle_t mutex;  
  } sysState;

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



KnobRotator knob0(0);
KnobRotator knob1(1);
KnobRotator knob2(2);
KnobRotator knob3(3);

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

std::bitset<4> readCols(){
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN,HIGH);
}

void sampleISR() {
  static uint32_t phaseAcc = 0;
  // static uint32_t localCurrentStepSize = 0;
  unsigned int localCurrentStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  phaseAcc += localCurrentStepSize;
  analogWrite(OUTR_PIN, (phaseAcc >> 24) >> (8 - rotation[3] / 2));
}

void displayUpdateTask(void * pvParameters) {
  u8g2.setFont(u8g2_font_squeezed_r6_tr);
  const TickType_t xFrequency = 36/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime= xTaskGetTickCount();   
  u8g2.clearBuffer();
  u8g2.setCursor(58,22);
  u8g2.print("MusicSynth");
  u8g2.sendBuffer();  
  delay(2000);
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

    // Volume
    
    u8g2.setCursor(2,10);
    u8g2.print("Vol:");
    int localRotationValue3 = __atomic_load_n(&rotation[3], __ATOMIC_RELAXED);
    int localRotationValue0 = __atomic_load_n(&rotation[0], __ATOMIC_RELAXED);
    u8g2.drawFrame(2, 11, 40, 5);
    u8g2.drawBox(2, 11, localRotationValue3*40/16, 5);
    u8g2.drawFrame(50, 11, 40, 5);
    u8g2.drawBox(50, 11, localRotationValue0*40/16, 5);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.print(sysState.inputs.to_ulong(),HEX); 
    xSemaphoreGive(sysState.mutex);
    u8g2.sendBuffer();          // transfer internal memory to the display
    }
  }

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int preKey = 12,stepSize = 0, octave;
  uint8_t posn0 = 0, posn1 = 0, posn2 = 8;
  uint8_t TX_Message[8] = {0};
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    bool released = true; // Assume all inputs are HIGH initially
    for (int i=0; i<=4; i++) { // extent to 3 to scan ror3
      setRow(i);
      delayMicroseconds(3);
      std::bitset<4> value = readCols();
      for (int j=0; j<=4; j++){
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        sysState.inputs[i*4+j] = value[j];
        xSemaphoreGive(sysState.mutex);
      }
    }
    for (int i = 0; i < 12; i++) {
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        if (sysState.inputs[i] == LOW) {
            // static uint32_t localCurrentStepSize = stepSizes[i];
            __atomic_store_n(&currentStepSize, stepSizes[i], __ATOMIC_RELAXED);          
            // currentStepSize = stepSizes[i]; // Set currentStepSize based on this index
            released = false; // Found a LOW, so not all inputs are HIGH
        }
      xSemaphoreGive(sysState.mutex);  
    }
    if (released) {
        // If after checking all inputs, they are all HIGH, set to stepSizes[12]
        currentStepSize = stepSizes[12];
    }
        // Volume Knob
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    knob3.updateRotation(sysState.inputs);
    knob2.updateRotation(sysState.inputs);
    knob1.updateRotation(sysState.inputs);
    knob0.updateRotation(sysState.inputs);
    xSemaphoreGive(sysState.mutex);
  }
}

TIM_TypeDef *Instance = TIM1;
HardwareTimer *sampleTimer = new HardwareTimer(Instance);

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

  sysState.mutex = xSemaphoreCreateMutex();

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(displayUpdateTask, "displayUpdate",512, NULL, 1, &displayUpdateHandle);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
  vTaskStartScheduler();
}

void loop() {
  //Update display

  //Toggle LED
  digitalToggle(LED_BUILTIN);
  
}