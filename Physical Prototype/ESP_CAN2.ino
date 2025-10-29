/**
 * @file ESP_CAN2.ino
 * @author Zameer Mahomed
 * @brief ESP32 Sketch for CANopen motor control, refactored for high-performance multi-tasking.
 * @version 4.1 
 * * @details
 * This code reads an ADC, talks to a 60AIM40C over CanOpen and uses a serial link at 115200 to send and recieve commands from Matlab Simulink.
 *
 * --- Core Allocation ---
 * - Core 1 (High Priority): Handles all real-time CAN bus communication.
 * - Core 0 (Background): Handles ADC reading and serial communication with MATLAB/Simulink.
 *
 * The Arduino loop() function is no longer used.
 */

#include "driver/twai.h"
#include "driver/i2s.h"
#include "esp_adc_cal.h"

// --- CONFIGURATION ---
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4
#define LOAD_CELL_PIN ADC1_CHANNEL_6 // GPIO34
#define LIMIT_SWITCH_1_PIN 25
#define LIMIT_SWITCH_2_PIN 26

#define I2S_ADC_UNIT ADC_UNIT_1
#define I2S_ADC_CHANNEL LOAD_CELL_PIN
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE 10000

const int MOTOR_NODE_ID = 1;
const uint32_t NMT_ID = 0x000;
const uint32_t SDO_TX_ID = 0x600 + MOTOR_NODE_ID;
const uint32_t SDO_RX_ID = 0x580 + MOTOR_NODE_ID;
const uint32_t TPDO1_RX_ID = 0x180 + MOTOR_NODE_ID;

// --- GLOBAL VARIABLES ---
volatile float last_known_position = 0;
volatile float last_known_current = 0.0;
volatile float commanded_position = 0;
volatile float load_cell_reading = 0.0;
volatile float limit_switch_1_state = 0.0;
volatile float limit_switch_2_state = 0.0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// --- FORWARD DECLARATIONS ---
void sendCANMessage(uint32_t id, uint8_t dlc, const uint8_t *data, TickType_t timeout);

// --- INTERRUPT HANDLERS ---
void IRAM_ATTR onLimitSwitch1() {
  portENTER_CRITICAL_ISR(&mux);
  limit_switch_1_state = (digitalRead(LIMIT_SWITCH_1_PIN) == HIGH) ? 1.0 : 0.0;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR onLimitSwitch2() {
  portENTER_CRITICAL_ISR(&mux);
  limit_switch_2_state = (digitalRead(LIMIT_SWITCH_2_PIN) == HIGH) ? 1.0 : 0.0;
  portEXIT_CRITICAL_ISR(&mux);
}

// --- GLOBAL HELPER FUNCTIONS --- to send and recieve can 

void sendCANMessage(uint32_t id, uint8_t dlc, const uint8_t *data, TickType_t timeout) {
    twai_message_t message;
    message.identifier = id;
    message.flags = TWAI_MSG_FLAG_NONE;
    message.data_length_code = dlc;
    for (int i = 0; i < dlc; i++) { message.data[i] = data[i]; }
    twai_transmit(&message, timeout);
}

void requestMotorCurrent() {
    uint8_t cmd_req_current[] = { 0x40, 0x78, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
    sendCANMessage(SDO_TX_ID, 8, cmd_req_current, pdMS_TO_TICKS(5));
}

void decodeCANMessage() {
    twai_message_t message;
    if (twai_receive(&message, 0) == ESP_OK) { // Use non-blocking receive
        if (message.identifier == TPDO1_RX_ID && message.data_length_code >= 4) {
            last_known_position = (float)((int32_t)(message.data[0] | (message.data[1] << 8) | (message.data[2] << 16) | (message.data[3] << 24)));
        } else if (message.identifier == SDO_RX_ID && message.data_length_code == 8) {
            if (message.data[0] == 0x4B && message.data[1] == 0x78 && message.data[2] == 0x60) {
                int16_t raw_count = (int16_t)(message.data[4] | (message.data[5] << 8));
                last_known_current = (float)raw_count / 2000.0;
            }
        }
    }
}

void commandMotorPosition(int32_t position) {
    uint8_t pos_cmd[8] = { 0x23, 0x7A, 0x60, 0x00, 0, 0, 0, 0 };
    pos_cmd[4]=(uint8_t)(position&0xFF); pos_cmd[5]=(uint8_t)((position>>8)&0xFF); pos_cmd[6]=(uint8_t)((position>>16)&0xFF); pos_cmd[7]=(uint8_t)((position>>24)&0xFF);
    sendCANMessage(SDO_TX_ID, 8, pos_cmd, pdMS_TO_TICKS(5));
    uint8_t trigger_cmd[] = { 0x2B, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00 };
    sendCANMessage(SDO_TX_ID, 8, trigger_cmd, pdMS_TO_TICKS(5));
}

void checkSerialInput() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("P")) {
            commanded_position = command.substring(1).toFloat();
            commandMotorPosition((int32_t)commanded_position);
        }
    }
}


// --- TASKS ---

// Task for reading the ADC via I2S DMA (runs on Core 0) - faster read speed than analog read upa to 50Hz
void adcReadTask(void *parameter) {
  uint16_t i2s_read_buff[1024];
  size_t bytes_read;
  while (true) {
    i2s_read(I2S_PORT, &i2s_read_buff, sizeof(i2s_read_buff), &bytes_read, portMAX_DELAY);
    if (bytes_read > 0) {
      uint32_t sum = 0;
      int samples_read = bytes_read / sizeof(uint16_t);
      for (int i = 0; i < samples_read; i++) {
        sum += i2s_read_buff[i] & 0x0FFF;
      }
      load_cell_reading = (float)(sum / samples_read);
    }
  }
}

// Task for all CAN Bus communication (runs on Core 1)
void canBusTask(void *parameter) {
  const TickType_t loopPeriod = pdMS_TO_TICKS(10); // Run at 100 Hz
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    decodeCANMessage();
    requestMotorCurrent();
    vTaskDelayUntil(&lastWakeTime, loopPeriod);
  }
}

// Task for all Serial communication with MATLAB/Simulink (runs on Core 0) - transmit every interval
void matlabCommTask(void *parameter) {
    const TickType_t loopPeriod = pdMS_TO_TICKS(20); // 50 Hz
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {
        // Part 1: Check for incoming commands from MATLAB
        checkSerialInput();

        // Part 2: Prepare and send the main data packet to MATLAB
        float cmd_pos_copy = commanded_position;
        float actual_pos_copy = last_known_position;
        float actual_cur_copy = last_known_current;
        float load_cell_copy = load_cell_reading;
        float limit_sw_1_copy, limit_sw_2_copy;

        portENTER_CRITICAL(&mux);
        limit_sw_1_copy = limit_switch_1_state;
        limit_sw_2_copy = limit_switch_2_state;
        portEXIT_CRITICAL(&mux);

        uint8_t frame[27];
        frame[0] = 0xAA;
        memcpy(&frame[1], &cmd_pos_copy, sizeof(float));
        memcpy(&frame[5], &actual_pos_copy, sizeof(float));
        memcpy(&frame[9], &actual_cur_copy, sizeof(float));
        memcpy(&frame[13], &load_cell_copy, sizeof(float));
        memcpy(&frame[17], &limit_sw_1_copy, sizeof(float));
        memcpy(&frame[21], &limit_sw_2_copy, sizeof(float));
        frame[25] = 0x0D;
        frame[26] = 0x0A;
        Serial.write(frame, sizeof(frame));

        // Wait for the next cycle
        vTaskDelayUntil(&lastWakeTime, loopPeriod);
    }
}


// initilaisation rroutines
void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(LIMIT_SWITCH_1_PIN, INPUT_PULLDOWN);
  pinMode(LIMIT_SWITCH_2_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_1_PIN), onLimitSwitch1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_2_PIN), onLimitSwitch2, CHANGE);
  
  // --- I2S ADC Initialization ---
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = I2S_SAMPLE_RATE, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0, .dma_buf_count = 4, .dma_buf_len = 1024, .use_apll = false
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL);
  i2s_adc_enable(I2S_PORT);

  // --- TWAI (CAN) Initialization ---
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
    while (1);
  }

  // --- Run Motor Startup Commands ---> from data sheet
  //need to tell motor to set up in position mode then set 
/*
1. motor startup mode
2. postion control mode
3. Set speed torque profile
4. Electronic gearbox ratio to 1
5. configure to recieve current feedbaxk as SDO
6. Enable drive and strat ontrol
*/
  Serial.println("\n--- Starting Motor Configuration Sequence ---");
  uint8_t cmd_tpdo1_type[]={0x2F,0x00,0x18,0x02,0x01,0x00,0x00,0x00}; sendCANMessage(SDO_TX_ID,8,cmd_tpdo1_type, pdMS_TO_TICKS(1000));
  delay(20);
  uint8_t cmd_tpdo1_timer[]={0x2B,0x00,0x18,0x05,0x14,0x00,0x00,0x00}; sendCANMessage(SDO_TX_ID,8,cmd_tpdo1_timer, pdMS_TO_TICKS(1000));
  delay(20);
  uint8_t cmd2[]={0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00}; sendCANMessage(SDO_TX_ID,8,cmd2, pdMS_TO_TICKS(1000));
  delay(20);
  uint8_t cmd_profile_vel[]={0x23,0x81,0x60,0x00,0xE8,0x03,0x00,0x00}; sendCANMessage(SDO_TX_ID,8,cmd_profile_vel, pdMS_TO_TICKS(1000));
  delay(20);
  uint8_t cmd_profile_acc[]={0x23,0x83,0x60,0x00,0x20,0x4E,0x00,0x00}; sendCANMessage(SDO_TX_ID,8,cmd_profile_acc, pdMS_TO_TICKS(1000));
  delay(20);
  uint8_t cmd_nmt_start[]={0x01,(uint8_t)MOTOR_NODE_ID}; sendCANMessage(NMT_ID,2,cmd_nmt_start, pdMS_TO_TICKS(1000));
  delay(20);
  uint8_t cmd_enable1[]={0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00}; sendCANMessage(SDO_TX_ID,8,cmd_enable1, pdMS_TO_TICKS(1000));
  delay(20);
  uint8_t cmd_enable2[]={0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00}; sendCANMessage(SDO_TX_ID,8,cmd_enable2, pdMS_TO_TICKS(1000));
  delay(20);
  uint8_t cmd_enable3[]={0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00}; sendCANMessage(SDO_TX_ID,8,cmd_enable3, pdMS_TO_TICKS(1000));
  Serial.println("--- Motor initialization sequence sent. ---");
  
  // --- Create and Pin Tasks ---
  xTaskCreatePinnedToCore(adcReadTask, "ADC Task", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(matlabCommTask, "MATLAB Comm", 4096, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(canBusTask, "CAN Bus", 4096, NULL, 10, NULL, 1); // Highest priority
}

// The main loop is no longer used - faster operation
void loop() {
  vTaskDelete(NULL);
}

