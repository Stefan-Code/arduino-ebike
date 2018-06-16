#include <SPI.h>
#include <mcp_can.h>

#define DEBUG

// CAN MCP Pins
#define MCP_INT 7 // PE6
#define MCP_CS 4  // PD4

// LEDS
#define LED_RX 12
#define LED_TX 8
#define LED_F 6

// Time constants
#define LOOP_TIME_STEP 10 // milliseconds

// Gas
// TODO: auto lower/upper range calibration on startup
#define GAS_PIN 4
#define GAS_FILTER_ALPHA 6 // right shift x bits = division by 2^x
#define GAS_FILTER_NUM_READINGS 10
#define GAS_ADC_LOWER 172
#define GAS_ADC_UPPER 862

// Ebrake
#define EBRAKE_PIN 5
#define EBRAKE_FILTER_ALPHA 6 // right shift x bits = division by 2^x
#define EBRAKE_FILTER_NUM_READINGS 10
#define EBRAKE_ADC_LOWER 165
#define EBRAKE_ADC_UPPER 800

// Brakes
#define BRAKE_L_PIN 9
#define BRAKE_R_PIN 10

// Switches
#define SWITCH_L_PIN 3
#define SWITCH_R_PIN 2

// ERPM to actual wheel rpm conversion
#define ERPM_FACTOR 20 // motor pole pair count

// Initialize CAN controller
MCP_CAN CAN0(MCP_CS);

// Globals

// Gas
int32_t gas_filtered = 0;
int32_t gas_percent; // 1/1000th of a percent

// EBrake
int32_t ebrake_filtered = 0;
int32_t ebrake_percent; // 1/1000th of a percent

// Loop Time calculation
int loop_begin_micros;
int loop_end_micros;
int loop_time;

// brake state
int BRAKE_L;
int BRAKE_R;

// switch state
int SWITCH_L;
int SWITCH_R;

void setup() {
  Serial.begin(115200); // USB Debugging
  Serial.println("INIT");

  // Pinmodes
  pinMode(LED_F, OUTPUT);
  pinMode(LED_RX, OUTPUT);
  pinMode(LED_TX, OUTPUT);

  // Brake Pullups
  pinMode(BRAKE_L_PIN, INPUT_PULLUP);
  pinMode(BRAKE_R_PIN, INPUT_PULLUP);

  // Switch Pullups
  pinMode(SWITCH_L_PIN, INPUT_PULLUP);
  pinMode(SWITCH_R_PIN, INPUT_PULLUP);

  // No masks, no filters, 500kb/s CAN
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    led_flash_all(100, 3);
  }

  else {
    // halt
    while (1) {
      Serial.println("Error Initializing MCP2515...");
      digitalWrite(LED_F, HIGH);
      delay(900);
      digitalWrite(LED_F, LOW);
      delay(100);
    }
  }

  // Change to normal mode to allow messages to be transmitted
  CAN0.setMode(MCP_NORMAL);
}

void loop() {
  loop_begin_micros = micros();
  Serial.print("Loop Time: ");
  Serial.print(loop_time);
  Serial.println("us");

  update_switches();
  update_brakes();

  read_filter_adc(GAS_PIN, GAS_FILTER_NUM_READINGS, GAS_FILTER_ALPHA, &gas_filtered);
  read_filter_adc(EBRAKE_PIN, EBRAKE_FILTER_NUM_READINGS, EBRAKE_FILTER_ALPHA, &ebrake_filtered);
  Serial.print("GAS, FILTERED: ");
  Serial.println(gas_filtered);
  Serial.print("EBRAKE, FILTERED: ");
  Serial.println(ebrake_filtered);

  // convert gas value range to 1/1000th of a percent
  gas_percent = constrain(map(gas_filtered, GAS_ADC_LOWER, GAS_ADC_UPPER, 0, 100000), 0, 10000);
  // convert ebrake value range to 1/1000th of a percent
  ebrake_percent = constrain(map(ebrake_filtered, EBRAKE_ADC_LOWER, EBRAKE_ADC_UPPER, 0, 100000), 0, 10000);

  // TODO: check ifdef DEBUG
  Serial.print("GAS PERCENT: ");
  Serial.println(gas_percent);

  Serial.print("EBRAKE, PERCENT: ");
  Serial.println(ebrake_percent);

  Serial.print("BRAKE L ");
  Serial.println(digitalRead(BRAKE_L_PIN));

  Serial.print("BRAKE R ");
  Serial.println(digitalRead(BRAKE_R_PIN));
  
  byte sndStat;  //stores CAN transmission result, TODO: could be made global

  // TODO: this should be a state machine
  // TODO: EBrake should be reverse after coming to a complete stop (and releasing the brake once)
  if (ebrake_percent == 0) { // no brake, gas OK
    if (SWITCH_R) {          // Drive forward
      sndStat = comm_can_set_current_rel(1, gas_percent);
      Serial.println("GAS FORWARD!");
    } else { // Drive ackwards
      sndStat = comm_can_set_current_rel(1, -1 * gas_percent);
      Serial.println("GAS BACKWARDS!");
    }
  } else {  //EBrake is active, disable gas
    sndStat = comm_can_set_current_brake_rel(1, ebrake_percent);
    Serial.println("BRAKE!");
  }
  if (sndStat == CAN_OK) {  //check if CAN transmission was received
    digitalWrite(LED_TX, HIGH);
    Serial.println("Message Sent Successfully!");
    delay(5);
    digitalWrite(LED_TX, LOW);
  } else {
    digitalWrite(LED_F, HIGH);
    Serial.println("Error Sending Message...");
    delay(5);
    digitalWrite(LED_F, LOW);
  }

  // loop time calculation
  loop_end_micros = micros();
  loop_time = loop_end_micros - loop_begin_micros;
  delay(LOOP_TIME_STEP);
}

// TODO: move to header file
// CAN commands
typedef enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT,
  CAN_PACKET_SET_CURRENT_BRAKE,
  CAN_PACKET_SET_RPM,
  CAN_PACKET_SET_POS,
  CAN_PACKET_FILL_RX_BUFFER,
  CAN_PACKET_FILL_RX_BUFFER_LONG,
  CAN_PACKET_PROCESS_RX_BUFFER,
  CAN_PACKET_PROCESS_SHORT_BUFFER,
  CAN_PACKET_STATUS,
  CAN_PACKET_SET_CURRENT_REL,
  CAN_PACKET_SET_CURRENT_BRAKE_REL,
  CAN_PACKET_SET_CURRENT_HANDBRAKE,
  CAN_PACKET_SET_CURRENT_HANDBRAKE_REL
} CAN_PACKET_ID;

// TODO: move to VESC specific file
byte comm_can_set_current_rel(uint8_t controller_id, int32_t current) {
  int32_t send_index = 0; // incremented by buffer_append()
  uint8_t send_buffer[4];
  buffer_append_int32(send_buffer, current, &send_index);
  return CAN0.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), 1, send_index,
                         send_buffer);
}

byte comm_can_set_current_brake_rel(uint8_t controller_id, int32_t current) {
  int32_t send_index = 0; // incremented by buffer_append()
  uint8_t send_buffer[4];
  buffer_append_int32(send_buffer, current, &send_index);
  return CAN0.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), 1,
                         send_index, send_buffer);
}

byte comm_can_set_duty(uint8_t controller_id, int32_t duty) {
  int32_t send_index = 0; // incremented by buffer_append()
  uint8_t send_buffer[4];
  buffer_append_int32(send_buffer, duty, &send_index);
  // returns sendStat
  return CAN0.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), 1, send_index,
                         send_buffer);
}

void com_can_full_break(uint8_t controller_id) { comm_can_set_duty(controller_id, 0); }

void comm_can_release(uint8_t controller_id) { comm_can_set_current_rel(controller_id, 0); }

// utility functions

// TODO: ugly
void update_switches() {
  SWITCH_L = digitalRead(SWITCH_L_PIN);
  SWITCH_R = digitalRead(SWITCH_R_PIN);
}

void update_brakes() {
  BRAKE_L = digitalRead(BRAKE_L_PIN);
  BRAKE_R = digitalRead(BRAKE_R_PIN);
}

// TODO: implement 3 value median filter (one-shot, not running)
void read_filter_adc(int pin, int num_readings, int filter_alpha, int32_t *output) {
  for (int i = 0; i < num_readings; i++) {
    int32_t raw_read = analogRead(pin);
    int32_t filter_diff = (raw_read - *output) >> filter_alpha; // right shift 1 bit = division by 2
    *output = *output + filter_diff;
  }
}

void led_flash_all(int on_time, int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_F, HIGH);
    digitalWrite(LED_RX, HIGH);
    digitalWrite(LED_TX, HIGH);
    delay(on_time);
    digitalWrite(LED_F, LOW);
    digitalWrite(LED_RX, LOW);
    digitalWrite(LED_TX, LOW);
    if (i < count - 1) {
      delay(on_time);
    }
  }
}

int32_t median_3(int32_t a, int32_t b, int32_t c, uint8_t *output) {
  if ((a <= b) && (a <= c)) {
    *output = (b <= c) ? b : c;
  } else if ((b <= a) && (b <= c)) {
    *output = (a <= c) ? a : c;
  } else {
    *output = (a <= b) ? a : b;
  }
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

//TODO: evaluate map2() vs standard map()
long map2(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

/*********************************************************************************************************
END FILE
*********************************************************************************************************/
