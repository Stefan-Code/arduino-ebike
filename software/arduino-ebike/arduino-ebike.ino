#include <mcp_can.h>
#include <SPI.h>

//CAN MCP Pins
#define MCP_INT 7 //PE6
#define MCP_CS 4 //PD4

//LEDS
#define LED_RX 12
#define LED_TX 8
#define LED_F 6

// Time constants
#define LOOP_TIME_STEP 100 //milliseconds

// Gas
#define GAS_PIN 4
#define GAS_FILTER_ALPHA 4 // right shift 2 bits (= division by 4, so alpha=0.25) 
#define GAS_FILTER_NUM_READINGS 10
#define GAS_LOWER
#define GAS_UPPER

// Initialize CAN Controller
MCP_CAN CAN0(MCP_CS);     // Set CS to pin 10

// Gas
int32_t gas_filtered;

// Loop Time
int loop_begin_micros;
int loop_end_micros;
int loop_time;




void setup()
{
  Serial.begin(115200);  //USB Debugging
  //Pinmodes
  pinMode(LED_F, OUTPUT);
  pinMode(LED_RX, OUTPUT);
  pinMode(LED_TX, OUTPUT);
  
  // No masks, no filters, 500kb/s CAN
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    led_flash_all(100, 3);
  }
  
  else {
    // halt
    while(1) {
    Serial.println("Error Initializing MCP2515...");
    digitalWrite(LED_F, HIGH);
    delay(900);
    digitalWrite(LED_F, LOW);
    delay(100);
    }
  }
  
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}

void read_gas() {
   for (int i = 0; i < GAS_FILTER_NUM_READINGS; i++) {
    int32_t gas_raw_read = analogRead(GAS_PIN);
    int32_t gas_filter_diff = (gas_raw_read - gas_filtered) >> GAS_FILTER_ALPHA;
    gas_filtered = gas_filtered + gas_filter_diff;  //right shift 1 bit = division by 2
    /*
    Serial.println("FILTER: ");
    Serial.println(gas_raw_read);
    Serial.println(gas_filter_diff);
    Serial.println(gas_filtered);
    Serial.println("---------");
    */
  }
}

void loop()
{
  loop_begin_micros = micros();
  Serial.print("Loop Time: ");
  Serial.println(loop_time);
  
  int32_t gas_raw = analogRead(0);
  Serial.print("GAS, RAW: ");
  Serial.println(gas_raw);
  read_gas();
  Serial.print("GAS, FILTERED: ");
  Serial.println(gas_filtered);
  
  int32_t gas = map(gas_filtered, 172, 862, 0, 100000); //convert 0-1023 to 1/1000th of a percent
  if(gas < 0) {
    gas = 0;
  }
  Serial.print("GAS, PERCENT: ");
  Serial.println(gas);
  byte sndStat = comm_can_set_current_rel(1, gas);
  if(sndStat == CAN_OK){
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
  
  loop_end_micros = micros();
  loop_time = loop_end_micros - loop_begin_micros;
  delay(LOOP_TIME_STEP);   // send data per 100ms
}

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


byte comm_can_set_current_rel(uint8_t controller_id, int32_t current) {
        int32_t send_index = 0;  //incremented by buffer_append()
        uint8_t send_buffer[4];
        buffer_append_int32(send_buffer, current, &send_index);
        return CAN0.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), 1, send_index, send_buffer);
        //returns sendStat
        
}

byte comm_can_set_duty(uint8_t controller_id, int32_t duty) {
        int32_t send_index = 0;  //incremented by buffer_append()
        uint8_t send_buffer[4];
        buffer_append_int32(send_buffer, duty, &send_index);
        return CAN0.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), 1, send_index, send_buffer);
        //returns sendStat
        
}


//utility functions

void led_flash_all(int on_time, int count) {
  for(int i=0; i<count; i++) {
    digitalWrite(LED_F, HIGH);
    digitalWrite(LED_RX, HIGH);
    digitalWrite(LED_TX, HIGH);
    delay(on_time);
    digitalWrite(LED_F, LOW);
    digitalWrite(LED_RX, LOW);
    digitalWrite(LED_TX, LOW);
    if(i < count-1) {
      delay(on_time);
    }
  }
}

int32_t middle_of_3(int32_t a, int32_t b, int32_t c)
{
 int32_t middle;

 if ((a <= b) && (a <= c))
 {
   middle = (b <= c) ? b : c;
 }
 else if ((b <= a) && (b <= c))
 {
   middle = (a <= c) ? a : c;
 }
 else
 {
   middle = (a <= b) ? a : b;
 }
 return middle;
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
        buffer[(*index)++] = number >> 24;
        buffer[(*index)++] = number >> 16;
        buffer[(*index)++] = number >> 8;
        buffer[(*index)++] = number;
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
