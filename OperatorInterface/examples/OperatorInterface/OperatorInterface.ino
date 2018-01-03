#include "OperatorInterface.h"

// ---- Operator
Operator controller = Operator();

// ---- Communication
// Xbee on Serial1
Promulgate promulgate = Promulgate(&Serial1, &Serial1);

//void transmit_complete();
//void received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);

// ---- Xbee variables
#define XBEE_COORDINATOR_DH 0x00000000
#define XBEE_COORDINATOR_DL 0x00000000 // to coordinator
//#define XBEE_COORDINATOR_DL 0x0000FFFF // broadcast

#define MAX_ROBOTS 6

XBee xbee = XBee();
XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x0000ffff);
XBeeAddress64 addr_coord = XBeeAddress64(XBEE_COORDINATOR_DH, XBEE_COORDINATOR_DL);
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
ZBRxResponse rx = ZBRxResponse();
char message_tx[64];
char message_rx[64];
uint32_t msg_tx_count = 0;
uint32_t msg_rx_count = 0;
uint32_t msg_tx_err = 0;
long last_rx = 0;
long last_retry_time = 0;

XBeeAddress64 addr_all_robots[MAX_ROBOTS];
uint8_t num_addrs = 0;
uint8_t ind_addr_sent;
uint8_t failed_send_count[MAX_ROBOTS];
uint16_t ids_of_all_robots[MAX_ROBOTS];
long last_rx_all[MAX_ROBOTS];
long last_rx_check = 0;

int led = 13;
long last_led_blink = 0;
bool led_on = false;

// ---- Messages
Msg msg_none = { 9, '^', '0', 0, 0, '0', 0, 0, '!' };
Msg msg_a = { 8, '@', 'A', 1, 1, 'Z', 8, 8888, '!' };
Msg msg_b = { 8, '@', 'A', 0, 0, 'Z', 0, 0, '!' };

#define DEFAULT_RETRY_TIME 250
#define SECONDARY_RETRY_TIME 500

unsigned long last_rx_msg = 0;
unsigned long current_time = 0;
unsigned long diff_time = 0;
uint16_t retry_time = DEFAULT_RETRY_TIME;
uint8_t retry_count = 0;

// ---- Other
bool a_or_b = false;
boolean light_on = false;
boolean has_started = false;

bool SELECTED_ROBOT = false;
uint16_t SELECTED_ROBOT_ID[MAX_ROBOTS];
int num_robot_conn = 0;
XBeeAddress64 selected_robot_addr;



void setup() {
  // ---- 1. serial inits
  delay(1000); // this seems to make the setup function run with the prints
  Serial.begin(9600);
  Serial1.begin(9600);
  xbee.begin(Serial1);
  Serial.println(F("Operator Xbee Test"));

  // ---- 2. promulgate & xbees
  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  //promulgate.set_rx_callback(received_action);
  //promulgate.set_tx_callback(transmit_complete);
  promulgate.set_debug_stream(&Serial);

  //xbeeVarsInit();

  // ---- 3. operator
  controller.init();

}

void loop() {
  controller.update();
}
