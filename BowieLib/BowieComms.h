/*
 * Robot Missions Bowie Communications
 * ------------------------------------
 *
 * Communications protocol, parsing, and stack
 * for Bowie the Robot Missions robot.
 * Primarily for usage with the Xbee on board.
 *
 * Erin RobotGrrl for Robot Missions
 * --> http://RobotMissions.org
 *
 * Using Teensy 3.6, works with Teensy 3.2
 * 
 * Erin RobotGrrl
 * Dec. 24th, 2017
 *
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 *
 * Special thanks to this forum post: 
 * http://forum.arduino.cc/index.php?topic=311968.msg2164769#msg2164769
 * For the example of how to get a callback to reach a non-static function
 * by using a variable of the class instance.
 */

#include "Arduino.h"
#include <Streaming.h>
#include <XBee.h>
#include "PromulgateBig.h"

#ifndef _BOWIECOMMS_H_
#define _BOWIECOMMS_H_

#define ROBOT_ID 3

#define PROG_DEBUG true
#define COMM_DEBUG false
#define OP_DEBUG false

// Operator controllers
#define MAX_CONTROLLERS 10

// Messages
#define MSG_QUEUE_SIZE 3
#define REMOTE_OP_TIMEOUT 300
#define MAX_PERIODIC_MESSAGES 10

// Vars
#define HEARTBEAT_MS 1000

struct Packet {
  char cmd;
  uint8_t key;
  uint16_t val;
};

struct Msg {
  uint8_t priority;
  char action;
  Packet pck1;
  Packet pck2;
  char delim;
};

class BowieComms {

  static BowieComms *bcInstance;
  static void received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
  static void transmit_complete();

  public:

    int counter;

    BowieComms();
    void initComms();
    void updateComms();
    void setCommLed(uint8_t pin);
    unsigned long getCommLatency();

    // Callbacks
    void set_comms_timeout_callback( void (*commsTimeoutCallback)() );
    void set_controller_added_callback( void (*controllerAddedCallback)() );
    void set_controller_removed_callback( void (*controllerRemovedCallback)() );
    void set_received_action_callback( void (*receivedActionCallback)(Msg m) );

    // Promulgate
    Promulgate promulgate;
    unsigned long current_time;
    unsigned long diff_time;
    unsigned long last_rx_msg;
    unsigned long last_transmit;

    // Xbee
    XBee xbee = XBee();
    XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x0000ffff);
    ZBTxStatusResponse txStatus = ZBTxStatusResponse();
    ZBRxResponse rx = ZBRxResponse();
    char message_tx[64];
    char message_rx[64];
    uint32_t msg_tx_count;
    uint32_t msg_rx_count;
    uint32_t msg_tx_err;
    unsigned long last_rx;
    unsigned long last_tx;

    // Xbee Operator Controllers
    XBeeAddress64 addr_all_controllers[MAX_CONTROLLERS];
    uint8_t num_addrs;
    uint8_t ind_addr_sent;
    uint16_t addr_remote_controllers[MAX_CONTROLLERS];
    uint8_t failed_send_count[MAX_CONTROLLERS];
    long last_rx_all[MAX_CONTROLLERS];
    long last_rx_check;
    long last_led_blink;
    bool led_on;
    long last_retry_time;

    // Promulgate
    void sendNextMsg();
    void processAction(Msg m);
    void transmitDidComplete();

    // Messages
    Packet pck_none = { '0', 0, 0 };
    Msg msg_none = { 9, '^', pck_none, pck_none, '!' };
    Msg periodic_messages[MAX_PERIODIC_MESSAGES];
    uint8_t msg_send_items;
    uint8_t getMsgQueueLength();
    Msg popNextMsg();
    void addMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
    void addMsg(Msg m);
    void insertMsg(Msg m);
    void chooseNextMessage();
    void addPeriodicMessage(Msg m);
    void updatePeriodicMessage(Msg m);
    void removePeriodicMessage(uint8_t remove_ind);
    void removePeriodicMessage(Msg m);

    

    // Xbee
    void xbeeSend(Msg m);
    void xbeeSend(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
    void xbeeSendEasy(char c);

  private:

    // Callbacks
    void (*_commsTimeoutCallback)();
    void (*_controllerAddedCallback)();
    void (*_controllerRemovedCallback)();
    void (*_receivedActionCallback)(Msg m);

    // Custom
    uint8_t COMM_LED;

    // Comms
    uint8_t msgs_in_queue;
    uint8_t msg_send_index;
    Msg msg_queue[MSG_QUEUE_SIZE];
    unsigned long last_rx_comms;
    uint8_t unlikely_count = 0;

    // Xbee
    void addXbeeToList(XBeeAddress64 newAddr);
    void updateRxTime(XBeeAddress64 senderLongAddress);
    void xbeeWatchdog();
    bool xbeeRead();
    void xbeeBlink();
    void print32Bits(uint32_t dw);
    void print16Bits(uint16_t w);
    void print8Bits(byte c);

};

#endif

