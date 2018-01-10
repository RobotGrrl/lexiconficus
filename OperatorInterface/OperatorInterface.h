/*
 * Robot Missions Operator Interface
 * ---------------------------------
 *
 * Communications with the robot using Robot Missions's
 * hardware interface. Uses an Xbee to connect with the
 * robot. Xbee is on Serial1.
 *
 * Erin RobotGrrl for Robot Missions
 * --> http://RobotMissions.org
 *
 * Using Teensy 3.2, or Teensy LC (with an external 3.3V regulator)
 * 
 * Erin RobotGrrl
 * Jan. 3rd, 2018
 *
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 *
 */

#include "Arduino.h"
#include <Streaming.h>
#include <XBee.h>
#include "PromulgateBig.h"
#include <Bounce2.h>

#ifndef _OPINTERFACE_H_
#define _OPINTERFACE_H_

#define COMM_DEBUG false // anything with promulgate
#define OP_DEBUG true    // anything with buttons, or op in general
#define XBEE_DEBUG false // anything with the xbee scope
#define CONN_DEBUG false // anything with the connection stack
#define MSG_DEBUG false  // anything with adding / removing Msgs

// speeds
#define MAX_SPEED 255
#define MIN_SPEED 120
#define TURN_SPEED_FWD 180
#define TURN_SPEED_REV 180

// button pins
#define BUTTON1 7
#define BUTTON2 8
#define BUTTON3 14
#define BUTTON4 2
#define BUTTON5 3
#define BUTTON6 4

// buttons
#define DEBOUNCE 20

// mode switch
#define MODE_SW A10
#define MODE1_THRESH 1000 // 1023
#define MODE2_THRESH 500  // 512
#define MODE3_THRESH 0    // 1
#define MODE1 1
#define MODE2 2
#define MODE3 3

// led pins
#define BOARD_LED 13
#define LED1 5
#define LED2 6
#define LED3 20
#define LED4 23
#define LED5 22
#define LED6 21

// speaker pin
#define SPEAKER 10

// display pins
#define OLED_RESET 12

// messages
#define MSG_QUEUE_SIZE 3

// joystick
#define JOYSTICK_X 16
#define JOYSTICK_Y 17
#define JOYSTICK_SW 15
#define ACTIVITY_TIMEOUT 1000
#define IDLE_UPDATE_FREQ 250
#define MAX_X 1019  // left
#define MIN_X 6     // right
#define MAX_Y 1023  // up
#define MIN_Y 8     // down
#define ZERO_ZONE 50

// states
#define IDLE_STATE 0
#define SEARCHING_STATE 1
#define ACTIVE_STATE 2

// xbee
#define XBEE_COORDINATOR_DH 0x00000000
#define XBEE_COORDINATOR_DL 0x00000000 // to coordinator
//#define XBEE_COORDINATOR_DL 0x0000FFFF // broadcast

// vars
#define MAX_ROBOTS 6
#define DEFAULT_RETRY_TIME 250
#define SECONDARY_RETRY_TIME 500
#define REMOTE_OP_TIMEOUT 15000

// conns
#define XBEE_CONN 1
#define USB_CONN 2
#define BT_CONN 3

// speaker
// source: https://www.hackster.io/sanyam-chugh/super-mario-beats-on-arduino-5d96a8
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

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

class OperatorInterface {

  static OperatorInterface *opInstance;
  static void received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
  static void transmit_complete();

  public:
    OperatorInterface();

    /** Copy assignment operator */
    OperatorInterface& operator= (const OperatorInterface& other)
    {
        // OperatorInterface tmp(other);         // re-use copy-constructor
        // *this = std::move(tmp); // re-use move-assignment
        return *this;
    }

    /** Move assignment operator */
    OperatorInterface& operator= (OperatorInterface&& other) noexcept
    {
        // if (this == &other)
        // {
        //     // take precautions against `foo = std::move(foo)`
        //     return *this;
        // }
        // delete[] data;
        // data = other.data;
        // other.data = nullptr;
        return *this;
    }

    void begin();
    void setOpID(uint8_t the_op_id);
    void setCommLed(uint8_t pin);
    void setAutoconnect(bool b);
    unsigned long getCommLatency();
    unsigned long getLastRXTime();
    int getMotorSpeed(int m); // 1 = left, 2 = right
    bool getMotorDir(int m);
    int getCurrentMode();
    bool TESTING;
    uint8_t OP_ID;
    bool isConnectedToRobot();

    // Callbacks
    void set_comms_timeout_callback( void (*commsTimeoutCallback)() );
    void set_controller_added_callback( void (*controllerAddedCallback)() );
    void set_controller_removed_callback( void (*controllerRemovedCallback)() );
    void set_received_action_callback( void (*receivedActionCallback)(Msg m) );
    void set_button_changed_callback( void (*buttonChangedCallback)(int button, int value) );
    void set_mode_changed_callback( void (*modeChangedCallback)(int mode) );
    void set_robot_added_callback( void (*robotAddedCallback)() );
    void set_robot_removed_callback( void (*robotRemovedCallback)(bool still_connected) );

    // Init & Update
    HardwareSerial *serialyeah;
    void initOperator(int conn, long baud, HardwareSerial *serial);
    void updateOperator();

    // Promulgate
    Promulgate promulgate;
    unsigned long current_time;
    unsigned long diff_time;
    unsigned long last_rx_msg;
    unsigned long last_transmit;
    bool use_base64_parsing; // false by default
    uint16_t retry_time = DEFAULT_RETRY_TIME;
    uint8_t retry_count = 0;

    // Xbee
    XBee xbee;
    XBeeAddress64 addr64;
    XBeeAddress64 addr_coord;
    XBeeAddress64 addr_robot;
    XBeeAddress64 addr_fake;
    ZBTxStatusResponse txStatus;
    ZBRxResponse rx;
    char message_tx[32];
    char message_rx[32];
    uint32_t msg_tx_count;
    uint32_t msg_rx_count;
    uint32_t msg_tx_err;
    unsigned long last_rx;
    unsigned long last_tx;
    unsigned long last_retry_time;

    // Xbee Robots
    XBeeAddress64 addr_all_robots[MAX_ROBOTS];
    uint8_t num_addrs = 0;
    uint8_t ind_addr_sent;
    uint8_t failed_send_count[MAX_ROBOTS];
    uint16_t ids_of_all_robots[MAX_ROBOTS];
    long last_rx_all[MAX_ROBOTS];
    long last_rx_check = 0;

    bool SELECTED_ROBOT = false;
    uint16_t SELECTED_ROBOT_ID[MAX_ROBOTS];
    int num_robot_conn = 0;
    XBeeAddress64 selected_robot_addr;

    // Promulgate
    void sendNextMsg();
    void processAction(Msg m);
    void transmitDidComplete();

    // Messages
    Packet pck_none = { '0', 0, 0 };
    Msg msg_none = { 9, '^', pck_none, pck_none, '!' };
    uint8_t getMsgQueueLength();
    Msg popNextMsg();
    void addMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
    void addMsg(Msg m);
    void insertMsg(Msg m);

    // Conn
    void connRead();
    void connSend(Msg m);
    void connSend(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
    void connSendEasy(char c);
    void connRetrySend();
    void chooseRobotToConnect(); // added to public in case user wants to call this from their sketch

    // Mode & State
    int CURRENT_STATE; // For continuous states (ie, with joystick)
    int LAST_STATE;
    int CURRENT_MODE; // Mode (operator / semi-autonomous / full autonomous)
    void updateModeSwitch();

    // Joystick
    int HOME_X;
    int HOME_Y;
    bool joystick_on;
    void updateJoystick();
    void calibrateHome();
    int getJoyX();
    int getJoyY();
    void joystickDriveControl();
    void joystickArmControl();
    
    // Buttons
    uint8_t button_states[7];
    bool sticky_buttons;
    bool getButton(uint8_t b);
    void setButtonState(uint8_t b, uint8_t state);
    bool getJoystickButton();
    void resetButtonStates();
    void ledQuickFade(uint8_t pin, uint8_t from, uint8_t to);
    void ledQuickPulseAll();
    
    // LEDs
    long last_led_blink;
    bool led_on;
    void ledsOff();
    void introLedSequence();
    void breatheLeds();

    // Speaker
    void buzz(long frequency, long length);

  private:

    // Callbacks
    void (*_commsTimeoutCallback)();
    void (*_controllerAddedCallback)();
    void (*_controllerRemovedCallback)();
    void (*_receivedActionCallback)(Msg m);
    void (*_buttonChangedCallback)(int button, int value);
    void (*_modeChangedCallback)(int mode);
    void (*_robotAddedCallback)();
    void (*_robotRemovedCallback)(bool still_connected);

    // Custom
    uint8_t COMM_LED;
    uint8_t CONN_TYPE;
    bool AUTOCONNECT; // defaults to true

    // Init
    void initLeds();
    void initButtons();
    void initJoystick();
    void initSpeaker();

    // Comms
    uint8_t msgs_in_queue;
    uint8_t msg_send_index;
    Msg msg_queue[MSG_QUEUE_SIZE];
    uint8_t unlikely_count = 0;

    // Conn
    void connBlink();

    // Xbee
    void xbeeSendEasy(char c);
    void xbeeSend(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
    void xbeeSendToList(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
    void addXbeeToList(XBeeAddress64 newAddr);
    void updateRxTime(XBeeAddress64 senderLongAddress);
    void xbeeWatchdog();
    bool xbeeRead();
    void print32Bits(uint32_t dw);
    void print16Bits(uint16_t w);
    void print8Bits(byte c);

    // Pins
    uint8_t button_pins[7];
    uint8_t led_pins[7];
    Bounce bounce_buttons[7];

    // Misc
    bool turn_on_spot;
    bool slower_speed;
    uint8_t slow_speed;
    uint8_t letter_itr;
    long last_letter_itr;

    // Joystick
    uint16_t joy_x;
    uint16_t joy_y;
    uint16_t joy_x_prev;
    uint16_t joy_y_prev;
    uint16_t joy_sw;
    long last_increment;
    unsigned long last_activity;
    bool scrolling_up;
    
    // Control
    bool motor_l_dir;
    int motor_l_speed;
    bool motor_r_dir;
    int motor_r_speed;
    int motor_speed;
    int turn_speed;
    int incr_speed;
    int arm_pos;
    
    // Buttons
    void updateButtons();

};

/*
 * Notes:
 * The reason for overloading the = operand is because when creating the
 * instance of OperatorInterface in the sketch, it didn't construct all
 * of the variables - or some reason from the behaviour we observed. This
 * is most likely because of the static instance of the class, which is a
 * necessary pointer to be able to call its non-static functions from the 
 * static functions. To get the static instance to work, the instance has
 * to be constructed again from the sketch in the setup function. My guess
 * as to why this is is because of a "static initialization order fiasco"
 * https://isocpp.org/wiki/faq/ctors#static-init-order - this came to my
 * attention through a post by Paul S here: 
 * https://forum.pjrc.com/threads/24157-Passing-Teensy-3-0-hardware-serial-
 * as-an-object?p=66265&viewfull=1#post66265
 * Before confirming that this was indeed the cause of this issue, passing
 * HardwareSerial to the library was also tried. Following this thread
 * was helpful to learn more about the pointers:
 * http://forum.arduino.cc/index.php?topic=181603.0
 * This worked fine, but didn't resolve the issue about the constructor.
 * So, OK, we have to call the constructor again. Not a big deal. There hasn't
 * been any observed issues. 
 * Well, the new problem is that the Adafruit SSD1306 display library (for
 * OLED small screens), has constructors with variables in them, which means
 * the = operand can no longer be used.
 * The error text was that the = operand:
 * "is implicitly deleted because the default definition would be ill-formed".
 * Was initially stuck here for a while, then learned about the 'Rule of Three',
 * or 'Rule of Five' for C++ programming. Luckily on this page, there are 
 * examples of overloading the = operand for a copy and move constructor.
 * https://en.wikipedia.org/wiki/Rule_of_three_(C%2B%2B_programming)
 * Testing this out, adding lines of code back in one by one, it started to
 * compile without errors! The display code had to be added in the sketch,
 * rather than a separate library. It's not that big of a deal - considering
 * for the modularity of the Operator Interface, repurpusing it for other
 * uses would require different things to be displayed on the screen.
 * Now, after initial tests, the observed behaviour appears to be working!
 * Fingers crossed something else does not come up.
 */

#endif