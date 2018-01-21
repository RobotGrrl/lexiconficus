/*******************
Promulgate
-----------

This library lets other devices trigger actions using a very simple API.
You can also send commands to other devices to trigger actions.

Requires the Streaming and SoftwareSerial libraries.
(SoftwareSerial is included with Arduino now)

Erin K / RobotGrrl - May 21, 2014
Last update Jan. 2, 2018
--> http://RobotMissions.org
--> http://RobotGrrl.com/blog
--> http://RoboBrrd.com

MIT license, check LICENSE for more information
All text above must be included in any redistribution
*******************/

#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Streaming.h"
#include <Stream.h>

#ifndef _PROMULGATE_H_
#define _PROMULGATE_H_

class Promulgate {

	public:

		Promulgate();
		Promulgate(Stream *in, Stream *out);

		enum Level {
		  ERROR_,
		  WARN,
		  INFO,
		  DEBUG
		} the_level;

		Level LOG_LEVEL;

		void set_debug_stream(Stream *db);
		void set_rx_callback( void (*rxCallback)(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) );
		void set_tx_callback( void (*txCallback)() );
		void useBase64Parsing(bool b);

		void parse_message(char msg[], uint8_t len);
		void parse_message64(char msg[], uint8_t len);
		void organize_message(char c);
		
		void transmit_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);


	private:

		boolean          reading_message;
		char             ser[30];
		uint8_t          ser_len;
		bool             use_base64_parsing;

		Stream *in_stream;
		Stream *out_stream;
		Stream *debug_stream;

		// typedef enum Outputter {
		//   HARDWARE_SERIAL,
		//   SOFTWARE_SERIAL
		// };

		// Outputter        OUTPUT_METHOD;

		void begin();
		void reset_buffer();

		void (*_rxCallback)(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
		void (*_txCallback)();

};

#endif