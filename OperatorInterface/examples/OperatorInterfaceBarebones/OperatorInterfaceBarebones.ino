#include "OperatorInterface.h"

Operator the_operator = Operator();

void setup() {
  Serial.begin(9600);
  
  the_operator.initOperator(XBEE_CONN);

}

void loop() {

  the_operator.updateOperator();

}
