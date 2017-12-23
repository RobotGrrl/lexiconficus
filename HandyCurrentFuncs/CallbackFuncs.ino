void waitingToCoolDown(bool first) {
  // you might want to detach the servos here
  // (or de-activate the dc motors)
  // first == true when it's called the first time
}

void reactivateAfterCoolDown() {
 // you might want to re-attach the servos here
 // and send them back to a position
 // (or re-activate the dc motors)
}

void overCurrentThreshold(bool first) {
  // you might want to move the robot, it might
  // be in a bad position.
  // as well, de-activate the servos / motors.
  // PS: waitingToCoolDown() will be called prior to
  // this function
  // first == true when it's called the first time
}

