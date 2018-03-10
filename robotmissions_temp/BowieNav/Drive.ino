void robotDriveTrackHeading() {

  if(PREV_DRIVE_STATE != DRIVE_STATE) {
    // TODO: blink the leds or something
    
    // we know we can reset this flag on state change
    bowie.bowiedrive.resetTurnSequence();
  }

  switch(DRIVE_STATE) {
    case IDLE:
      bowie.bowiedrive.goSpeed(true, 0, 1); // fwd, speed, del
    break;
    case FORWARD:
      bowie.bowiedrive.goSpeed(true, true, 255, 1); // fwd, left, speed, del
      bowie.bowiedrive.goSpeed(true, false, 255, 1); // fwd, right, speed, del
    break;
    case RIGHT:
      bowie.bowiedrive.goSpeed(true, true, 255, 1); // fwd, left, speed, del
      bowie.bowiedrive.goSpeed(false, false, 255, 1); // fwd, right, speed, del
    break;
    case HARD_RIGHT:
      bowie.bowiedrive.turnSequence(true);
    break;
    case LEFT:
      bowie.bowiedrive.goSpeed(false, true, 255, 1); // fwd, left, speed, del
      bowie.bowiedrive.goSpeed(true, false, 255, 1); // fwd, right, speed, del
    break;
    case HARD_LEFT:
      bowie.bowiedrive.turnSequence(false);
    break;
    case REVERSE:
      bowie.bowiedrive.goSpeed(false, true, 255, 1); // rev, left, speed, del
      bowie.bowiedrive.goSpeed(false, false, 255, 1); // rev, right, speed, del
    break;
  }
   
}

