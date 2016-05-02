import de.voidplus.myo.*;
import processing.serial.*;

static final boolean DEBUG = false;

Myo myo;

boolean home_pos = false;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

float roll_delta = 0.0;
float pitch_delta = 0.0;
float yaw_delta = 0.0;

float pitch_home = 8.5;
float pitch_home_range = 1.0;
float pitch_max = 11.8;
float pitch_min = 2.5;

float roll_home = 7.0;
float roll_vals_range = 0.4;
float roll_max = 7.2;
float roll_min = 5.5;

float yaw_home = 13.8;
float yaw_home_range = 0.5;
float yaw_min = 4.2;
float yaw_max = 14.5;

void setup() {
  size(1200, 500);
  noStroke();
  
  int port = 0;

  for (int i=0; i<Serial.list().length; i++) {
    println(i + " " + Serial.list()[i]);
    if (Serial.list()[i].equals("/dev/tty.usbmodem1421")) {
      println("ding!");
      connected = true;
      port = i;
    }
  }
  
  arduino = new Serial(this, Serial.list()[port], 9600);
  
  /*
  // future- for disconnecting
    arduino.clear();
    arduino.stop();
    connected = false;
  */
  
  background(255);
  
  myo = new Myo(this);
  //myo.setVerbose(true);
  // myo.setVerboseLevel(4); // Default: 1 (1-4)
  
  myo.setLockingPolicy(Myo.LockingPolicy.NONE);
  
}

void draw() {
  background(255);
  
  if(home_pos) {
    fill(0, 255, 0);
  } else {
    fill(0, 0, 0);
  }
  
  rect(0, 0, width, height);
  
  
  if(connected) {
    fill(128, 0, 128);
    ellipse(0, 0, 100, 100);
  }
  
  fill(255);
  textSize(32);
  
  int yaw_text_x = 30;
  int yaw_text_y = 70;
  text(("yaw: " + yaw), yaw_text_x, yaw_text_y+(30*0));
  text(("yaw delta: " + yaw_delta), yaw_text_x, yaw_text_y+(30*1));
  text(("yaw home: " + yaw_home + " (a)"), yaw_text_x, yaw_text_y+(30*2));
  text(("yaw min: " + yaw_min + " (b)"), yaw_text_x, yaw_text_y+(30*3));
  text(("yaw max: " + yaw_max + " (c)"), yaw_text_x, yaw_text_y+(30*4));
  
  int pitch_text_x = 400;
  int pitch_text_y = 70;
  text(("pitch: " + pitch), pitch_text_x, pitch_text_y+(30*0));
  text(("pitch delta: " + pitch_delta), pitch_text_x, pitch_text_y+(30*1));
  text(("pitch home: " + pitch_home + " (d)"), pitch_text_x, pitch_text_y+(30*2));
  text(("pitch min: " + pitch_min + " (e)"), pitch_text_x, pitch_text_y+(30*3));
  text(("pitch max: " + pitch_max + " (f)"), pitch_text_x, pitch_text_y+(30*4));
  
  int roll_text_x = 770;
  int roll_text_y = 70;
  text(("roll: " + roll), roll_text_x, roll_text_y+(30*0));
  text(("roll delta: " + roll_delta), roll_text_x, roll_text_y+(30*1));
  text(("roll home: " + roll_home + " (g)"), roll_text_x, roll_text_y+(30*2));
  text(("roll min: " + roll_min + " (h)"), roll_text_x, roll_text_y+(30*3));
  text(("roll max: " + roll_max + " (i)"), roll_text_x, roll_text_y+(30*4));
  
  
  // we're not going to be receiving any data
  //readData();
  
}

void keyPressed() {
  switch(key) {
    case 'a':
      yaw_home = yaw;
    break;
    case 'b':
      yaw_min = yaw;
    break;
    case 'c':
      yaw_max = yaw;
    break;
    case 'd':
      pitch_home = pitch;
    break;
    case 'e':
      pitch_min = pitch;
    break;
    case 'f':
      pitch_max = pitch;
    break;
    case 'g':
      roll_home = roll;
    break;
    case 'h':
      roll_min = roll;
    break;
    case 'i':
      roll_max = roll;
    break;
  }
}

// ----------------------------------------------------------

void myoOnPair(Myo myo, long timestamp, String firmware) {
  println("Sketch: myoOnPair");
}

void myoOnUnpair(Myo myo, long timestamp) {
  println("Sketch: myoOnUnpair");
}

void myoOnConnect(Myo myo, long timestamp, String firmware) {
  println("Sketch: myoOnConnect");
}

void myoOnDisconnect(Myo myo, long timestamp) {
  println("Sketch: myoOnDisconnect");
}

void myoOnArmRecognized(Myo myo, long timestamp, Arm arm) {
  println("Sketch: myoOnArmRecognized");

  switch (arm.getType()) {
  case LEFT:
    println("Left arm.");
    break;
  case RIGHT:
    println("Right arm.");
    break;
  }

  if (myo.hasArm()) {
    if (myo.isArmLeft()) {
      println("Left arm.");
    } else {
      println("Right arm.");
    }
  }
}

void myoOnLock(Myo myo, long timestamp){
  println("Sketch: myoOnLock");
}
  
void myoOnUnLock(Myo myo, long timestamp){
  println("Sketch: myoOnUnLock");
}

void myoOnArmUnsync(Myo myo, long timestamp) {
  println("Sketch: myoOnArmUnsync");
}

void myoOnPose(Myo myo, long timestamp, Pose pose) {
  println("Sketch: myoOnPose");
  switch (pose.getType()) {
  case REST:
    println("Pose: REST");
    break;
  case FIST:
    println("Pose: FIST");
    //myo.vibrate(1);
    break;
  case FINGERS_SPREAD:
    println("Pose: FINGERS_SPREAD");
    break;
  case DOUBLE_TAP:
    println("Pose: DOUBLE_TAP");
    break;
  case WAVE_IN:
    println("Pose: WAVE_IN");
    break;
  case WAVE_OUT:
    println("Pose: WAVE_OUT");
    break;
  default:
    break;
  }
}

// pitch = elbow up / down
// roll = hand in / out (twisting your arm)
// yaw = arm close to chest / away
void myoOnOrientation(Myo myo, long timestamp, PVector orientation) {
  //println("Sketch: myoOnOrientation");
  
  roll = orientation.x;
  pitch = orientation.y;
  yaw = orientation.z;
  
  roll_delta = roll-roll_home;
  pitch_delta = pitch-pitch_home;
  yaw_delta = yaw-yaw_home;
  
  //println("roll: " + roll + " pitch: " + pitch + " yaw: " + yaw);
  
  home_pos = false;
  
  if(pitch >= (pitch_home-pitch_home_range) && pitch <= (pitch_home+pitch_home_range)) { // arm is held out (like 90 degrees to ground)
    if(roll >= (roll_max-roll_vals_range) && roll <= (roll_max+roll_vals_range)) { // hand is palm down
      if(yaw >= (yaw_home-yaw_home_range) && yaw <= (yaw_home+yaw_home_range)) { // arm is held out (like 90 degrees to torso)
        println("home!");
        home_pos = true;
      }
    }
  }
  
  // yaw is glitchy
  
  int dir = 99;
  int speed = 0;
  if(roll >= roll_home) {
    // backwards
    dir = 0;
    speed = abs( (int)map(roll, roll_home, roll_max, 0, 255) );
  } else if(roll < roll_home) {
    // forwards
    dir = 1;
    speed = 255 - abs( (int)map(roll, roll_min, roll_home, 0, 255) );
  }
  
  if(speed > 255) speed = 255;
  //speed *= 2;
  
  println("dir: " + dir + " speed: " + speed);
  
  //if(millis() % 100 == 0) {
    println("~");
    
    
    //if(dir == 0) {
      //arduino.write(speed + ";");
      arduino.write(speed + ";");
      //delay(20);
    //} else if(dir == 1) {
    //  arduino.write((speed+1000) + ";");
    //}
    
    
    //arduino.write(128);
    
    //transmit_action('@', 'L', dir, speed, '!');
    //delay(100);
    //transmit_action('@', 'R', dir, speed, '!');
    //delay(20);
  //}
  
}

void myoOnAccelerometer(Myo myo, long timestamp, PVector accelerometer) {
  // println("Sketch: myoOnAccelerometer");
}

void myoOnGyroscope(Myo myo, long timestamp, PVector gyroscope) {
  // println("Sketch: myoOnGyroscope");
}

void myoOnRssi(Myo myo, long timestamp, int rssi) {
  println("Sketch: myoOnRssi");
}

// ----------------------------------------------------------

void myoOn(Myo.Event event, Myo myo, long timestamp) {
  switch(event) {
  case PAIR:
    println("myoOn PAIR");
    break;
  case UNPAIR:
    println("myoOn UNPAIR");
    break;
  case CONNECT:
    println("myoOn CONNECT");
    break;
  case DISCONNECT:
    println("myoOn DISCONNECT");
    break;
  case ARM_SYNC:
    println("myoOn ARM_SYNC");
    break;
  case ARM_UNSYNC:
    println("myoOn ARM_UNSYNC");
    break;
  case POSE:
    println("myoOn POSE");  
    break;
  case ORIENTATION:
    // println("myoOn ORIENTATION");
    PVector orientation = myo.getOrientation();
    break;
  case ACCELEROMETER:
    // println("myoOn ACCELEROMETER");
    break;
  case GYROSCOPE:
    // println("myoOn GYROSCOPE");
    break;
  case RSSI:
    println("myoOn RSSI");
    break;
  }
}