void experiment() {

  int down_pos = 150;
  boolean did_touchdown = false;
  int iter = 0;
  boolean paused = false;
  int level_end_pos = 0;

  while(1<3) {

    int top_l = analogRead(FSR_TOP_L);
    int top_r = analogRead(FSR_TOP_R);
    int bot_l = analogRead(FSR_BOT_L);
    int bot_r = analogRead(FSR_BOT_R);

    Serial << "top L: " << top_l << " R: " << top_r;
    Serial << "bot L: " << bot_l << " R: " << bot_r;
    Serial << endl;
    delay(100);

    bowie.moveArm(ARM_MIN+down_pos);

    for(level_end_pos=END_MIN; level_end_pos<END_PARALLEL_BOTTOM+200; level_end_pos+=10) {
      bowie.end.writeMicroseconds( level_end_pos );
      
      top_l = analogRead(FSR_TOP_L);
      top_r = analogRead(FSR_TOP_R);
      bot_l = analogRead(FSR_BOT_L);
      bot_r = analogRead(FSR_BOT_R);

      delay(20);

      Serial << "bot L: " << bot_l << " R: " << bot_r << endl;
      
      if(bot_l < 300) {
        Serial << "touchdown!" << endl;
        did_touchdown = true;
      }
    }

    if(did_touchdown) {
      Serial << "Level! Took " << iter << " iterations - pos = " << level_end_pos << endl;
      paused = true;
    } else {
      down_pos -= 10;
      if(down_pos < 0) {
        paused = true;
      }
      iter++;
    }

    while(paused) {
      if(!did_touchdown) Serial << "paused" << endl;
      delay(100);
    }

  }
  
}

