void updateRobotsPeriodicMessages() {
  // Updating our periodic messages

  random_periodic1.priority = 10;
  random_periodic1.action = '@';
  random_periodic1.pck1.cmd = 'Z';
  random_periodic1.pck1.key = 1;
  random_periodic1.pck1.val = (int)random(0, 1024);
  random_periodic1.pck2.cmd = 'Z';
  random_periodic1.pck2.key = 1;
  random_periodic1.pck2.val = (int)random(0, 1024);
  random_periodic1.delim = '!';

  random_periodic2.priority = 9;
  random_periodic2.action = '@';
  random_periodic2.pck1.cmd = 'Z';
  random_periodic2.pck1.key = 1;
  random_periodic2.pck1.val = (int)random(1000, 1024);
  random_periodic2.pck2.cmd = 'Z';
  random_periodic2.pck2.key = 1;
  random_periodic2.pck2.val = (int)random(1000, 1024);
  random_periodic2.delim = '!';

  bowiecomms_xbee.updatePeriodicMessage(random_periodic1);
  bowiecomms_xbee.updatePeriodicMessage(random_periodic2);

  bowiecomms_arduino.updatePeriodicMessage(random_periodic1);
  bowiecomms_arduino.updatePeriodicMessage(random_periodic2);
  
}

