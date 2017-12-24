#include "BowieLogger.h"

bool BowieLogger::initSd() {

  // From the SD CardInfo example code
  // created  28 Mar 2011
  // by Limor Fried 
  // modified 9 Apr 2012
  // by Tom Igoe

  Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    return false;
  } else {
   Serial.println("Wiring is correct and a card is present."); 
  }

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return false;
  } else {
    Serial.println("card initialized.");
  }

  // print the type of card
  Serial.print("\nCard type: ");
  switch(card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  return true;
  
}


void BowieLogger::getAllFiles() {

  // from the SD listfiles example code
  // created   Nov 2010
  // by David A. Mellis
  // modified 9 Apr 2012
  // by Tom Igoe

  File root;
  root = SD.open("/");
  printDirectory(root, 0);
}

void BowieLogger::printDirectory(File dir, int numTabs) {

  // from the SD listfiles example code
  // created   Nov 2010
  // by David A. Mellis
  // modified 9 Apr 2012
  // by Tom Igoe
  
   while(true) {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}


uint32_t BowieLogger::getSizeUsed() {

  // returns kb
  // tends to over-calculate the size used
  // (as compared with Finder)

  File root;
  root = SD.open("/");
  uint32_t totalsize = enterNextDir(root);
  uint32_t totalsize_kb = 0;

  Serial.print("\n\n");

  Serial.print("Total size (bytes): ");
  Serial.println(totalsize);
  totalsize = ceil(totalsize/1024);
  totalsize_kb = totalsize;
  Serial.print("Total size (Kbytes): ");
  Serial.println(totalsize);
  totalsize = ceil(totalsize/1024);
  Serial.print("Total size (Mbytes): ");
  Serial.println(totalsize);

  return totalsize_kb;

}

uint32_t BowieLogger::enterNextDir(File dir) {
  uint32_t totalsize = 0;
  while(true) {
     
     File entry =  dir.openNextFile();
     if (!entry) {
       break;
     }
     
     if (entry.isDirectory()) {
       totalsize += enterNextDir(entry);
     } else {
       totalsize += entry.size();
     }
     entry.close();
   }
   return totalsize;
}

uint32_t BowieLogger::getSizeTotal() {

  // returns kb
  
  // From the SD CardInfo example code
  // created  28 Mar 2011
  // by Limor Fried 
  // modified 9 Apr 2012
  // by Tom Igoe

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    return 0;
  }
  
  // print the type and size of the first FAT-type volume
  uint32_t volumesize = 0;
  uint32_t volumesize_kb = 0;
  Serial.print("\nVolume type is FAT");
  Serial.println(volume.fatType(), DEC);
  Serial.println();
  
  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  if (volumesize < 8388608ul) {
    Serial.print("Volume size (bytes): ");
    Serial.println(volumesize * 512);        // SD card blocks are always 512 bytes
  }
  Serial.print("Volume size (Kbytes): ");
  volumesize /= 2;
  volumesize_kb = volumesize;
  Serial.println(volumesize);
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);

  return volumesize_kb;
  
}

float BowieLogger::getPercentAvailable() {

  float n = ( (float)getSizeUsed() / (float)getSizeTotal() ) * 100.0;
  n = 100.0-n;
  Serial.print("\n\n");
  Serial.print("Percent available: ");
  Serial.println(n);
  return n;
  
}

void BowieLogger::sendEntireFile(String filename, Stream *s) {

  // from the SD example code - DumpFile
  // created  22 December 2010
  // by Limor Fried
  // modified 9 Apr 2012
  // by Tom Igoe

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(filename.c_str());

  // if the file is available, write to it:
  if (dataFile) {
    while (dataFile.available()) {
      s->write(dataFile.read());
    }
    dataFile.close();
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.print("error opening ");
    Serial.println(filename);
  } 
  
}
