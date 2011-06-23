#include "MindSet.h"

/*
 * Reading data from the MindSet
 */
void MindSet::getData() {
  static unsigned char payloadData[256]; 
  byte generatedChecksum;
  byte checksum; 
  byte vLength;
  int payloadLength;
  int powerLength = 3; // defined in MindSet Communications Protocol
  int k;
  
  // Look for sync bytes
  if(readOneByte() == 170) {
    if(readOneByte() == 170) {

      do { payloadLength = readOneByte(); }
      while (payloadLength == 170);
      
      if(payloadLength > 170) {    //Payload length can not be greater than 170
         return;
      }
      
      generatedChecksum = 0;        
      for(int i = 0; i < payloadLength; i++) {  
        payloadData[i] = readOneByte();            //Read payload into memory
        generatedChecksum += payloadData[i];
      }   

      checksum = readOneByte();                      //Read checksum byte from stream      
      generatedChecksum = 255 - generatedChecksum;   //Take one's compliment of generated checksum
      
      if(checksum != generatedChecksum) {  
        // *** checksum error ***
      } else {  
        for(int i = 0; i < payloadLength; i++) {    // Parse the payload
          switch (payloadData[i]) {
          case 2:
            mBigPacket = true;            
            i++;            
            mErrorRate = payloadData[i];         
            break;
          case 4:
            i++;
            mAttention = payloadData[i];                        
            break;
          case 5:
            i++;
            mMeditation = payloadData[i];
            break;
          case 0x80: // raw data
            mNewRawData = true;
            i++;
            vLength = payloadData[i]; 
            mRaw = 0;
            for (int j=0; j<vLength; j++) {
              mRaw = mRaw | ( payloadData[i+vLength-j]<<(8*j) ); // bit-shift little-endian
            }
            i += vLength;
            break;
          case 0x83:  // power data
            i++;
            vLength = payloadData[i]; 
            k = 0;
            
            // parse power data starting at the last byte
            mGamma2 = 0; // mid-gamma (41 - 49.75Hz)
            for (int j=0; j<powerLength; j++) {
              mGamma2 = mGamma2 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mGamma1 = 0; // low-gamma (31 - 39.75Hz)
            for (int j=0; j<powerLength; j++) {
              mGamma1 = mGamma1 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mBeta2 = 0; // high-beta (18 - 29.75Hz)
            for (int j=0; j<powerLength; j++) {
              mBeta2 = mBeta2 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mBeta1 = 0; // low-beta (13 - 16.75Hz)
            for (int j=0; j<powerLength; j++) {
              mBeta1 = mBeta1 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mAlpha2 = 0; // high-alpha (10 - 11.75Hz)
            for (int j=0; j<powerLength; j++) {
              mAlpha2 = mAlpha2 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mAlpha1 = 0; // low-alpha (7.5 - 9.25Hz)
            for (int j=0; j<powerLength; j++) {
              mAlpha1 = mAlpha1 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mTheta = 0; // theta (3.5 - 6.75Hz)
            for (int j=0; j<powerLength; j++) {
              mTheta = mTheta | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mDelta = 0; // delta (0.5 - 2.75Hz)
            for (int j=0; j<powerLength; j++) {
              mDelta = mDelta | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            
            i += vLength;
            break;          
          default:
            break;
          } // switch
        } // for loop
      } // checksum success
    } // sync 2
  } // sync 1
}


/*
 * Read data from BlueTooth Serial UART
 *
 */
int MindSet::readOneByte() {
  while(!btSerial.available());
  return btSerial.read();
}


