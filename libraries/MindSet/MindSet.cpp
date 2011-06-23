#include "MindSet.h"

// ADDED FOR COMPATIBILITY WITH WIRING
extern "C" {
  #include <stdlib.h>
}

void MindSet::attach(mindsetCallbackFunction newFunction) {
  callback = newFunction;
}

void MindSet::reset() {
  bufferIndex = 0;
  current = NULL;
  last = NULL;
  payloadLength = 0;
  numSyncBytes = 0;
  checksum = 0;
}

void MindSet::init() {
  callback = NULL;
  mErrorRate = 255;
  reset();
}


uint8_t MindSet::process(uint8_t serialByte) {
  // Process the next byte coming in from the MindSet and check 
  // to see if there might be a complete packet. If so, parse it.
  
  // states: waiting for packet start or building packet
  // If waiting, then just discard bytes until we get a valid header 
  // (>=2 170s, and then a valid packet size). Once we get a valid
  // header size save it and go into building mode.
  // If building mode, add new bytes to buffer, then check to see if
  // we have a complete packet (bufferSize == packetSize). If so,
  // trigger the packet parser, call the callback, and reset our state.
  
  /*
  [SYNC] [SYNC] [PLENGTH]    [PAYLOAD...]    [CHKSUM]
  _______________________   _____________  ____________
  ^^^^^^^^(Header)^^^^^^^   ^^(Payload)^^  ^(Checksum)^

  The [PAYLOAD...] section is allowed to be up to 169 bytes long, while each of [SYNC], [PLENGTH],
  and [CHKSUM] are a single byte each. This means that a complete, valid Packet is a minimum of 4
  bytes long (possible if the Data Payload is zero bytes long, i.e. empty) and a maximum of 173 bytes
  long (possible if the Data Payload is the maximum 169 bytes long).
  */
  
  if(payloadLength==0){
    // We haven't gotten the size byte yet- still waiting for start of packet.
    // Count the sync bytes. There should be at least 2. After 2 or more sync
    // bytes, we should get a packet size byte. Payload size is always < 170.
    if(serialByte==170)
      numSyncBytes++;
    else if(numSyncBytes>2 && serialByte<170)
      payloadLength = serialByte;
  }else if(bufferIndex<payloadLength){
    // Filling the buffer for a packet; add the new byte to the buffer.
    buffer[bufferIndex] = serialByte;
    checksum += serialByte;
    bufferIndex++;
  }else{
    // payloadLength is > 0 and bufferIndex == payloadLength, so
    // the packet is complete! Parse it and then reset the buffer.
    // Note that the byte following the payload (current byte) is the checksum byte. 
    // one's compliment of the computed checksum should == the checksum byte
    if(255-checksum != serialByte) {  
      // *** checksum error ***
      // *** do something useful here! Maybe show an error?
    }else{
      parsePayload();
      if(callback != NULL)
        (*callback)();
    }
    reset();
  }
}

/*
 * Parse the payload.
 */
void MindSet::parsePayload() {
  uint8_t vLength;
  uint8_t powerLength = 3; // defined in MindSet Communications Protocol
  uint8_t k;
  uint8_t success;

    for(uint8_t i = 0; i < payloadLength; i++){
      switch (buffer[i]) {
        case 2:
            mBigPacket = true;            
            i++;            
            mErrorRate = buffer[i];         
            break;
          case 4:
            i++;
            mAttention = buffer[i];                        
            break;
          case 5:
            i++;
            mMeditation = buffer[i];
            break;
          case 0x80: // raw data
            i++;
            vLength = buffer[i]; 
            mRaw = 0;
            for (int j=0; j<vLength; j++) {
              mRaw = mRaw | ( buffer[i+vLength-j]<<(8*j) ); // bit-shift little-endian
            }
            i += vLength;
            break;
          case 0x83:  // power data
            i++;
            vLength = buffer[i]; 
            k = 0;
            
            // parse power data starting at the last uint8_t
            mGamma2 = 0; // mid-gamma (41 - 49.75Hz)
            for (int j=0; j<powerLength; j++) {
              mGamma2 = mGamma2 | ( buffer[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mGamma1 = 0; // low-gamma (31 - 39.75Hz)
            for (int j=0; j<powerLength; j++) {
              mGamma1 = mGamma1 | ( buffer[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mBeta2 = 0; // high-beta (18 - 29.75Hz)
            for (int j=0; j<powerLength; j++) {
              mBeta2 = mBeta2 | ( buffer[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mBeta1 = 0; // low-beta (13 - 16.75Hz)
            for (int j=0; j<powerLength; j++) {
              mBeta1 = mBeta1 | ( buffer[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mAlpha2 = 0; // high-alpha (10 - 11.75Hz)
            for (int j=0; j<powerLength; j++) {
              mAlpha2 = mAlpha2 | ( buffer[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mAlpha1 = 0; // low-alpha (7.5 - 9.25Hz)
            for (int j=0; j<powerLength; j++) {
              mAlpha1 = mAlpha1 | ( buffer[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mTheta = 0; // theta (3.5 - 6.75Hz)
            for (int j=0; j<powerLength; j++) {
              mTheta = mTheta | ( buffer[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            mDelta = 0; // delta (0.5 - 2.75Hz)
            for (int j=0; j<powerLength; j++) {
              mDelta = mDelta | ( buffer[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            
            i += vLength;
            break;          
          default:
            break;
          } // switch
        } // for loop
}




