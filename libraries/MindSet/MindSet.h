#include <WProgram.h>

#ifndef HardwareSerial
  #define HardwareSerial Serial
#endif

class MindSet {
  public:
    MindSet(HardwareSerial btSerialIn, int baudIn) : btSerial(btSerialIn), baud(baudIn) {}
    MindSet(HardwareSerial btSerialIn) : btSerial(btSerialIn), baud(115200) {}
    //void init(uint8_t switchvcc);
    void getData(byte c);

    byte errorRate() const { return(mErrorRate); }
    byte attention() const { return(mAttention); }
    byte meditation() const { return(mMeditation); }
    short raw() const { return(mRaw); }
    unsigned int delta() const { return(mDelta); }
    unsigned int theta const { return(mTheta); }
    unsigned int alpha1 const { return(mAlpha1); }
    unsigned int alpha2 const { return(mAlpha2); }
    unsigned int beta1 const { return(mBeta1); }
    unsigned int beta2 const { return(mBeta2); }
    unsigned int gamma1 const { return(mGamma1); }
    unsigned int gamma2 const { return(mGamma2); }

    boolean newRawData const { return(mNewRawData); }
    boolean bigPacket const { return(mBigPacket); }

 private:
  byte mErrorRate = 255;
  byte mAttention;
  byte mMeditation;
  short mRaw;
  unsigned int mDelta;
  unsigned int mTheta;
  unsigned int mAlpha1;
  unsigned int mAlpha2;
  unsigned int mBeta1;
  unsigned int mBeta2;
  unsigned int mGamma1;
  unsigned int mGamma2;

  boolean mNewRawData = false;
  boolean mBigPacket = false;

  int baud;
  int readOneByte();

  HardwareSerial btSerial;
};
