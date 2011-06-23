#include <WProgram.h>

#include <inttypes.h>

#define MINDSETBUFFERSIZE 170
#define MINDSET_WAITING 1
#define MINDSET_FILLING 2

extern "C" {
// callback function
    typedef void (*mindsetCallbackFunction)(void);
}


class MindSet {
  public:
    MindSet(int timeoutIn) : timeout(timeoutIn) { init(); }
    MindSet() : timeout(0) { init(); }

    uint8_t process(uint8_t serialByte);
    
    void attach(mindsetCallbackFunction newFunction);

    uint8_t errorRate() const { return(mErrorRate); }
    uint8_t attention() const { return(mAttention); }
    uint8_t meditation() const { return(mMeditation); }
    short raw() const { return(mRaw); }
    unsigned int delta() const { return(mDelta); }
    unsigned int theta() const { return(mTheta); }
    unsigned int alpha1() const { return(mAlpha1); }
    unsigned int alpha2() const { return(mAlpha2); }
    unsigned int beta1() const { return(mBeta1); }
    unsigned int beta2() const { return(mBeta2); }
    unsigned int gamma1() const { return(mGamma1); }
    unsigned int gamma2() const { return(mGamma2); }

 private:
  void reset();
  void init();
  void parsePayload();
 
  uint8_t mBigPacket;
  uint8_t mErrorRate;
  uint8_t mAttention;
  uint8_t mMeditation;
  short mRaw;
  unsigned int mDelta;
  unsigned int mTheta;
  unsigned int mAlpha1;
  unsigned int mAlpha2;
  unsigned int mBeta1;
  unsigned int mBeta2;
  unsigned int mGamma1;
  unsigned int mGamma2;

  int timeout;
  uint8_t payloadLength;
  uint8_t numSyncBytes;
  uint8_t checksum;

  mindsetCallbackFunction callback;
  
  uint8_t buffer[MINDSETBUFFERSIZE];
  uint8_t* current; // Pointer to current data
  uint8_t* last;
  uint8_t bufferIndex;
};
