#ifndef EncoderManager_h
#define EncoderManager_h

#include "FunctionType.h"

// This class manages the encoder data. 
class EncoderManager
{
public:
    EncoderManager(volatile long* leftEncoderPos, volatile long* rightEncoderPos,
                    int ticksPerRev, functiontype updateEncoders);
    volatile long* getLeftPosPointer();
    volatile long* getRightPosPointer();
    int getTicksPerRev();
    functiontype _updateEncoders;

private:
    volatile long* _leftPos;
    volatile long *_rightPos;
    int _ticksPerRev;
};

#endif
