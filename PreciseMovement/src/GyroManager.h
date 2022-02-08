#ifndef GyroManager_h
#define GyroManager_h

#include "FunctionType.h"

// This class manages the encoder data. 
class GyroManager
{
public:
    GyroManager(volatile float* omega, functiontype updateGyro);
    volatile float* getOmega();
    functiontype _updateGyro;

private:
    volatile float* _omega;
};

#endif
