#include "GyroManager.h"
#include "FunctionType.h"
#include <Arduino.h>

GyroManager::GyroManager(volatile float* omega, functiontype updateGyro)
{
    _omega = omega;
    _updateGyro = updateGyro;
}

volatile float* GyroManager::getOmega()
{
    return _omega;
}
