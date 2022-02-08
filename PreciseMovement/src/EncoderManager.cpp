#include "EncoderManager.h"
#include "FunctionType.h"
#include <Arduino.h>

EncoderManager::EncoderManager(volatile long* leftEncoderPos, volatile long* rightEncoderPos, int ticksPerRev, functiontype updateEncoders)
{
    _leftPos = leftEncoderPos;
    _rightPos = rightEncoderPos;
    _ticksPerRev = ticksPerRev;
    _updateEncoders = updateEncoders;
}

volatile long* EncoderManager::getLeftPosPointer()
{
    return _leftPos;
}

volatile long* EncoderManager::getRightPosPointer()
{
    return _rightPos;
}

int EncoderManager::getTicksPerRev()
{
    return _ticksPerRev;
}
