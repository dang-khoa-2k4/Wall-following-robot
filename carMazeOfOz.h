#ifndef carMazeOfOz_h
#define carMazeOfOz_h
#include <Arduino.h>

class carMazeOfOz {
  public:
    carMazeOfOz();
    void setPin();
    void setInterrupt();
    void setMotorLeft(byte speed, bool direction);
    void setMotorRight(byte speed, bool direction);
    void setSpeedLeft(volatile float speed);
    void setSpeedRight(volatile float speed);
    float getSpeedLeft();
    float getSpeedRight();
    float getDistanceHead();
    float getDistanceLeft();
    float getDistanceRight();
    float getRateOfChangeDistanceHead();
    float getRateOfChangeDistanceLeft();
    float getRateOfChangeDistanceRight();
    void configureSpeed(volatile float &speedValueLeft, volatile float &speedValueRight);
  private:
    volatile float speedValueLeft, speedValueRight;
};

#endif
