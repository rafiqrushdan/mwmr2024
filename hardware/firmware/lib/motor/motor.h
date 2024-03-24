#pragma once

class Motor
{
private:
public:
    int leftPin;
    int rightPin;
    Motor(int, int);
    void setSpeed(int);
};