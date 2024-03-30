#pragma once

class Encoder
{
private:
    int pin0;
    int pin1;
    volatile int value;
    int currentState;
    int lastState;
    int cpr;
    unsigned long prev_time;

public:
    Encoder(int pin0, int pin1, int cpr, bool reversed);
    ~Encoder();
    int get_count();
    double get_rev();
    double get_rpm();
    void encoderUpdate();
};