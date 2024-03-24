#include <Arduino.h>
#include "encoder.h"

Encoder::Encoder(int pin0, int pin1, int cpr, bool reversed)
{
    if (!reversed)
    {
        this->pin0 = pin0;
        this->pin1 = pin1;
    }
    else
    {
        this->pin0 = pin1;
        this->pin1 = pin0;
    }

    this->cpr = cpr;

    pinMode(this->pin0, INPUT_PULLUP);
    pinMode(this->pin1, INPUT_PULLUP);

    lastState = digitalRead(this->pin0);
}

Encoder::~Encoder()
{
}

int Encoder::get_count()
{
    int return_value = value;
    value = 0;
    return return_value;
}

double Encoder::get_rev()
{
    return (double)get_count() / (double)cpr;
}

void Encoder::encoderUpdate()
{
    currentState = digitalRead(pin0);
    if (currentState != lastState && currentState == 1)
    {
        if (digitalRead(pin1) != currentState)
            value--;
        else
            value++;
    }
    lastState = currentState;
}