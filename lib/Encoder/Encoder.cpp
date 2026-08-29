#include "Encoder.h"

Encoder *Encoder::_instances[2] = {nullptr, nullptr};

Encoder::Encoder(uint8_t pinA, uint8_t pinB) : _pinA(pinA), _pinB(pinB), _count(0) {}

void Encoder::begin(uint8_t slot) {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    _instances[slot] = this;
    if (slot == 0) {
        attachInterrupt(digitalPinToInterrupt(_pinA), isrSlot0, RISING);
    } else {
        attachInterrupt(digitalPinToInterrupt(_pinA), isrSlot1, RISING);
    }
}

void Encoder::onPulse() {
    // Quadrature decode: direction is given by the level of channel B
    // at the moment channel A rises.
    if (digitalRead(_pinB) == HIGH) _count++;
    else _count--;
}

void Encoder::isrSlot0() { if (_instances[0]) _instances[0]->onPulse(); }
void Encoder::isrSlot1() { if (_instances[1]) _instances[1]->onPulse(); }

long Encoder::getCount() const { return _count; }

void Encoder::reset() {
    noInterrupts();
    _count = 0;
    interrupts();
}

float Encoder::getDistanceMM(float wheelDiameterMM, uint32_t pulsesPerRev) const {
    float circumferenceMM = PI * wheelDiameterMM;
    return (static_cast<float>(_count) / static_cast<float>(pulsesPerRev)) * circumferenceMM;
}
