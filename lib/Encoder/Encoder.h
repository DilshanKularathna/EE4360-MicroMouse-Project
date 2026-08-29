#pragma once
// ============================================================================
// Encoder
//
// Quadrature encoder counter. The Mega only exposes 6 interrupt-capable
// pins, and this robot has exactly two encoders, so this class supports
// exactly two live instances via begin(slot) with slot in {0, 1}. Channel A
// is interrupt-driven; channel B is polled inside the ISR to decode
// direction.
// ============================================================================

#include <Arduino.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB);

    // slot must be 0 or 1, and must be unique across your two Encoder
    // instances (see main.cpp).
    void begin(uint8_t slot);

    long getCount() const;
    void reset();

    // Convenience conversion using the wheel geometry from config.h.
    float getDistanceMM(float wheelDiameterMM, uint32_t pulsesPerRev) const;

private:
    uint8_t _pinA, _pinB;
    volatile long _count;

    void onPulse();

    static Encoder *_instances[2];
    static void isrSlot0();
    static void isrSlot1();
};
