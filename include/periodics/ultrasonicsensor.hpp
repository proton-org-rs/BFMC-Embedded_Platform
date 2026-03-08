#pragma once

#include "mbed.h"
#include "utils/task.hpp"
#include <string>

namespace periodics {

/**
 * @brief Periodic task for the HC-SR04 ultrasonic distance sensor.
 *
 * Publishes a single `1` over serial when an object enters the detection
 * zone, and a single `0` when it leaves — i.e. only on state transitions.
 *
 * Wire-up:
 *   TRIG pin  → DigitalOut  (10 µs pulse triggers measurement)
 *   ECHO pin  → DigitalIn   (pulse width proportional to distance)
 *
 * NOTE: The HC-SR04 ECHO line is 5 V; use a voltage divider or level
 *       shifter to protect the 3.3 V STM32 GPIO pin.
 */
class CUltrasonicsensor : public utils::CTask {
public:
    /**
     * @param f_period       Task execution period.
     * @param f_trigger_pin  TRIG GPIO pin.
     * @param f_echo_pin     ECHO GPIO pin.
     * @param f_serial       Reference to the serial port used for output.
     * @param f_name         Channel name used in the serial message, e.g. "ultrasonic".
     * @param f_threshold_cm Detection threshold in centimetres (default 30 cm).
     */
    CUltrasonicsensor(
        std::chrono::milliseconds   f_period,
        PinName                     f_trigger_pin,
        PinName                     f_echo_pin,
        mbed::UnbufferedSerial&     f_serial,
        const std::string&          f_name,
        uint16_t                    f_threshold_cm = 30
    );

    ~CUltrasonicsensor();

    /**
     * @brief Serial command callback — send "1" to activate, "0" to deactivate.
     *        Registered in the serial monitor subscriber map.
     */
    void serialCallbackULTRASONICcommand(char const* a, char* b);

    std::string m_name;

private:
    DigitalOut              m_trigger;
    DigitalIn               m_echo;
    mbed::UnbufferedSerial& m_serial;

    bool     m_isActive;
    bool     m_lastDetected;   // Tracks previous detection state for edge detection
    uint16_t m_threshold_cm;

    void _run() override;
};

} // namespace periodics
