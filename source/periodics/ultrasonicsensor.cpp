#include "ultrasonicsensor.hpp"

namespace periodics {

CUltrasonicsensor::CUltrasonicsensor(
    std::chrono::milliseconds   f_period,
    PinName                     f_trigger_pin,
    PinName                     f_echo_pin,
    mbed::UnbufferedSerial&     f_serial,
    const std::string&          f_name,
    uint16_t                    f_threshold_cm)
    : utils::CTask(f_period)
    , m_name(f_name)
    , m_trigger(f_trigger_pin, 0)
    , m_echo(f_echo_pin)
    , m_serial(f_serial)
    , m_isActive(false)
    , m_lastDetected(false)
    , m_threshold_cm(f_threshold_cm)
{
    m_echo.mode(PullDown);  // avoid floating input
}

CUltrasonicsensor::~CUltrasonicsensor()
{
}

void CUltrasonicsensor::serialCallbackULTRASONICcommand(char const* a, char* b)
{
    uint8_t l_isActivate = 0;
    uint8_t l_res = sscanf(a, "%hhu", &l_isActivate);
    if (l_res == 1) {
        m_isActive = (l_isActivate >= 1);
        // Reset detection state so the next run starts fresh
        if (!m_isActive) {
            m_lastDetected = false;
        }
        sprintf(b, "%hhu", l_isActivate);
    } else {
        sprintf(b, "syntax error");
    }
}

void CUltrasonicsensor::_run()
{
    if (!m_isActive) return;

    // trigger
    m_trigger = 0;
    wait_us(2);
    m_trigger = 1;
    wait_us(10);
    m_trigger = 0;

    // wait for echo
    Timer t;
    t.start();
    while (!m_echo.read()) {
        if (t.elapsed_time() > std::chrono::milliseconds(30)) {
            if (m_lastDetected) {
                char buf[64];
                snprintf(buf, sizeof(buf), "@%s:0;;\r\n", m_name.c_str());
                m_serial.write(buf, strlen(buf));
            }
            m_lastDetected = false;
            return;
        }
    }

    //measure echo pulse width
    t.reset();
    while (m_echo.read()) {
        if (t.elapsed_time() > std::chrono::milliseconds(38)) {
            break; 
        }
    }
    t.stop();

    // Distance (cm) = echo pulse duration (µs) / 58
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
                           t.elapsed_time()).count();
    uint16_t distance_cm = static_cast<uint16_t>(duration_us / 58);

    bool detected = (distance_cm > 0 && distance_cm <= m_threshold_cm);

    if (detected) {
        // While an object is in range, keep publishing its current distance.
        char buf[64];
        snprintf(buf, sizeof(buf), "@%s:1;;\r\n", m_name.c_str());
        m_serial.write(buf, strlen(buf));
        m_lastDetected = true;
    } else {
        // When object leaves range, publish one zero on transition.
        if (m_lastDetected) {
            char buf[64];
            snprintf(buf, sizeof(buf), "@%s:0;;\r\n", m_name.c_str());
            m_serial.write(buf, strlen(buf));
        }
        m_lastDetected = false;
    }
}

} // namespace periodics
