/**
 * @file motor.h
 * @author Alexander Dolgii (Anarcom)
 * @brief Contains class for working with motors. 
 * @version 0.5
 * @date 2022-08-03
 * 
 */

#ifndef MOTOR_H_

#include <stdint.h>

/**
 * @brief Contains names of motor ports.
 * 
 */
enum MotorPort{
    A, //!< Output port A
    B, //!< Output port B
    C, //!< Output port C
    D  //!< Output port D
};

enum MotorRegulationMode{
    RegulatedMode, //!< Motor will use PID regulator from firmware
    UnregulatedMode //!< Motor will not use PID regulator from firmware (raw motor)
};

class Motor
{
    private:

    int8_t portMask;
    bool isInversed;
    bool isRegulated;

    public:
    Motor(
        MotorPort port, 
        bool inverse = false, 
        MotorRegulationMode regulationMode = MotorRegulationMode::RegulatedMode
        );
    void on(int8_t speed);
    void off(bool breakAtEnd = true);
    int8_t getPower();
    // void resetEncoder();
    // int getEncoderValue();
    // void onDegRotate(unsigned int angle, int8_t speed, bool breakAtEnd = true);
};



#endif
