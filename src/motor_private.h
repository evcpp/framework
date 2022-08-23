#ifndef MOTOR_PRIVATE_H_

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define PORT_A    0x01 //!< Motor port A
#define PORT_B    0x02 //!< Motor port B
#define PORT_C    0x04 //!< Motor port C
#define PORT_D    0x08 //!< Motor port D
#define PORT_ALL  0x0f //!< All output ports (A-D) 

#define NUM_MOTOR_PORTS 4
#define LMS_PWM_DEVICE            "lms_pwm"         //!< PWM device name
#define LMS_PWM_DEVICE_NAME       "/dev/lms_pwm"    //!< PWM device file name
#define LMS_MOTOR_DEVICE          "lms_motor"       //!< TACHO device name
#define LMS_MOTOR_DEVICE_NAME     "/dev/lms_motor"  //!< TACHO device file name
#define OUT_MASK                  0x0f              //!< The output port mask
#define LAYER_MASK                0x70              //!< The layer mask

#define opOutputPolarity            0xA7 /*!< opOutputPolarity */
#define opOutputReset               0xA2 /*!< opOutputReset */
#define opOutputStop                0xA3 /*!< opOutputStop */
#define opOutputPower               0xA4 /*!< opOutputPower */
#define opOutputSpeed               0xA5 /*!< opOutputSpeed */
#define opOutputStart               0xA6 /*!< opOutputStart */
#define opProgramStart              0x03 /*!< opProgramStart */
#define opOutputClearCount          0xB2 /*!< opOutputClearCount */

#define OWNER_NONE                0x0000
#define LAYER_MASTER              0x00

#define OUT_REV    0x00
#define OUT_TOGGLE 0x40
#define OUT_FWD    0x80

#define OUT_FLOAT 0x00
#define OUT_OFF   0x40
#define OUT_ON    0x80

#define RESET_NONE           0x00 //!< No counters will be reset 
#define RESET_COUNT          0x08 //!< Reset the internal tachometer counter 
#define RESET_BLOCK_COUNT    0x20 //!< Reset the block tachometer counter 
#define RESET_ROTATION_COUNT 0x40 //!< Reset the rotation counter 
#define RESET_BLOCKANDTACHO  0x28 //!< Reset both the internal counter and the block counter 
#define RESET_ALL            0x68 //!< Reset all tachometer counters 


int8_t portToMaskConverter(MotorPort);
int8_t speedFilter(int8_t);

bool motorsInit();
bool isMotorsInitialized();
bool outputOpen();
bool resetOutputs();
bool outputStop(uint8_t outputs, bool useBrake);
int writeToPWMDevice(int8_t *bytes, int num_bytes);
void decodeOutputs(uint8_t *outputs, uint8_t *layer);
void setDirection(uint8_t Outputs, uint8_t Dir);
void setOutputEx(uint8_t Outputs, uint8_t Mode, uint8_t reset);
bool outputPolarity(uint8_t outputs, int8_t polarity);
void resetCount(uint8_t outputs, uint8_t reset);
void resetTachoCount(uint8_t Outputs);
void resetBlockTachoCount(uint8_t Outputs);
void resetRotationCount(uint8_t Outputs);
void resetAllTachoCounts(uint8_t Outputs);
void resetBlockAndTachoCount(uint8_t Outputs);
bool outputReset(uint8_t outputs);
bool outputClearCount(uint8_t Outputs);
bool outputStartEx(uint8_t Outputs, uint8_t Owner);
void setPower(uint8_t Outputs, int8_t Power);
void setSpeed(uint8_t Outputs, int8_t Speed);
bool outputPower(uint8_t Outputs, int8_t Power);
bool outputSpeed(uint8_t Outputs, int8_t Speed);
bool outputGetActualSpeed(uint8_t output, int8_t *speed);
bool outputRead(uint8_t output, int8_t *speed, int *tachoCount, int *tachoSensor);
uint8_t outputToMotorNum(uint8_t output);

#endif
