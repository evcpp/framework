#include "../include/motor.h"
#include "motor_private.h"

Motor::Motor(
    MotorPort port, 
    bool inverse, 
    MotorRegulationMode regulationMode
){
    this->portMask = portToMaskConverter(port);
    this->isInversed = inverse;
    this->isRegulated = (regulationMode == MotorRegulationMode::RegulatedMode);
    // if motors is not initialized -> it's time to initialize it
    if(!isMotorsInitialized()){
      motorsInit();
    }
}

void Motor::on(int8_t speed){
    if(this->isRegulated){
      setSpeed(this->portMask, speedFilter(speed));
    } else {
      setPower(this->portMask, speedFilter(speed));
    }
    setOutputEx(this->portMask, OUT_ON, RESET_NONE);
}

void Motor::off(bool breakAtEnd){
  setOutputEx(
      this->portMask, 
      (breakAtEnd ? OUT_OFF : OUT_FLOAT),
      RESET_NONE
    );
}

int8_t Motor::getPower(){
  int8_t result = 0;
  outputGetActualSpeed(this->portMask, &result);
  return result;
}

int8_t portToMaskConverter(MotorPort port){
    switch (port)
    {
        case MotorPort::A:
            return PORT_A;
        case MotorPort::B:
            return PORT_B;
        case MotorPort::C:
            return PORT_C;
        case MotorPort::D:
            return PORT_D;
        default:
            throw;
    }
}

int8_t speedFilter(int8_t speed){
    if(speed > 100){
        return 100;
    } else if(speed < -100){
        return -100;
    }
    return speed;
}

//TODO: refactor names
typedef struct {
  int TachoCounts;
  int8_t Speed;
  int TachoSensor;
} MotorData;

typedef struct {
  uint8_t Cmd;
  uint8_t Outputs;
  int8_t PwrOrSpd;
  int StepOrTime1;
  int StepOrTime2;
  int StepOrTime3;
  uint8_t Brake;
} StepOrTimePwrOrSpd;

typedef struct {
  uint8_t Cmd;
  uint8_t Outputs;
  int8_t Speed;
  short Turn;
  int StepOrTime;
  uint8_t Brake;
} StepOrTimeSync;

typedef struct {
  int8_t outputTypes[NUM_MOTOR_PORTS];
  short owners[NUM_MOTOR_PORTS];

  int pwmFile;
  int motorFile;

  MotorData motorData[NUM_MOTOR_PORTS];
  MotorData *pMotor;
} OutputGlobals;

OutputGlobals outputInstance;

//TODO: add atexit function that stops all motors
bool motorsInit(){
    if (isMotorsInitialized())
        return true;

  MotorData *pTmp;

  // To ensure that pMotor is never uninitialised
  outputInstance.pMotor = outputInstance.motorData;

  // Open the handle for writing commands
  outputInstance.pwmFile = open(LMS_PWM_DEVICE_NAME, O_RDWR);

  if (outputInstance.pwmFile >= 0) {
    // Open the handle for reading motor values - shared memory
    outputInstance.motorFile = open(LMS_MOTOR_DEVICE_NAME, O_RDWR | O_SYNC);
    if (outputInstance.motorFile >= 0) {
      pTmp = (MotorData *)mmap(0, sizeof(outputInstance.motorData),
                               PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED,
                               outputInstance.motorFile, 0);
      if (pTmp == MAP_FAILED) {
        //        LogErrorNumber(OUTPUT_SHARED_MEMORY);
        close(outputInstance.motorFile);
        close(outputInstance.pwmFile);
        outputInstance.motorFile = -1;
        outputInstance.pwmFile = -1;
      } else {
        outputInstance.pMotor = pTmp;
        bool outputOpenTest = outputOpen();

        /**
         * Set the polarity here to fix the issue:
         * https://github.com/simonedegiacomi/EV3-API/issues/13
         *
         * Then simply use a negative speed/power to go in reverse
         */
        // TODO: If the fix works, change function name
        setDirection(PORT_ALL, OUT_FWD);
        setOutputEx(PORT_ALL, OUT_OFF, RESET_ALL);

        return outputOpenTest;
      }
    }
  }
  return false;
}

bool isMotorsInitialized(){
    return (outputInstance.pwmFile != -1) && (outputInstance.pMotor != NULL);
}

bool outputOpen() {
  if (!isMotorsInitialized())
    return false;

  int8_t cmd;

  bool result = resetOutputs();
  if (result) {
    cmd = opProgramStart;
    return writeToPWMDevice(&cmd, 1) == 1;
  }

  return result;
}

bool resetOutputs() {
  // stop all the motors
  int i;
  for (i = 0; i < NUM_MOTOR_PORTS; i++) {
    outputInstance.owners[i] = OWNER_NONE;
  }

  return outputStop(PORT_ALL, false);
}

bool outputStop(uint8_t Outputs, bool useBrake) {
  if (!isMotorsInitialized())
    return false;

  int cmdLen = 3;
  int8_t cmd[3];
  uint8_t Layer;
  
  decodeOutputs(&Outputs, &Layer);
  
    cmd[0] = opOutputStop;
    cmd[1] = Outputs;
    cmd[2] = useBrake;
    return writeToPWMDevice(cmd, cmdLen) == cmdLen;
}

int writeToPWMDevice(int8_t *bytes, int num_bytes) {
  int result = -1;
  if (outputInstance.pwmFile >= 0) {
    // for some reason write is not returning num_bytes -
    // it usually returns zero
    result = write(outputInstance.pwmFile, bytes, num_bytes);
    if (result >= 0) {
      return num_bytes;
    }
  }
  return result;
}

void decodeOutputs(uint8_t *outputs, uint8_t *layer) {
  *layer = *outputs & LAYER_MASK;
  *outputs = *outputs & OUT_MASK;
}

//TODO: refactor this
void setDirection(uint8_t Outputs, uint8_t Dir) {
  int8_t Polarity;
  switch (Dir) {
  case OUT_REV:
    Polarity = -1;
    break;
  case OUT_TOGGLE:
    Polarity = 0;
    break;
  default:
    Polarity = 1;
    break;
  }

  outputPolarity(Outputs, Polarity);
}

// TODO: refactor this define
#define outputStart(_outputs) outputStartEx((_outputs), OWNER_NONE)

void setOutputEx(uint8_t Outputs, uint8_t Mode, uint8_t reset) {
  switch (Mode) {
  case OUT_FLOAT:
    outputStop(Outputs, false);
    resetCount(Outputs, reset);
    break;
  case OUT_OFF:
    outputStop(Outputs, true);
    resetCount(Outputs, reset);
    break;
  case OUT_ON:
    resetCount(Outputs, reset);
    outputStart(Outputs);
    break;
  }
}

bool outputPolarity(uint8_t outputs, int8_t polarity) {
  if (!isMotorsInitialized())
    return false;

  int cmdLen = 3;
  uint8_t layer;
  int8_t cmd[3];
  // opOutputPolarity (outputs, polarity)
  // Set polarity of the outputs
  //  - -1 makes the motor run backward
  //  -  1 makes the motor run forward
  //  -  0 makes the motor run the opposite direction
  decodeOutputs(&outputs, &layer);
  if (layer == LAYER_MASTER) {
    cmd[0] = opOutputPolarity;
    cmd[1] = outputs;
    cmd[2] = polarity;
    return writeToPWMDevice(cmd, cmdLen) == cmdLen;
  } else {
    return false;
  }
}

void resetCount(uint8_t outputs, uint8_t reset) {
  // reset tacho counter(s)
  switch (reset) {
  case RESET_COUNT:
    resetTachoCount(outputs);
    return;
  case RESET_BLOCK_COUNT:
    resetBlockTachoCount(outputs);
    return;
  case RESET_ROTATION_COUNT:
    resetRotationCount(outputs);
    return;
  case RESET_BLOCKANDTACHO:
    resetBlockAndTachoCount(outputs);
    return;
  case RESET_ALL:
    resetAllTachoCounts(outputs);
    return;
  }
}

bool outputStartEx(uint8_t Outputs, uint8_t Owner) {
  if (!isMotorsInitialized())
    return false;

  int cmdLen = 2;
  uint8_t Layer;
  int8_t cmd[2];
  // Starts the outputs
  decodeOutputs(&Outputs, &Layer);
  if (Layer == LAYER_MASTER) {
    cmd[0] = opOutputStart;
    cmd[1] = Outputs;
    bool result = writeToPWMDevice(cmd, cmdLen) == cmdLen;
    if (result) {
      int i;
      for (i = 0; i < NUM_MOTOR_PORTS; i++) {
        if (Outputs & (0x01 << i))
          outputInstance.owners[i] = Owner;
      }
    }
    return result;
  } else {
    return false;
  }
}

void resetTachoCount(uint8_t Outputs) {
  // reset tacho counter(s)
  outputReset(Outputs);
}

void resetBlockTachoCount(uint8_t Outputs) {
  // synonym for ResetTachoCount
  resetTachoCount(Outputs);
}

void resetRotationCount(uint8_t Outputs) {
  // reset tacho counter(s)
  outputClearCount(Outputs);
}

void resetAllTachoCounts(uint8_t Outputs) {
  // clear all tacho counts
  outputReset(Outputs);
  outputClearCount(Outputs);
}

//TODO: refactor
void resetBlockAndTachoCount(uint8_t Outputs) {
  // synonym for ResetTachoCount
  resetTachoCount(Outputs);
}

bool outputReset(uint8_t Outputs) {
  if (!isMotorsInitialized())
    return false;

  int cmdLen = 2;
  uint8_t Layer;
  int8_t cmd[2];
  // opOutputReset (outputs)
  // Resets the Tacho counts
  decodeOutputs(&Outputs, &Layer);
  if (Layer == LAYER_MASTER) {
    cmd[0] = opOutputReset;
    cmd[1] = Outputs;
    return writeToPWMDevice(cmd, cmdLen) == cmdLen;
  } else {
    return false;
  }
}

bool outputClearCount(uint8_t Outputs) {
  if (!isMotorsInitialized())
    return false;

  int cmdLen = 2;
  uint8_t Layer;
  int8_t cmd[2];
  decodeOutputs(&Outputs, &Layer);
  if (Layer == LAYER_MASTER) {
    cmd[0] = opOutputClearCount;
    cmd[1] = Outputs;
    bool result = writeToPWMDevice(cmd, cmdLen) == cmdLen;
    if (result) {
      int i;
      for (i = 0; i < NUM_MOTOR_PORTS; i++) {
        if (Outputs & (0x01 << i))
          outputInstance.pMotor[i].TachoSensor = 0;
      }
    }
    return result;
  } else {
    return false;
  }
}

//TODO: refactor next 2 functions
void setPower(uint8_t Outputs, int8_t Power) { 
  outputPower(Outputs, Power); 
}

void setSpeed(uint8_t Outputs, int8_t Speed) { 
  outputSpeed(Outputs, Speed); 
}

bool outputPower(uint8_t Outputs, int8_t Power) {
  if (!isMotorsInitialized())
    return false;
  int cmdLen = 3;
  uint8_t Layer;
  int8_t cmd[3];
  // opOutputPower (outputs, power)
  // Set power of the outputs
  decodeOutputs(&Outputs, &Layer);
  if (Layer == LAYER_MASTER) {
    cmd[0] = opOutputPower;
    cmd[1] = Outputs;
    cmd[2] = Power;
    return writeToPWMDevice(cmd, cmdLen) == cmdLen;
  } else {
    return false;
  }
}

bool outputSpeed(uint8_t Outputs, int8_t Speed) {
  if (!isMotorsInitialized())
    return false;

  int cmdLen = 3;
  uint8_t Layer;
  int8_t cmd[3];
  // opOutputSpeed (outputs, speed)
  // Set speed of the outputs
  // (relative to polarity - enables regulation if the output has a tachometer)
  decodeOutputs(&Outputs, &Layer);
  if (Layer == LAYER_MASTER) {
    cmd[0] = opOutputSpeed;
    cmd[1] = Outputs;
    cmd[2] = Speed;
    return writeToPWMDevice(cmd, cmdLen) == cmdLen;
  } else {
    return false;
  }
}

//TODO: refactor (return value by return)
bool outputGetActualSpeed(uint8_t output, int8_t *speed) {
  int tcount = 0;
  int tsensor = 0;
  return outputRead(output, speed, &tcount, &tsensor);
}

//TODO: refactor
bool outputRead(
  uint8_t output, 
  int8_t *speed, 
  int *tachoCount,
  int *tachoSensor
  ) {
  if (!isMotorsInitialized())
    return false;

  uint8_t layer;
  *speed = 0;
  *tachoCount = 0;
  *tachoSensor = 0;
  decodeOutputs(&output, &layer);
  if (layer == LAYER_MASTER) {
    output = outputToMotorNum(output);
    if (output < NUM_MOTOR_PORTS) {
      *speed = outputInstance.pMotor[output].Speed;
      *tachoCount = outputInstance.pMotor[output].TachoCounts;
      *tachoSensor = outputInstance.pMotor[output].TachoSensor;
      return true;
    }
  }
  return false;
  // the firmware code does not contain any sign of letting you read
  // tacho values from other layers via this opcode.
}

uint8_t outputToMotorNum(uint8_t output) {
  switch (output) {
  case PORT_A:
    return 0;
  case PORT_B:
    return 1;
  case PORT_C:
    return 2;
  case PORT_D:
    return 3;
  }
  return NUM_MOTOR_PORTS;
}
