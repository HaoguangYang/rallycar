#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <math.h>
#include <Servo.h>

// ROS Serial stuff
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>  // a replacement for Vextor3 to save memory
#include <std_msgs/ColorRGBA.h>     // a replacement for Quaternion to save memory
#include <std_msgs/Time.h>
namespace ros
{
  // use a lean definition
  typedef NodeHandle_<ArduinoHardware, 2, 4, 150, 150> MyNodeHandle;
}

#define SPEED_LIMITER_PIN 2
#define STEER_PIN 6
#define ACCEL_BRAKE_PIN 7
#define LED_PIN 13

const float targetRate = 100.0;
const float accPedalRange = 2048.0;
const float escMin = 1000.0;
const float escMax = 2000.0;
const float steerRange = 2048.0;
const float steerServoMin = 1000.0;
const float steerServoMax = 2000.0;
// if speed limiter is on the "trim" knob
//const float spdLimiterMin = 1380.0;
const float spdLimiterMin = 1500.0;   // starting from the middle s.t. the car will not move when RC powers off.
const float spdLimiterMax = 1620.0;
// if speed limiter is the actual "throttle" trigger
//const float spdLimiterMin = 1500.0;
//const float spdLimiterMax = 2000.0;

const float steerServoZero = (steerServoMin + steerServoMax) * 0.5;
const float escZero = (escMin + escMax) * 0.5;
//const long targetLoopInterval = floor(1000000./targetRate);

volatile uint16_t spdLimiterPulseHighEdge  = 0;  // The time at beginning of the pulse (on rising edge) in us
volatile uint8_t throttleWatchdogSeconds = 0;
volatile float escLimit = 0;  // The width of pulse in us(microseconds)

ros::MyNodeHandle nh;
geometry_msgs::Point32 accMsg, gyroMsg;
std_msgs::ColorRGBA quatMsg;
std_msgs::Time rosTimeMsg;
ros::Publisher accPub("imu/accelerometer", &accMsg);
ros::Publisher gyroPub("imu/gyroscope", &gyroMsg);
ros::Publisher quatPub("imu/orientation", &quatMsg);
ros::Publisher stampPub("imu/stamp", &rosTimeMsg);

MPU6050 imu;
uint8_t fifoBuffer[64]; // FIFO storage buffer

Servo eSC, steeringServo;

inline float fmap (const float& in, const float& in_min, const float& in_max, const float& out_min, const float& out_max) {
  return (in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void steerCb(const std_msgs::Float32& steerCmd) {
  float cmd = constrain(-steerCmd.data, -steerRange, steerRange);
  cmd = fmap(cmd, -steerRange, steerRange, steerServoMin, steerServoMax);
  steeringServo.writeMicroseconds((int)cmd);
}

void accelBrakeCb(const std_msgs::Float32& accelBrakeCmd) {
  float cmd = constrain(accelBrakeCmd.data, -escLimit, escLimit);
  cmd = fmap(cmd, -accPedalRange, accPedalRange, escMin, escMax);
  eSC.writeMicroseconds((int)cmd);
  throttleWatchdogSeconds = (uint8_t)((micros() >> 20) & 0xFF); // approx 1.048 secs
}

ros::Subscriber<std_msgs::Float32> steerSub("steering_cmd", &steerCb);
ros::Subscriber<std_msgs::Float32> accelBrakeSub("accelerator_cmd", &accelBrakeCb);

void setup(void) {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ROS components
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(accPub);
  nh.advertise(gyroPub);
  nh.advertise(quatPub);
  nh.advertise(stampPub);
  nh.subscribe(steerSub);
  nh.subscribe(accelBrakeSub);

  // Try to initialize IMU
  imu.initialize();
  while (!imu.testConnection()) {
    delay(50);
    imu.initialize();
  }
  int16_t gyroOffset[3], accelOffset[3];
  imu.CoarseCalibrate(&gyroOffset[0], &accelOffset[0]);
  while (imu.dmpInitialize()) {
    delay(50);
  }

  // supply your own gyro offsets here, scaled for min sensitivity
  /*
  imu.setXGyroOffset(220);
  imu.setYGyroOffset(76);
  imu.setZGyroOffset(-85);
  imu.setZAccelOffset(1788); // 1688 factory default for my test chip
  */
  imu.setXGyroOffset(gyroOffset[0]);
  imu.setYGyroOffset(gyroOffset[1]);
  imu.setZGyroOffset(gyroOffset[2]);
  imu.setXAccelOffset(accelOffset[0]);
  imu.setYAccelOffset(accelOffset[1]);
  imu.setZAccelOffset(accelOffset[2]);
  // fine-tune
  imu.CalibrateAccel(6);
  imu.CalibrateGyro(6);
  imu.setDMPEnabled(true);

  // Initialize drive by wire
  steeringServo.attach(STEER_PIN);      // Steer servo connected
  eSC.attach(ACCEL_BRAKE_PIN);          // ESC connected
  // centering the servo and ESC
  steeringServo.writeMicroseconds((int)steerServoZero);
  eSC.writeMicroseconds((int)escZero);

  //Setup the interrput pin to read the speed limiter (trim) signal;
  pinMode(SPEED_LIMITER_PIN, INPUT_PULLUP);
  // External hardware interrupt is attached to the pin and the interrupt is set to trigger on 'CHANGE'
  // in state of pin. On trigger the Interrupt service routing trimPulseTimer is called.
  attachInterrupt(digitalPinToInterrupt(SPEED_LIMITER_PIN), trimPulseTimer, CHANGE);

  digitalWrite(LED_PIN, HIGH);
}

bool GetIMUData(std_msgs::Time* time, geometry_msgs::Point32* acc, geometry_msgs::Point32* gyro, std_msgs::ColorRGBA* quat) {
  bool ret = false;
  const uint16_t mpuDmpPacketSize = imu.dmpGetFIFOPacketSize();
  uint16_t fifoCount = imu.getFIFOCount();// get current FIFO count
  // packets not ready yet
  if (fifoCount < mpuDmpPacketSize) return ret;

  // At least one data packet is available
  time->data = nh.now();

  uint8_t mpuIntStatus = imu.getIntStatus();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    imu.resetFIFO();
    // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // read all available packets from FIFO
    while (fifoCount >= mpuDmpPacketSize) {
      // Lets catch up to NOW, in case someone is using the dreaded delay()!
      imu.getFIFOBytes(fifoBuffer, mpuDmpPacketSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= mpuDmpPacketSize;
    }
    // global_fifo_count = imu.getFIFOCount(); //should be zero here

    // get orientation in quaternion
    int16_t outRaw[4];
    if (!imu.dmpGetQuaternion(outRaw, fifoBuffer)) {
      const float scalingFactor = 1./16384.0f;
      quat->a = (float)outRaw[0] * scalingFactor;
      quat->r = (float)outRaw[1] * scalingFactor;
      quat->g = (float)outRaw[2] * scalingFactor;
      quat->b = (float)outRaw[3] * scalingFactor;
      ret = true;
    }

    if (!imu.dmpGetAccel(outRaw, fifoBuffer)) {
      const float scalingFactor = 9.80665f / 16384.0f;
      acc->x = outRaw[0] * scalingFactor;
      acc->y = outRaw[1] * scalingFactor;
      acc->z = outRaw[2] * scalingFactor;
      ret = true;
    }

    if (!imu.dmpGetGyro(outRaw, fifoBuffer)) {
      const float scalingFactor = M_PI / (16.384 * 180.0);
      gyro->x = outRaw[0] * scalingFactor;
      gyro->y = outRaw[1] * scalingFactor;
      gyro->z = outRaw[2] * scalingFactor;
      ret = true;
    }
  }
  return ret;
}

void loop() {
  //retreive the most current accel/gyro/orientation values from IMU
  if (GetIMUData(&rosTimeMsg, &accMsg, &gyroMsg, &quatMsg)) {
    accPub.publish(&accMsg);
    gyroPub.publish(&gyroMsg);
    quatPub.publish(&quatMsg);
    stampPub.publish(&rosTimeMsg);
  }
  // throttle watchdog times out at 2 seconds
  uint8_t secNow = (uint8_t)((micros() >> 20) & 0xFF);
  if (secNow - throttleWatchdogSeconds > 2) eSC.writeMicroseconds((int)escZero);
  nh.spinOnce();
}

void trimPulseTimer() {
  if (digitalRead(SPEED_LIMITER_PIN) == HIGH) {
    spdLimiterPulseHighEdge = micros() & 0xFFFF;
  } else {
    float spdLimiterPulseWidth = (float)((micros() & 0xFFFF) - spdLimiterPulseHighEdge);
    escLimit = (constrain(spdLimiterPulseWidth, spdLimiterMin, spdLimiterMax)
      - spdLimiterMin) / (spdLimiterMax - spdLimiterMin) * accPedalRange;
  }
}
