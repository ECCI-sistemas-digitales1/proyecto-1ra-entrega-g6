#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 30

MPU6050 mpu;

// Velocidades de los motores
double MotorVelocidadIzq = 0.55;
double MotorVelocidadDer = 0.55;

// Punto de equilibrio deseado (en grados)
double PuntoEquilibrio = 184.5;

// Pines de control de motores
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 9;
int IN4 = 8;
int ENB = 10;

// Constantes del PID
double Kp = 45;
double Kd = 1.5;
double Ki = 300;

// Estado inicial
int estado = 'g';

// Variables del MPU
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Variables de orientación
Quaternion q;
VectorFloat gravity;
float ypr[3];  // yaw, pitch, roll

// PID
double originalSetpoint = PuntoEquilibrio;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = MotorVelocidadIzq;
double motorSpeedFactorRight = MotorVelocidadDer;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// Interrupción del MPU
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24;  // I2C a 400kHz
  #endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Calibración del giroscopio y acelerómetro
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    // Configuración del PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  } else {
    Serial.begin(9600);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  if (!dmpReady) return;

  // Espera a interrupción del MPU o a tener suficientes datos
  while (!mpuInterrupt && fifoCount < packetSize) {
    pid.Compute();  // Ejecuta el PID
    motorController.move(output, MIN_ABS_SPEED);  // Control de motores
  }

  // Limpia la interrupción y obtiene el estado
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // Cuenta de FIFO
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // Desbordamiento de FIFO
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Obtención del ángulo
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Entrada del PID (conversión a grados)
    input = ypr[1] * 180/M_PI + 180;
  }
}
