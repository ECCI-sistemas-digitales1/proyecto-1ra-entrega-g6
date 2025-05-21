[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-2e0aaae1b6195c2367325f4f02e2d04e9abb55f0b24a779b69b11b9e10269abc.svg)](https://classroom.github.com/online_ide?assignment_repo_id=19162583&assignment_repo_type=AssignmentRepo)
# INTEGRANTES 

Cristian Felipe Archila Barahona - 79630    
Juan Camilo Quevedo Jaimes - 83994  
Johan Antonio Romero Olaya - 95164  
Grupo 6 y 10

## Proyecto - 1ra entrega

🤖 Proyecto: Control de Péndulo Invertido con MPU6050 + PID + L298N 
Este proyecto implementa un sistema de autoestabilización de un péndulo invertido (tipo robot bípedo o seguidor vertical) mediante un algoritmo de control PID y el uso de un sensor MPU6050 para obtener datos de orientación. Se controla un par de motores DC con un puente H L298N.

📦 Dependencias
Asegúrate de tener instaladas estas librerías desde el Administrador de Bibliotecas de Arduino:

-PID_v1

-I2Cdev

-MPU6050 (versión con DMP)

-LMotorController (implementación personalizada para motores, puede ser incluida manualmente)

🔧 Código Fuente

    // === LIBRERÍAS ===
    #include <PID_v1.h>                            // Controlador PID estándar de Arduino
    #include <LMotorController.h>                  // Clase personalizada para controlar dos motores con puente H
    #include "I2Cdev.h"                            // Manejo de dispositivos I2C
    #include "MPU6050_6Axis_MotionApps20.h"        // Manejo del sensor MPU6050 con procesamiento DMP

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"                              // Librería I2C estándar
    #endif

    // === PARÁMETROS DE CONFIGURACIÓN ===
    #define MIN_ABS_SPEED 30                       // Velocidad mínima permitida para que los motores superen la fricción

    MPU6050 mpu;                                   // Objeto principal para acceder al sensor

    // === VELOCIDADES BASE DE LOS MOTORES (ESCALADO) ===
    double MotorVelocidadIzq = 0.55;
    double MotorVelocidadDer = 0.55;

    // === PUNTO DE EQUILIBRIO DESEADO ===
    double PuntoEquilibrio = 184.5;               // En grados. Ajusta según tu calibración

    // === PINES DEL PUENTE H L298N ===
    int ENA = 5;
    int IN1 = 6;
    int IN2 = 7;
    int IN3 = 9;
    int IN4 = 8;
    int ENB = 10;

    // === CONSTANTES DEL CONTROLADOR PID ===
    double Kp = 45;    // Componente proporcional: responde al error
    double Kd = 1.5;   // Componente derivativa: suaviza cambios rápidos
    double Ki = 300;   // Componente integral: corrige errores persistentes

    // === VARIABLES DE ESTADO ===
    int estado = 'g';  // Estado arbitrario (puede usarse para futuras extensiones)

    // === VARIABLES MPU Y DMP ===
    bool dmpReady = false;
    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;         // Tamaño de paquete FIFO DMP
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];      // Buffer de lectura FIFO

    // === VARIABLES DE ORIENTACIÓN ===
    Quaternion q;                // Cuaternión del sensor
    VectorFloat gravity;         // Vector de gravedad derivado
    float ypr[3];                // yaw, pitch, roll (en radianes)

    // === VARIABLES PID ===
    double originalSetpoint = PuntoEquilibrio;  // Ángulo objetivo inicial
    double setpoint = originalSetpoint;         // Referencia actual del PID
    double movingAngleOffset = 0.1;             // Margen de oscilación permitido
    double input, output;                       // Entrada = ángulo actual, Salida = velocidad de corrección

    PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);  // Configuración del PID

    // === INSTANCIA DEL CONTROLADOR DE MOTORES ===
    double motorSpeedFactorLeft = MotorVelocidadIzq;
    double motorSpeedFactorRight = MotorVelocidadDer;
    LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

    // === INTERRUPCIÓN PARA DATOS NUEVOS DEL DMP ===
    volatile bool mpuInterrupt = false;
    void dmpDataReady() {
    mpuInterrupt = true;
    }

    // === FUNCIÓN SETUP ===
    void setup() {
    // Inicializa el bus I2C
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24;  // Configura velocidad I2C a ~400kHz
    #endif

    // Inicializa el MPU6050 y carga el DMP
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // Calibración del sensor (estos valores pueden variar según cada unidad)
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    // Verifica si la inicialización fue exitosa
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);                          // Habilita el DMP
        attachInterrupt(0, dmpDataReady, RISING);         // Usa interrupción externa (pin 2 en Arduino Uno)
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();          // Tamaño de paquete para cada lectura

        // Configura el PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);                            // Ejecutar cada 10 ms
        pid.SetOutputLimits(-255, 255);                   // Salida compatible con PWM
    } else {
        Serial.begin(9600);
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    }

    // === BUCLE PRINCIPAL ===
    void loop() {
    if (!dmpReady) return;  // No hace nada si el DMP no está listo

    // Espera datos nuevos del sensor o suficiente información en el buffer
    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();                                     // Calcula nueva salida del PID
        motorController.move(output, MIN_ABS_SPEED);       // Activa motores con salida PID
    }

    // Reinicia el estado de la interrupción
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // Verifica errores en el buffer FIFO
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();                                   // Limpia buffer si hay desbordamiento
        Serial.println(F("FIFO overflow!"));
    }
    else if (mpuIntStatus & 0x02) {
        // Lee los datos del FIFO
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // Calcula la orientación usando el DMP
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convierte pitch a grados (ypr[1]) y actualiza la entrada del PID
        input = ypr[1] * 180 / M_PI + 180;  // Convierte a grados y lo centra en 180
        }
        }   

🛠️ Hardware Utilizado
-Arduino UNO o Mega

-Sensor MPU6050 (I2C)

-Puente H L298N

-2 Motores DC

-Fuente externa 6–12V (para motores)

-Cables jumper, protoboard

🔌 Esquema de Conexiones
| Componente  | Arduino | Notas             |
| ----------- | ------- | ----------------- |
| MPU6050 SDA | A4      | I2C               |
| MPU6050 SCL | A5      | I2C               |
| MPU6050 VCC | 5V      | Alimentación      |
| MPU6050 GND | GND     | Tierra            |
| ENA         | 5       | PWM para Motor A  |
| IN1         | 6       | Dirección Motor A |
| IN2         | 7       | Dirección Motor A |
| IN3         | 8       | Dirección Motor B |
| IN4         | 9       | Dirección Motor B |
| ENB         | 10      | PWM para Motor B  |


📈 ¿Cómo Funciona?
1 El MPU6050 mide el ángulo de inclinación del péndulo.

2 El algoritmo PID calcula cuánta corrección debe aplicarse.

3 El sistema activa los motores para recuperar el equilibrio.

4 El proceso se repite continuamente cada 10 ms para mantener la verticalidad.

### iMPLEMENTACIÓN

[Pendulo Invertido Presentación](Pendulo.pdf)
