/* ---------------------------------------------------------------
                    Libraries and Definitions
 ------------------------------------------------------------------- */

// Library Inclusions
#include "MPU6050.h"              // Accelerometer/Gyroscope
#include <L298NX2.h>              // Motor Driver (Two Motors)
#include <Arduino_FreeRTOS.h>     // FreeRTOS
#include <task.h>
#include <queue.h>

// Define Potentiometer Pin
#define POT A0

// Define L298N Pins
#define EN_A 3
#define IN1_A 5
#define IN2_A 6

#define EN_B 7
#define IN1_B 8
#define IN2_B 9

// Define Global Variables for Gyro Values
int16_t ax, ay, az;
int16_t gx, gy, gz;
float accAngle[2];
float gyrAngle[2];
int minVal=265;
int maxVal=402;
float rad_to_deg = 180/3.141592654;

// Define Global Variables for Motor Values
int speedA;     // Range from 0-255
int speedB;     // Range from 0-255

// Define Global Variables for Controller Values
long prevT, currT;
float deltaT;
float kp, kd, ki;
int desired;
float current, desiredDeg;
float e, e1, ie, u;
int uPWM;

// MPU6050
MPU6050 accelgyro;

// L298N
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

// Define Queue Handlers
QueueHandle_t InputQueue;         // Handles User Input
QueueHandle_t AngleQueue;         // Handles Pitch Angle
QueueHandle_t SpeedQueue;         // Handles Motor Speed

/* ---------------------------------------------------------------
                            Main
 ------------------------------------------------------------------- */

void setup() {
  // Initialize Serial
  Serial.begin(38400);
  while (!Serial) {}

  // Initialize Data Queues
  InputQueue = xQueueCreate(1, sizeof(int));
  AngleQueue = xQueueCreate(1, sizeof(float));
  SpeedQueue = xQueueCreate(1, sizeof(int));

  // Set Variable Values
  deltaT = 0;
  currT = 0;
  prevT = 0;
  current = 0;
  desired = 512;
  desiredDeg = 0;
  e = 0;
  e1 = 0;
  u = 0;
  kp = 3.5;
  ki = 0.005;   
  kd = 2;

  // Initialize MPU6050
  accelgyro.initialize();

  // Initialize L298N
  motors.setSpeedA(127);
  motors.setSpeedB(127);

  // FreeRTOS Tasks Setup
  xTaskCreate(                // User Input Task
    InputTask,                // Function
    "Input Task",             // Task Label
    256,                      // Stack Size
    NULL,                     // Pointer to Structure Data
    2,                        // Priority
    NULL);        // Pointer to Task Handle

  xTaskCreate(                // MPU6050 Task
    GyroTask,                 // Function
    "MPU6050 Task",           // Task Label
    256,                      // Stack Size
    NULL,                     // Pointer to Structure Data
    2,                        // Priority
    NULL);                    // Pointer to Task Handle

  xTaskCreate(                // L298N Task
    MotorTask,                // Function
    "Motor Task",             // Task Label
    256,                      // Stack Size
    NULL,                     // Pointer to Structure Data
    2,                        // Priority
    NULL);                    // Pointer to Task Handle

  xTaskCreate(                // Control Task
    ControlTask,              // Function
    "Control Task",           // Task Label
    1024,                     // Stack Size
    NULL,                     // Pointer to Structure Data
    2,                        // Priority
    NULL);                    // Pointer to Task Handle

  // Start FreeRTOS
  vTaskStartScheduler();

  Serial.println("IF THIS PRINTS SOMETHING WENT WRONG!");
}

void loop() {
  // Empty Loop - FreeRTOS will run tasks
}

/* ---------------------------------------------------------------
                        FreeRTOS Tasks
 ------------------------------------------------------------------- */

void InputTask(void* parameter) {         // Read User Input for Control
  (void) parameter;
  int userInput;

  while(true) {
    // Read input from Potentiometer
    userInput = analogRead(POT);
    
    // Queue User Input and Delay
    xQueueSend(InputQueue, &userInput, portMAX_DELAY);
    vTaskDelay(10);
  }
}

void GyroTask(void *parameter) {          // Sense and Process Pitch Angle
  (void) parameter;
  float pitchAngle;
  double x, y, z;
  int xAng, yAng, zAng;
  
  while (true) {
    // Read Acceleration and Rotation
    accelgyro.getAcceleration(&ax, &ay, &az);
    accelgyro.getRotation(&gx, &gy, &gz);

    xAng = map(ax,minVal,maxVal,-90,90);    // Finds x acceleration
    yAng = map(ay,minVal,maxVal,-90,90);    // Finds y acceleration
    zAng = map(az,minVal,maxVal,-90,90);    // Finds z acceleration

    x = RAD_TO_DEG*(atan2(-yAng,-zAng)+PI);         // Finds y-z angle
    y = RAD_TO_DEG*(atan2(-xAng,-zAng)+PI);         // Finds x-z angle
    z = (RAD_TO_DEG*(atan2(-yAng,-xAng)+PI))-271;   // Finds x-y angle

    // Queue Pitch Angle and Delay (x-y angle)
    xQueueSend(AngleQueue, &z, portMAX_DELAY);
    vTaskDelay(10);
  }
}

void MotorTask(void *parameter) {         // Send Controlled PID Signal to motors
  (void) parameter;
  int receivedSpeed;
  
  while (true) {
    // Recieve Controlled Speed
    xQueueReceive(SpeedQueue, &receivedSpeed, portMAX_DELAY);
    uPWM = receivedSpeed;

    /* PROCESS CONTROLLED SPEED TO FIND TWO MOTOR SPEEDS */
    speedA = 120 + uPWM;
    speedB = 120 - uPWM;

    // Adjust Motor Speeds to Remove System Error
    motors.setSpeedA(speedA);
    motors.setSpeedB(speedB);
    motors.forward();

    // Delay
    vTaskDelay(10);
  }
}

void ControlTask(void *parameter) {       // Process Data and Determine Necessary Controls
  (void) parameter;
  int receivedInput;
  float receivedAngle;
  int motorSpeed;
  
  while (true) {
    // Recieve User Input and Pitch Angle
    xQueueReceive(InputQueue, &receivedInput, portMAX_DELAY);
    xQueueReceive(AngleQueue, &receivedAngle, portMAX_DELAY);

    // Find User's Desired Angle in Degrees
    desired = receivedInput;
    desiredDeg = 36*(((float)desired - 512)/512);

    // Find Current Angle
    current = receivedAngle;

    // Time Difference
    currT = micros();
    deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Find System Error (In Degrees)
    e = desiredDeg - current;
    ie += e*deltaT;

    // Determine Control Signal
    u = kp*e + ki*ie + (kd/deltaT)*(e-e1);

    // If Error is Small, Control is Not Needed
    if(abs(e) < 3) {
      u = 0;
    }

    // Constrain Control For Motor Signal
    u = constrain(u, -120, 120);
    motorSpeed = (int)u;
    e1 = e;
    
    // Open Loop Control *Uncomment for open loop*
    // motorSpeed = 0;

    /* POLLING FOR DEBUG*/
    Serial.print("PitchAngle:");
    Serial.print(current);
    Serial.print(",");
    Serial.print("Desired:");
    Serial.print(desiredDeg);
    Serial.print(",");
    Serial.print("Error:");
    Serial.print(e);
    Serial.print("Control:");
    Serial.print(u);
    Serial.print(",");
    Serial.print("MotorSpeedA:");
    int A = 120 + motorSpeed;
    Serial.print(A);
    Serial.print(",");
    Serial.print("MotorSpeedB:");
    int B = 120 - motorSpeed;
    Serial.println(B);

    // Queue Motor Speed and Delay
    xQueueSend(SpeedQueue, &motorSpeed, portMAX_DELAY);
    vTaskDelay(10);
  }
}