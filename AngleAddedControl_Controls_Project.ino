/* ---------------------------------------------------------------
                    Libraries and Definitions
 ------------------------------------------------------------------- */

// Library Inclusions
#include "MPU6050.h"              // Accelerometer/Gyroscope
#include <L298NX2.h>              // Motor Driver (Two Motors)
#include <Arduino_FreeRTOS.h>     // FreeRTOS
#include <task.h>
#include <queue.h>

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

// Define Global Variables for Motor Values
int speedA;     // Range from 0-255
int speedB;     // Range from 0-255

// Define Global Variables for Controller Values
float rad_to_deg = 180/3.141592654;
long prevT, currT;
float deltaT;
long prevT2, currT2;
float deltaT2;
float kp, kd, ki;
float current, desired, desiredDeg;
float e, e1, e2;
float u, u1;
int uPWM;

// MPU6050
MPU6050 accelgyro;

// L298N
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

// Define Queue Handlers
QueueHandle_t dataQueue1;       // Handles User Input
QueueHandle_t dataQueue2;       // Handles Pitch Angle
QueueHandle_t dataQueue3;       // Handles Pitch Angle 2
QueueHandle_t dataQueue4;       // Handles Motor Speed

/* ---------------------------------------------------------------
                            Main
 ------------------------------------------------------------------- */

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  while (!Serial) {}

  // Initialize Data Queues
  dataQueue1 = xQueueCreate(1, sizeof(int));
  dataQueue2 = xQueueCreate(1, sizeof(int));
  dataQueue3 = xQueueCreate(1, sizeof(int));
  dataQueue4 = xQueueCreate(1, sizeof(int));

  // Set Variable Values
  deltaT = 0;
  currT = 0;
  prevT = 0;
  current = 0;
  desired = 0;
  desiredDeg = 0;
  e = 0;
  e1 = 0;
  e2 = 0;
  u = 0;
  u1 = 0;
  kp = 3.55;
  ki = 0.005;
  kd = 2.05;

  // Initialize MPU6050
  accelgyro.initialize();

  // Initialize L298N
  motors.setSpeed(127);

  // FreeRTOS Tasks Setup
  xTaskCreate(                // User Input Task
    InputTask,                // Function
    "Input Task",             // Task Label
    128,                      // Stack Size
    NULL,                     // Pointer to Structure Data
    2,                        // Priority
    NULL);                    // Pointer to Task Handle

  xTaskCreate(                // MPU6050 Task
    GyroTask,                 // Function
    "MPU6050 Task",           // Task Label
    128,                      // Stack Size
    NULL,                     // Pointer to Structure Data
    2,                        // Priority
    NULL);                    // Pointer to Task Handle

  xTaskCreate(                // L298N Task
    MotorTask,                // Function
    "Motor Task",             // Task Label
    128,                      // Stack Size
    NULL,                     // Pointer to Structure Data
    2,                        // Priority
    NULL);                    // Pointer to Task Handle

  xTaskCreate(                // Control Task
    ControlTask,              // Function
    "Control Task",           // Task Label
    128,                      // Stack Size
    NULL,                     // Pointer to Structure Data
    2,                        // Priority
    NULL);                    // Pointer to Task Handle

  // Start FreeRTOS
  vTaskStartScheduler();
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
    //Wait for Input
    if(Serial.available() > 0) {
      Serial.print("Please input integer (0-1023) for desired angle (-36 to 36 degrees): ");
      userInput = Serial.parseInt(); //Read Int from Input
      if(userInput >= 0 && userInput <= 1023) {
        // Process Valid Range
        Serial.print("Received input: ");
        Serial.println(userInput);
      }
      else {
        // Handle Invalid Range
        Serial.println("Invalid input. Enter a number between 0 and 1023.");
      }
    }

    // Queue User Input and Delay
    userInput = (int)userInput;
    xQueueSend(dataQueue1, &userInput, portMAX_DELAY);
    delay(10);
  }
}

void GyroTask(void *parameter) {          // Sense and Process Pitch Angle
  (void) parameter;
  int pitchAngle;
  int pitchAngle2;
  
  while (true) {
    // Read Acceleration and Rotation
    int minVal=265;
    int maxVal=402;
    double x, y, z;
    accelgyro.getAcceleration(&ax, &ay, &az);
    accelgyro.getRotation(&gx, &gy, &gz);


    int xAng = map(ax,minVal,maxVal,-90,90);
    int yAng = map(ay,minVal,maxVal,-90,90);
    int zAng = map(az,minVal,maxVal,-90,90);
    x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    z= (RAD_TO_DEG * (atan2(-yAng, -xAng)+PI))-270;
    
    
    Serial.print("AngleX:");
    Serial.print(x);
    Serial.print(",");
    Serial.print("AngleY:");
    Serial.print(y);
    Serial.print(",");
    Serial.print("AngleZ:");
    Serial.println(z);

    // Queue Pitch Angle and Delay
    xQueueSend(dataQueue2, &z, portMAX_DELAY);
    xQueueSend(dataQueue3, &z, portMAX_DELAY);
    delay(10);
  }
}

void MotorTask(void *parameter) {         // Send Controlled PID Signal to motors
  (void) parameter;
  int receivedSpeed;
  
  while (true) {
    // Recieve Controlled Speed
    xQueueReceive(dataQueue4, &receivedSpeed, portMAX_DELAY);
    uPWM = receivedSpeed;

    /* PROCESS CONTROLLED SPEED TO FIND TWO MOTOR SPEEDS */
    speedA = 63 + uPWM;
    speedB = 63 - uPWM;


    // Adjust Motor Speeds to Remove System Error
    motors.setSpeedA(speedA);
    motors.setSpeedB(speedB);
    motors.forward();

    // Delay
    delay(10);
  }
}

void ControlTask(void *parameter) {       // Process Data and Determine Necessary Controls
  (void) parameter;
  int receivedInput;
  int receivedAngle;
  int receivedAngle2;
  int motorSpeed;
  
  while (true) {
    // Recieve User Input and Pitch Angle
    xQueueReceive(dataQueue1, &receivedInput, portMAX_DELAY);
    xQueueReceive(dataQueue2, &receivedAngle, portMAX_DELAY);
    xQueueReceive(dataQueue3, &receivedAngle2, portMAX_DELAY);

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

    // Determine Control Signal
    u = u1 + kp*(e-e1) + ki*deltaT*e1 + (kd/deltaT)*(e-2*e1+e2);

    // If Error is Small, Control is Not Needed
    if(abs(e) < 0.5) {
      u = 0;
    }

    // Constrain Control For Motor Signal
    u = constrain(u, -100, 100);
    motorSpeed = (int)(abs(u) * 0.62);

    // Store Previous Error and Control
    e2 = e1;
    e1 = e;
    u1 = u;

    // Queue Motor Speed and Delay
    xQueueSend(dataQueue4, &motorSpeed, portMAX_DELAY);
    delay(10);
  }
}