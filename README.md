# Obstacle avoidance Robot car

An obstacle-avoiding car is a robotic project designed to detect and avoid obstacles in its path using sensors and a microcontroller. Typically, an ultrasonic sensor (such as HC-SR04) measures the distance between the car and nearby objects. When an obstacle is detected within a certain range, the sensor sends this information to the microcontroller (like Arduino), which processes the data. The microcontroller then instructs a motor driver (like L298N) to stop or change direction by controlling the car’s DC motors.

The car moves forward, and upon detecting an obstacle, it stops, checks for the clearest direction, and turns to avoid the obstacle before continuing. This basic project introduces key concepts in robotics and embedded systems, including sensor interfacing, motor control, and real-time decision-making, making it a great starting point for learning about autonomous vehicles and robotic navigation.



![alt text](https://res.cloudinary.com/dhrd4kfja/image/upload/v1729691803/car2_hy3lcb.webp)



## Components Required

| Items            | Quantity                                                               |
| ----------------- | ------------------------------------------------------------------ |
| Arduino Uno board |x1 |
| Motor driver shield | x1 |
| Ultrasonic sensor | x1 |
| Servo motor | x1 |
| Gear motor | x4 |
| Robot wheels | x4 |
| Li-ion battery | x1 |
| Battery holder | x1 |
| Dot board | x1 |
| Jumper wires |  |


### Arduino Uno

![alt text](https://res.cloudinary.com/dhrd4kfja/image/upload/v1729693224/arduino_uno_ai7quf.jpg)

Microcontroller (Arduino): Acts as the brain of the car. It processes input from sensors and controls the motor driver based on programmed logic. The Arduino interprets sensor data and commands the motors to change direction when obstacles are detected.

### Ultrasonic Sensor (HC-SR04)

![alt text](https://res.cloudinary.com/dhrd4kfja/image/upload/v1729693205/ultrasonic_omver3.jpg)

Emits ultrasonic waves and measures the time it takes for the echo to return after hitting an obstacle. Using this time, it calculates the distance to the obstacle. The sensor has four pins: VCC (power), GND (ground), TRIG (to trigger the wave), and ECHO (to receive the echo).

### Motor Driver (L298N/L293D)

![alt text](https://res.cloudinary.com/dhrd4kfja/image/upload/v1729693592/motor_driver_pics4f.jpg)

Converts low-power control signals from the microcontroller into higher-power signals to drive the DC motors. It allows the microcontroller to control motor direction and speed by managing the flow of current.

### Servo Motor

![alt text](https://res.cloudinary.com/dhrd4kfja/image/upload/v1729693182/servo_motor_ehijnm.jpg)

A servo motor is a precise rotary actuator commonly used in robotics and automation for controlling angular position. It consists of a DC motor, gearbox, control circuit, and a position sensor (usually a potentiometer). The motor’s rotation is controlled using Pulse Width Modulation (PWM) signals, where the width of the pulse determines the angle of rotation (For example, a 1 ms pulse might turn the servo to 0°, 1.5 ms to 90°, and 2 ms to 180°).


## Circuit Diagram

![alt text](https://res.cloudinary.com/dhrd4kfja/image/upload/v1729694303/circuit_xefzx3.webp)


We have to connect the wires as shown in the above circuit diagram to ensure the proper functioning of all components, including the microcontroller, ultrasonic sensor, servo motor, and motor driver, with each component correctly wired to the appropriate power, ground, and signal pins.


## Arduino code

Upload this code into arduino. 

```bash
#include <AFMotor.h>
#include <Servo.h>
#define Speed 180
#define Trig A0
#define Echo A1
#define spoint 90
int distance;
int Left;
int Right;
int L = 0;
int R = 0;
Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

void setup() {
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  servo.attach(10);
  start();
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);

}

void loop() {
  distance = ultrasonic();
  if (distance <= 5) {
    Stop();
    backward();
    delay(100);
    Stop();
    L = leftsee();
    servo.write(spoint);
    delay(800);
    R = rightsee();
    servo.write(spoint);
    if (L < R) {
      turnleft();
      delay(500);
      Stop();
      delay(200);
    } else if (L > R) {
      turnright();
      delay(500);
      Stop();
      delay(200);
    }
  } else {
    forward();
  }
}

void forward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}
void backward() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}
void turnleft() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}
void turnright() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}
void Stop() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}
int leftsee() {
  servo.write(20);
  delay(800);
  Left = ultrasonic();
  return Left;
}

int rightsee() {
  servo.write(150);
  delay(800);
  Right = ultrasonic();
  return Right;
}

int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  long cm = t / 29 / 2; //time convert distance
  return cm;
}
void start() {
  delay(3000);
  for (int a = 0; a < 4; a++) {
    servo.write(spoint);
    delay(50);
    servo.write(40);
    delay(50);
    servo.write(90);
    delay(50);
    servo.write(spoint);
  }
}

```


### Finally !!

![alt text](https://res.cloudinary.com/dhrd4kfja/image/upload/v1729693258/car3_du065z.jpg)

