#include "DualMC33926MotorShield.h"
#include <SoftwareSerial.h>

// Real world note, 15s to open LEFT
// Left to open first,
// Right to close first

// Maximum value passed to setMXSpeed function
// setM1Speed/setM2Speed doesn't receive a percentage,
// instead it receive a value from 0 (stopped) to 400 (full speed)
int MAX_SPEED = 400;

// Maximum current before we stop the motors.
// When there is an obstacle, the motor consume more current,
// this is anormal and we should stop the motor.
int MAX_CURRENT = 500; // in milliamps (mA)

// Object for the motor controller
DualMC33926MotorShield md;

// Serial link to the bluetooth shield
SoftwareSerial bt(5, 6); // RX, TX

enum side_t {LEFT, RIGHT};
enum direction_t {OPEN, CLOSE};
enum state_t {OPENED, CLOSED};

bool isMotorBlocked(enum side_t side) {
  int current;
  if (side == LEFT) {
    current = md.getM1CurrentMilliamps();
  } else {
    current = md.getM2CurrentMilliamps();
  }
  //Serial.print("Current:");
  //Serial.println(current);
  if (current > MAX_CURRENT) {
    Serial.print("Reached max current, motor stopped!");
    return true;
  }
  return false;
}

// Delay between each iteration
int ITERATION_DELAY = 500; // in ms

// Time spent to start opening only left door when opening
// or right door when closing (so that they open/close in the right order
// without overlaping incorrectly)
int FIRST_DOOR_ONLY_DELAY = 3 * 1000; // in ms

// Time spent going from stopped, to full speed, in milliseconds.
// When we start opening, closing the door, we gradualy increase the speed.
int EASE_DELAY = 3 * 1000; // in ms

// Maximum time to open a door, in milliseconds
int TIMEOUT = 20 * 1000; // in ms


void move(enum direction_t direction) {
  Serial.print(direction == OPEN ? "Open " : "Close ");
  Serial.println(" - start easing");

  md.setM1Speed(0);
  md.setM2Speed(0);

  bool left_blocked = false;
  bool right_blocked = false;
  
  int number_of_iterations = TIMEOUT / ITERATION_DELAY;
  for (int t = 0; t < TIMEOUT; t += ITERATION_DELAY) {
    int left_speed = 0;
    int right_speed = 0;
    if (t < FIRST_DOOR_ONLY_DELAY) {
      long speed = MAX_SPEED * ((double)t / FIRST_DOOR_ONLY_DELAY);
      if (direction == OPEN) {
        left_speed = speed;
      } else {
        right_speed = speed;
      }
    } else if (t < FIRST_DOOR_ONLY_DELAY + EASE_DELAY) {
      long speed = MAX_SPEED * ((double)(t - FIRST_DOOR_ONLY_DELAY) / EASE_DELAY);
      if (direction == OPEN) {
        left_speed = MAX_SPEED;
        right_speed = speed;
      } else {
        right_speed = MAX_SPEED;
        left_speed = speed;
      }
    } else if (t < TIMEOUT - EASE_DELAY) {
      left_speed = MAX_SPEED;
      right_speed = MAX_SPEED;
    } else {
      left_speed = MAX_SPEED / 2;
      right_speed = MAX_SPEED / 2;
    }

    // Check if one door is blocked, if that's the case
    // always set its speed to zero.
    left_blocked = left_blocked || isMotorBlocked(LEFT);
    right_blocked = right_blocked || isMotorBlocked(RIGHT);
    if (left_blocked) {
      left_speed = 0;
    }
    if (right_blocked) {
      right_speed = 0;
    }

    if (md.getFault()) {
      Serial.print("Something wrong with the controller, stop everything");
      break;
    }
    
    Serial.print("Speed at ");
    Serial.print(t);
    Serial.print(" left:");
    Serial.print(left_speed);
    Serial.print(" right:");
    Serial.println(right_speed);
    
    int dir = direction == OPEN ? 1 : -1;
    md.setM1Speed(dir * left_speed);
    md.setM2Speed(dir * right_speed);

    delay(ITERATION_DELAY);
  }

  md.setM1Speed(0);
  md.setM2Speed(0);
}

// Read buffer, where raw data from bluetooth are stored
#define INPUT_SIZE 20
char cmd[INPUT_SIZE + 1];

// Function run once when the device starts
void setup()
{
  // Setup the console output
  Serial.begin(115200);
  Serial.println("Portail");

  // Setup the link to the bluetooth shield
  bt.begin(9600);

  // Wait for it to be ready
  do {
    bt.print("AT");
    byte size = bt.readBytes(cmd, INPUT_SIZE);
    cmd[size] = 0;
    Serial.println(cmd);
  } while(strcmp(cmd, "OK") != 0);
  
  // Change bluetooth device name
  bt.print("AT+NAMEHop");
  while (!bt.available()) {}
  bt.read();
  
  // Change its password
  bt.print("AT+PASS319705");
  while (!bt.available()) {}
  bt.read();
  
  // Reboot the bluetooth shield to update the name
  bt.print("AT+RESET");
  while (!bt.available()) {}
  bt.read();
  delay(500);
  Serial.println("Bluetooth ready!");
  
  // Initialize the motor object "md"
  md.init();
  
  open();
}

// Main entry point, function run indefinitely
void loop()
{
  if (bt.available()) {
    byte size = bt.readBytes(cmd, INPUT_SIZE);
    cmd[size] = 0;
    Serial.print("Received command: ");
    Serial.print(size);
    Serial.print(" - ");
    Serial.print(cmd);
    if (strcmp(cmd, "a") == 0) {
      open();
    } else if (strcmp(cmd, "b") == 0) {
      close();
    }
  }
  /*
   *  DEBUG: allow to send AT commands to bluetooth shield
  if (Serial.available()) {
    bt.write(Serial.read());
  }
  */
}

void open() {
  move(OPEN);
}

void close() {
  move(CLOSE);
}
