// 3D scanner firmware for ENGR2110 using two servos and an infrared distance
// sensor. A finite state machine is used to control the state of the scanner and 

#include <Servo.h>
Servo pan_servo;  // create servo object for the pan servo
Servo tilt_servo; // create servo object for the tilt servo

const uint8_t PAN_SERVO = 5;
const uint8_t TILT_SERVO = 6;
const uint8_t INFRARED_SENSOR = 0;

// User parameters of the 3D scanner
const uint8_t scanner_distance = 0.20; // Distance from the sensor to the letter (m)
const uint8_t sensor_height = 0.1; // Distance from the sensor to the ground (m)

// User settings of the 3D scanner
const uint8_t scanning_resolution = 1 // Angular resolution of the scanner (degreesb)

// Define states for the 3D scanner
enum states {
  NONE,
  SCANNING
}

// Define relative states
states piror_state, state;

void calculate_bounding_box() {
  
}

void scanning() {
  // Initialization for scanning state
  if (state != prior_state) {
    prior_state = state;
    calculate_bounding_box();
    // Move to top left to begin scanning
  }
  // Scanning code in grid
}

void setup() {
  // Define inputs and outputs
  pan_servo.attach(5);
  tilt_servo.attach(6);
}

void loop() {
  switch (state) { // This looped section is the core of the state machine.
    case SCANNING:
      scanning();
      break;
  }
}
