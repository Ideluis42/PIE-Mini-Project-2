// 3D scanner firmware for ENGR2110 using two servos and an infrared distance
// sensor. A finite state machine is used to control the state of the scanner and 

#include <Servo.h>
Servo pan_servo;  // create servo object for the pan servo
Servo tilt_servo; // create servo object for the tilt servo

const uint8_t TILT_SERVO = 9;
const uint8_t PAN_SERVO = 10;
const uint8_t INFRARED_SENSOR = 0;

// User parameters of the 3D scanner and letter
const uint8_t scanner_distance = 0.20; // Distance from the sensor to the letter (m)
const uint8_t sensor_height = 0.1; // Distance from the sensor to the ground (m)
const uint8_t letter_height = 0.3556; // Height of the letter to be scanned (m)
const uint8_t letter_width = 0.2286; // Width of the letter to be scanned (m)
const uint8_t letter_tolerance = 0.05; // Tolerance to measure to the sides of the letter (m)
const uint8_t scanning_interval = 10; // Interval to wait between servo moves and scans (milliseconds)
const uint16_t DEBOUNCE_INTERVAL = 10; // Minimum interval to wait before checking button presses 

// User settings of the 3D scanner
const uint8_t scanning_resolution = 1 // Angular resolution of the scanner (degrees)

// Define states for the 3D scanner
enum states {
  NONE,
  READY,
  SCANNING
}

// Define relative states
states piror_state, state;

void calculate_bounding_box() {
  // Based on the reading of the IMU and the height and distance of the sensor
  // from the letter, calculate the angular bounding box to scan.
  // Get absolute pitch, roll, and yaw from the IMU
  float scanning_height = letter_height
}

void ready() {
  // Initialization for scanning state
  if (state != prior_state) {
    prior_state = state;
    // Move to top left to begin scanning
  }
  // Wait for button press before scanning
  current_time = millis(); // Get current time for checking state transisions

  // Check for state transisions
  if (current_time >= last_button_read + DEBOUNCE_INTERVAL) { // Guard state transition behind if statment to counteract debounce
    
  }
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

  prior_state = NONE;
  state = READY;

  uint32_t last_button_read  = 0; // Last time the button was read as pressed. Set to 0 on setup. #TODO: Confirm if variable def this way good practice
}

void loop() {
  switch (state) { // This looped section is the core of the state machine.
    case SCANNING:
      scanning();
      break;
  }
}
