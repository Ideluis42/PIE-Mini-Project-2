// 3D scanner firmware for ENGR2110 using two servos and an infrared distance
// sensor. A finite state machine is used to control the state of the scanner and 

#include <Servo.h>
#include <math.h>
Servo pan_servo;  // create servo object for the pan servo
Servo tilt_servo; // create servo object for the tilt servo

const uint8_t RED_LED = 5;
const uint8_t GREEN_LED = 6;
const uint8_t YELLOW_LED = 11;
const uint8_t TILT_SERVO = 9;
const uint8_t PAN_SERVO = 10;
const uint8_t INFRARED_SENSOR = 0;
const uint8_t SWITCH1 = 8;
uint32_t current_time;
uint32_t last_button_read;
bool debounce_flag;
uint32_t prior_time;
float t = 0;
float out;

// User parameters of the 3D scanner and letter
const float scanner_distance = 0.4; // Distance from the sensor to the letter (m)
const float sensor_height = 0.1; // Distance from the sensor to the ground (m)
const float letter_height = 0.3556; // Height of the letter to be scanned (m)
const float letter_width = 0.2286; // Width of the letter to be scanned (m)
const float letter_tolerance = 0.05; // Tolerance to measure to the sides of the letter (m)
const uint16_t scanning_interval = 10; // Interval to wait between servo moves and scans (milliseconds)
const uint16_t DEBOUNCE_INTERVAL = 10; // Minimum interval to wait before checking button presses 
float scanning_width_angle; // 
float scanning_height_lower_angle; //
float scanning_height_upper_angle; //

// User settings of the 3D scanner
// const uint8_t scanning_resolution = 1; // Angular resolution of the scanner (degrees) //TODO: Delete if unused
const uint8_t mesh_points_x = 10; //
const uint8_t mesh_points_y = 10; //

// Define states for the 3D scanner
enum states {
  NONE,
  IDLE,
  SCANNING
};

// Define relative states
states prior_state, state;

void calculate_bounding_box() {
  // Based on the reading of the IMU and the height and distance of the sensor
  // from the letter, calculate the angular bounding box to scan.
  // Get absolute pitch, roll, and yaw from the IMU
  float scanning_height = letter_height + 2 * letter_tolerance;
  //Serial.println(letter_height);
  float scanning_width = letter_width + 2 * letter_tolerance;
  //Serial.println(scanning_width);

  // Calculate the angles corresponding to half of the scanning range
  scanning_width_angle = (180*atan((scanning_width/2)/scanner_distance))/PI;
  scanning_height_lower_angle = (180*atan((sensor_height + letter_tolerance)/scanner_distance))/PI;
  scanning_height_upper_angle = (180*atan((letter_height - sensor_height + letter_tolerance)/scanner_distance))/PI;
  Serial.println(scanning_width_angle);
  Serial.println(scanning_height_lower_angle);
  Serial.println(scanning_height_upper_angle);
}

void idle() {
  // Initialization for scanning state
  if (state != prior_state) {
    prior_state = state;
    // Move to top left to begin scanning
  }
  // Wait for button press before scanning
  current_time = millis(); // Get current time for checking state transisions

  if (current_time - prior_time > 10 ){
  
    t += 0.0005;
    out = sin(t)* 127.5 + 127.5;
  }

  analogWrite(RED_LED, out);   
  analogWrite(GREEN_LED, out);
  analogWrite(YELLOW_LED, out);

  // Check for state transisions
  if (current_time >= last_button_read + DEBOUNCE_INTERVAL) { // Guard state transition behind if statment to counteract debounce
    if (digitalRead(SWITCH1) == HIGH && debounce_flag) {
      state = SCANNING;
      digitalWrite(RED_LED, LOW);   
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      debounce_flag = false;
    }
    if (digitalRead(SWITCH1) == LOW && !debounce_flag) {
      debounce_flag = true;
    }
    last_button_read = millis();
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
  for (float i=-scanning_width_angle; i <= scanning_width_angle; i+=0.01) {
    pan_servo.write(i+90);
    //Serial.println(i);
    delay(1);
  }
  for (float i=-scanning_height_upper_angle; i <= scanning_height_lower_angle; i+=0.01) {
    tilt_servo.write(i+90);
    //Serial.println(i);
    delay(1);
  }

  // Return servo positions to zero upon state exit
  pan_servo.write(90);
  tilt_servo.write(90);
  state = IDLE;
}

//void scan_width(bool ) {

void setup() {
  Serial.begin(9600);
  
  // Define servos, inputs, and outputs
  pan_servo.attach(PAN_SERVO);
  tilt_servo.attach(TILT_SERVO);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(SWITCH1, INPUT);

  prior_state = NONE;
  state = SCANNING; //TODO: Change to "IDLE" upon code completion

  last_button_read  = 0; // Last time the button was read as pressed. Set to 0 on setup. #TODO: Confirm if variable def this way good practice
  prior_time = 0;
  debounce_flag = true;
}

void loop() {
  switch (state) { // This looped section is the core of the state machine.
    case IDLE:
      idle();
      break;
    case SCANNING:
      scanning();
      break;
  }
}
