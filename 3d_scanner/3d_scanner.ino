// 3D scanner firmware for ENGR2110 using two servos and an infrared distance
// sensor. A finite state machine is used to control the state of the scanner and 

#include <Servo.h>
#include <math.h>
Servo pan_servo;  // create servo object for the pan servo
Servo tilt_servo; // create servo object for the tilt servo

// Sensor variables
int sensor_value = 0; // Raw output from the sensor
float voltage = 0; // Raw output from the sensor converted to voltage
int current_scan_count = 0; // Number of scans currently completed by the sensor
float total_distance; // Addition of all scans for averaging

// Pin definitions
const uint8_t RED_LED = 3;
const uint8_t GREEN_LED = 5;
const uint8_t YELLOW_LED = 6;
const uint8_t TILT_SERVO = 11;
const uint8_t PAN_SERVO = 10;
const uint8_t INFRARED_SENSOR = A0;
const uint8_t SWITCH1 = 7;

// Time variables
uint32_t current_time;
uint32_t last_button_read;
bool debounce_flag;
uint32_t prior_time;
float t = 0;
float out;

// User parameters of the 3D scanner and letter
const uint16_t DEBOUNCE_INTERVAL = 1; // Minimum interval to wait before checking button presses 
float scanning_width_angle; // Calculated half width the scanner should scan (degrees)
float scanning_height_lower_angle; // Calculated lower angle the scanner should scan (degrees)
float scanning_height_upper_angle; // Calculated upper angle the scanner should scan (degrees)
float scanning_width_total_angle; // Calculated width the scanner should scan (degrees)
float scanning_height_total_angle; // Calculated height the scanner should scan (degrees)
float x_angle; // Angle corresponding to the angle of the pan servo
float y_angle; // Angle corresponding to the angle of the tilt servo
int current_mesh_points_x; // Current number of mesh points measured in x
int current_mesh_points_y; // Current number of mesh points measured in y

// User settings of the 3D scanner
// const uint8_t scanning_resolution = 1; // Angular resolution of the scanner (degrees) //TODO: Delete if unused
const uint8_t mesh_points_x = 100; // Number of mesh points to measure in x
const uint8_t mesh_points_y = 100; // Number of mesh points to measure in y
int scan_count = 10; // Number of sensor measurements to take per mesh point
const float scanner_distance = 0.35; // Distance from the sensor to the letter (m)
const float sensor_height = 0.16; // Distance from the sensor to the ground (m)
const float letter_height = 0.359; // Height of the letter to be scanned (m)
const float letter_width = 0.23; // Width of the letter to be scanned (m)
const float letter_tolerance = 0.1; // Tolerance to measure to the sides of the letter (m)
const uint16_t scanning_interval = 4; // Interval to wait between servo moves and scans (milliseconds)

// Structure to hold data from sensor converted to x,y,z, coordinates
struct coordinates{
  float x;
  float y;
  float z;
};
coordinates scan = {0, 0, 0};

// Define states for the 3D scanner
enum states {
  NONE,
  IDLE,
  SCANNING
};

// Define relative states
states prior_state, state;

void calculate_bounding_box() {
  // Based on the letter height, width, distance, and tolerance, calculate the angular bounding box
  float scanning_height = letter_height + 2 * letter_tolerance;
  float scanning_width = letter_width + 2 * letter_tolerance;
  scanning_width_angle = (180*atan((scanning_width/2)/scanner_distance))/PI;

  scanning_height_lower_angle = (180*atan((sensor_height + letter_tolerance)/scanner_distance))/PI;
  scanning_height_upper_angle = (180*atan((letter_height - sensor_height + letter_tolerance)/scanner_distance))/PI;
  scanning_width_total_angle = scanning_width_angle*2;
  scanning_height_total_angle = scanning_height_upper_angle + scanning_height_lower_angle;
//  Serial.println(scanning_width_angle);
//  Serial.println(scanning_height_lower_angle);
//  Serial.println(scanning_height_upper_angle);
}

void calculate_scan_coordinates(float pan_angle, float tilt_angle) {
  // Based on the pan and tilt angle calculate the coordinate at which the sensor is scanning
  scan.y = scanner_distance*tan((PI*pan_angle)/180);
  scan.z = scanner_distance*tan((PI*tilt_angle)/180);
  scan.x = sqrt(pow(scan.y,2) + pow(scan.z,2) + pow(scanner_distance,2));
}

void idle() {
  // Initialization for scanning state
  if (state != prior_state) {
    prior_state = state;
  }

  // Wait for button press before scanning
  current_time = millis(); // Get current time for checking state transisions

  // Idle LEDs
  if (current_time - prior_time > 10 ){
    t += 0.0005;
    out = sin(t)* 127.5 + 127.5;
  }
  analogWrite(RED_LED, out);   
  analogWrite(GREEN_LED, out);
  analogWrite(YELLOW_LED, out);

  // Check for state transision
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

void set_position(float x, float y) {
  // Set the absolute position of the sensor in the angular domain
  pan_servo.write(90-x);
  tilt_servo.write(90-y);
}

void scanning() {
  // Initialization for scanning state. Move into position and scan first point
  if (state != prior_state) {
    prior_state = state;
    calculate_bounding_box();
    // Set first point to scan in the top left corner
    x_angle = -scanning_width_angle;
    y_angle = scanning_height_upper_angle;
    current_mesh_points_x = 0;
    current_mesh_points_y = 0;
    set_position(x_angle,y_angle);
    delay(300);
    scan_point();
    current_mesh_points_x += 1;
    x_angle += scanning_width_total_angle/mesh_points_x;
  }
  current_time = millis();
  // If the current number of y mesh points is less than the total, continue scanning in x and y
  if (current_mesh_points_y <= mesh_points_y-1) {
    // If the current number of x mesh points is less than the total, continue scanning in x
    if (current_mesh_points_x < mesh_points_x-1) {
      if (current_time >= prior_time + scanning_interval) { // Prevent servo movements faster than scanning_inverval
        prior_time = current_time;
        set_position(x_angle, y_angle);
        scan_point();
        current_mesh_points_x += 1;
        // If on an odd row, scan in the x opposite direction
        if ((current_mesh_points_y % 2) == 0) {
          x_angle += scanning_width_total_angle/mesh_points_x;
        } else {
          x_angle -= scanning_width_total_angle/mesh_points_x;
        }
      }
    } else {
      // Once the current row has been scanned, move down in y
      if (current_time >= prior_time + scanning_interval) {
        prior_time = current_time;
        set_position(x_angle, y_angle);
        scan_point();
        y_angle -= scanning_height_total_angle/mesh_points_y;
        current_mesh_points_y += 1;
        current_mesh_points_x = 0;
      }
    }
  } else {
    // Return servo positions to zero and set state to idle upon state exit
    pan_servo.write(90);
    tilt_servo.write(90);
    state = IDLE;
  }
}

void scan_point() {
  // Scan and print data converted into x,y,z coordinates
  calculate_scan_coordinates(x_angle, y_angle);
  while (scan_count < scan_count) {
    sensor_value = analogRead(INFRARED_SENSOR);
    voltage = sensor_value * (5.0 / 1023.0);
    total_distance += log((voltage - 0.5)/4)/(-3.5);
    scan_count++;
  }
  scan.x = total_distance/scan_count - scan.x + scanner_distance;
  total_distance = 0;
  scan_count = 0;
  Serial.print(scan.x);
  Serial.print(", ");
  Serial.print(scan.y);
  Serial.print(", ");
  Serial.println(scan.z);
}

void setup() {
  Serial.begin(250000);
  
  // Define servos, inputs, and outputs
  pan_servo.attach(PAN_SERVO);
  tilt_servo.attach(TILT_SERVO);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(SWITCH1, INPUT);

  prior_state = NONE;
  state = IDLE;

  last_button_read  = 0; // Last time the button was read as pressed. Set to 0 on setup
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
