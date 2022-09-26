// 3D scanner firmware for ENGR2110 using two servos and an infrared distance
// sensor. A finite state machine is used to control the state of the scanner and 

// To Do:
// Calculate x and y distance (pre sent parameters for how many points,
// but will need to troubleshoot
// Create vectors/arrays to store x,y points
// Send arrays to matlab

#include <Servo.h>
#include <math.h>
Servo pan_servo;  // create servo object for the pan servo
Servo tilt_servo; // create servo object for the tilt servo

// Arrays to store calculated x and y distance of number
const int points_measured = 9; // to change when we know how many points to take
float x_distances[points_measured];
float y_distances[points_measured];

int sensor_value = 0;
float total_measured_distances = 0;
float distance = 0;
float voltage = 0;
int scan_count = 0;
float total_distance;

const uint8_t RED_LED = 3;
const uint8_t GREEN_LED = 5;
const uint8_t YELLOW_LED = 6;
const uint8_t TILT_SERVO = 11;
const uint8_t PAN_SERVO = 10;
const uint8_t INFRARED_SENSOR = A0;
const uint8_t SWITCH1 = 7;
uint32_t current_time;
uint32_t last_button_read;
bool debounce_flag;
uint32_t prior_time;
float t = 0;
float out;

// User parameters of the 3D scanner and letter
const float scanner_distance = 0.35; // Distance from the sensor to the letter (m)
const float sensor_height = 0.16; // Distance from the sensor to the ground (m)
const float letter_height = 0.359; // Height of the letter to be scanned (m)
const float letter_width = 0.23; // Width of the letter to be scanned (m)
const float letter_tolerance = 0.1; // Tolerance to measure to the sides of the letter (m)
const uint16_t scanning_interval = 100; // Interval to wait between servo moves and scans (milliseconds)
const uint16_t DEBOUNCE_INTERVAL = 1; // Minimum interval to wait before checking button presses 
float scanning_width_angle; // 
float scanning_height_lower_angle; //
float scanning_height_upper_angle; //
float scanning_width_total_angle; //
float scanning_height_total_angle; //
float x_position; //
float y_position; //
int current_mesh_points_x;
int current_mesh_points_y;

// User settings of the 3D scanner
// const uint8_t scanning_resolution = 1; // Angular resolution of the scanner (degrees) //TODO: Delete if unused
const uint8_t mesh_points_x = 50; //
const uint8_t mesh_points_y = 50; //

// Structure to hold x,y,z data from sensor
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
  scanning_width_total_angle = scanning_width_angle*2;
  scanning_height_total_angle = scanning_height_upper_angle + scanning_height_lower_angle;
//  Serial.println(scanning_width_angle);
//  Serial.println(scanning_height_lower_angle);
//  Serial.println(scanning_height_upper_angle);
}

void calculate_scan_coordinates(float pan_angle, float tilt_angle) {
  scan.y = scanner_distance*tan((PI*pan_angle)/180);
  scan.z = scanner_distance*tan((PI*tilt_angle)/180);
  scan.x = sqrt(pow(scan.y,2) + pow(scan.z,2) + pow(scanner_distance,2));
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

void set_position(float x, float y) {
  // Set the absolute position of the sensor in the angular domain
  pan_servo.write(90-x);
  tilt_servo.write(90-y);
}

void scanning() {
  // Initialization for scanning state
  if (state != prior_state) {
    prior_state = state;
    calculate_bounding_box();
    // Set first point to scan in the top left corner
    x_position = -scanning_width_angle;
    y_position = scanning_height_upper_angle;
    current_mesh_points_x = 0;
    current_mesh_points_y = 0;
    set_position(x_position,y_position);
    delay(300);
    scan_point();
    current_mesh_points_x += 1;
    x_position += scanning_width_total_angle/mesh_points_x;
  }
  current_time = millis();
//  Serial.print(current_mesh_points_x);
//  Serial.print("\t");
//  Serial.println(current_mesh_points_y);
  if (current_mesh_points_y <= mesh_points_y-1) {
    if (current_mesh_points_x < mesh_points_x-1) {
      if (current_time >= prior_time + scanning_interval) {
        prior_time = current_time;
        set_position(x_position, y_position);
        // Scanning:
        // can change num of scans per distance, but 50 is a good place to start
        scan_point();
        // calculate distance on xy plane
        // save distance in x_distance and y_distance
        //Serial.println(x_position);
        //Serial.println(distance);
        current_mesh_points_x += 1;
        if ((current_mesh_points_y % 2) == 0) {
          x_position += scanning_width_total_angle/mesh_points_x;
        } else {
          x_position -= scanning_width_total_angle/mesh_points_x;
        }
      }
    } else {
      if (current_time >= prior_time + scanning_interval) {
        prior_time = current_time;
        set_position(x_position, y_position);
        scan_point();
        //calculate_scan_coordinates(x_position, y_position);
        y_position -= scanning_height_total_angle/mesh_points_y;
        current_mesh_points_y += 1;
        current_mesh_points_x = 0;
      }
    }
  } else {
    // Return servo positions to zero upon state exit
    pan_servo.write(90);
    tilt_servo.write(90);
    state = IDLE;
  }
}

void scan_point() {
  calculate_scan_coordinates(x_position, y_position);
  while (scan_count < 1) {
    sensor_value = analogRead(INFRARED_SENSOR);
    voltage = sensor_value * (5.0 / 1023.0);
    total_distance += log((voltage - 0.5)/4)/(-3.5);
    //Serial.println(total_distance);
    scan_count++;
  }
  scan.x = total_distance/10 - scan.x + scanner_distance;
  total_distance = 0;
  scan_count = 0;
  Serial.print(scan.x);
  Serial.print(", ");
  Serial.print(scan.y);
  Serial.print(", ");
  Serial.println(scan.z);
  //Serial.print(", ");
  //Serial.print(current_mesh_points_x);
  //Serial.print(", ");
  //Serial.println(current_mesh_points_y);
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
  state = IDLE; //TODO: Change to "IDLE" upon code completion

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
