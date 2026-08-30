#include <LSM303.h>
#include <Wire.h>
#include <millisDelay.h>  // include delay timer library (from SafeString library)
#include <TinyGPS.h>
#include <Servo.h>
#include <EEPROM.h>
#include <math.h>
#include <Pixy2SPI_SS.h>
#include <Pixy2CCC.h>  
#include <bits/stdc++.h>

#define Bluetooth_RX 35
#define Bluetooth_TX 34

#define right_limit 22
#define left_limit 23
#define rear_limit 21

#define THROTTLE_PIN 2
#define STEERING_PIN 3
#define DEAD_MAN_PIN 4

#define RC_TOGGLE 0  // Initialize RC_TOGGLE check (1 == Check for remote control, 0 == Don't check for remote control)

#define hall_effect_one 7
#define hall_effect_two 24
#define hall_effect_three 29
#define hall_effect_four 30

#define compass_offset 12.84

#define ultra_sonic_left   6
#define ultra_sonic_right  31
#define ultra_sonic_middle 5


int right_limit_state;
int left_limit_state;
int rear_limit_state;
volatile double holding_turn = 0;

/////////////////////////////////limit switch Variables////////////////////////////////////////////// 
short limit_last_direction = 0; //0 = left, 1 = right

/////////////////////////////////////////////////////////////////////////////// 

/////////////////////////////////GPS Variables////////////////////////////////////////////// 
short current_waypoint_target = 0;

//lat long speed
double waypoint_array [5][6] = {  //goal lat, goal long, speed, search for cone, time limit in millis, use align and search

    {37.3270933, -121.89155077, 125, 0, 15000,0}, //second cone (0.5) orignial -121.8915782 
    {37.32736958367491, -121.89176964390656, 130, 0, 12000,0},
    {37.327432273672194, -121.89188033305484, 112, 1, 60000,0},
    {37.327432273672194, -121.89188033305484, 112, 1, 60000,0}, 
    {37.327432273672194, -121.89188033305484, 112, 1, 60000,0} //final cone (goal)

        // {37.32765158613123, -121.89206431310139, 130, 0, 60000,0}

    };

static double CURRENT_LAT = 0;   // Initialize current latitude to 0
static double CURRENT_LONG = 0;  // Initialize current longitude to 0 //for test added long

double LAT_ARRAY[5] = { 0, 0, 0, 0, 0 };
double LONG_ARRAY[5] = { 0, 0, 0, 0, 0 };

static double temp_current_lat_for_swap = 0;

volatile double CURRENT_HEADING = 0;  // Initialize current heading of rover to 0 degrees
double TARGET_LAT = 0;                // Initialize target latitude to 0
double TARGET_LONG = 0;               // Initialize target longitude to 0
int TARGET_SPEED = 0;
volatile double DISTANCE = 0;        // Initialize distance variable to 0
volatile double TARGET_HEADING = 0;  // Initialize target heading to 0
volatile double ANGLE_TURN = 0;      // Initialize angle provisional to 0

volatile double RELATIVE_LONG = 0;
volatile double RELATIVE_LAT = 0;

const double HEADING_ERROR = 10;  // Initialize margin of error for TurnToHeading()

volatile double heading = 0;

volatile double current_heading_error = 0;
volatile double CURRENT_WAYPOINT_P_ERROR = 0;

volatile double current_dis_from_goal = 0;

float turn_kP = 0.4;
float turn_kI = 0;
float turn_kD = 0;

///////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////COMPASS////////////////////////////
LSM303 COMPASS;
LSM303::vector<int16_t> running_min = { 32767, 32767, 32767 }, running_max = { -32768, -32768, -32768 };
short MINIMUM_X, MINIMUM_Y, MINIMUM_Z, MAXIMUM_X, MAXIMUM_Y, MAXIMUM_Z;
char report[80];

short EE_PROM_MIN_X, EE_PROM_MIN_Y, EE_PROM_MIN_Z, EE_PROM_MAX_X, EE_PROM_MAX_Y, EE_PROM_MAX_Z;
double COMPASS_ARRAY[5] = { 0, 0, 0, 0, 0 };
/////////////////////////////////////////////////////////////////

////////////////////////////SERVO///////////////////////////////////
Servo ESC_MOTOR;
Servo TURN_SERVO;
/////////////////////////////////////////////////////////////////

/////////////////////////////Vision///////////////////////////////
bool away_from_center = false;
bool just_swapped_direction = false;
bool vision_turning_right = false;
double initial_vision = 0;
unsigned long CONE_TIMER = 0;
enum visionState
{
  REST,   // Turn off vision sensor
  SEARCH, // Search for the cone
  ALIGN,  // Align rover with cone detected
  PURSUIT // Drive into the cone detected
};

enum visionState STATE;

// Vision Sensor Variables
Pixy2SPI_SS pixy; // Pixy2 camera object
#define VISION_ERROR_MARGIN 17 // Error margin for checking OBJECT_CENTER_TO_CAM --> Avoid multiples of 5
#define CAMERA_CENTER 157 // Pixel center of camera screen
#define ROVER_CENTER_ERROR 27 // Error margin for checking if rover is aligned with cone --> Avoid multiples of 5
bool OBJECT_CENTER_TO_CAM; // Check for whether camera is centered to the screen
bool rightConeCollision;
bool leftConeCollision;

unsigned int CENTER_OF_OBJECT = 0; // Initialize variable for tracking center of object detected
int DISTANCE_TILL_CENTER = 0; // Distance between CAMERA_CENTER & CENTER_OF_OBJECT
static int VISION_ANGLE; // Pan servo angle of vision sensor --> 0 == RIGHT, 1000 == LEFT
static int TILT_ANGLE = 600;
const int ROVER_CENTER = 420; // Angle at which vision sensor is aligned with rover
volatile int CAMERA_ROVER_DIFF = 0; // Difference between VISION_ANGLE & ROVER_CENTER

short ALIGN_SPEED = 102; // Initialize speed of alignment in AlignRover()
short PURSUIT_SPEED = 100; // Initialize speed of pursuit in PursueCone()
//////////////////////////////////////////////////////////////////

/////////////////////////////GPS///////////////////////////////
TinyGPS gps;
#define lat_offset 0
#define long_offset 0

double AVERAGE_CURRENT_LAT = 0;
double AVERAGE_CURRENT_LONG = 0;

int len_of_waypoint_array = sizeof(waypoint_array) / sizeof(waypoint_array[0]);
/////////////////////////////////////////////////////////////////

////////////////////////////HALL_EFFECT//////////////////////////////////////
int distance_travled = 0;
int hall_one_state;
int hall_two_state;
int hall_three_state;
int hall_four_state;
//////////////////////////////////////////////////////////////////////////

///////////////////////RC_CONTROLLER//////////////////////////////
volatile long TH_START_TIME = 0;
volatile long TH_CURRENT_TIME = 0;
volatile long THROTTLE_PULSE = 0;
int THROTTLE_PW = 0;
int THROTTLE_VALUE = 0;
int THROTTLE_CENTER = 1422;


volatile long ST_START_TIME = 0;
volatile long ST_CURRENT_TIME = 0;
volatile long STEERING_PULSE = 0;
int STEERING_PW = 0;
int STEERING_VALUE = 0;
int STEERING_CENTER = 1425;

volatile short DEAD_MAN_VALUE;
bool DEAD_MAN = false;
bool RC_CONTROLLER = false;
bool AUTON_CONTROL = true;

/////////////////////////////////TIMERS/////////////////////////////////////////
elapsedMillis limit_switch_timer;  //timers are in millis
#define limit_delay_millis 100

elapsedMillis GPS_timer;
#define GPS_delay_millis 700

elapsedMillis compass_timer;
#define compass_delay_millis 100

elapsedMillis waypoint_timeout_timer;

elapsedMillis vision_timeout_timer;

elapsedMillis align_timer;

///////////////////////////////////Hierarchy///////////////////////////////////////
// 0 = Limit switch
// 1 = Ultrasonics
// 2 = Hall effects
// 3 = Camera
// 4 = GPS
short current_hierarchy = 4;
//////////////////////////////////////////////////////////////////////////
void Throttle_timer() {
  TH_CURRENT_TIME = micros();
  if (TH_CURRENT_TIME > TH_START_TIME) {
    THROTTLE_PULSE = TH_CURRENT_TIME - TH_START_TIME;
    TH_START_TIME = TH_CURRENT_TIME;
  }
}

void Steering_timer() {
  ST_CURRENT_TIME = micros();
  if (ST_CURRENT_TIME > ST_START_TIME) {
    STEERING_PULSE = ST_CURRENT_TIME - ST_START_TIME;
    ST_START_TIME = ST_CURRENT_TIME;
  }
}

void RCREAD() {
  if (THROTTLE_PULSE < 2200) {
    THROTTLE_PW = THROTTLE_PULSE;
  }

  if (STEERING_PULSE < 2200) {
    STEERING_PW = STEERING_PULSE;
  }
}

void RC_SETUP() {
  // pinMode(THROTTLE_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), Throttle_timer, CHANGE);
  // pinMode(STEERING_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(STEERING_PIN), Steering_timer, CHANGE);
  pinMode(DEAD_MAN_PIN, INPUT);

  ESC_MOTOR.attach(9);
  TURN_SERVO.attach(8);

  ESC_MOTOR.write(90);
  TURN_SERVO.write(90);
}

void RC_Drive() {
  RCREAD();
  //Both are in R position'

  //Throttle
  //back = 1954 forward = 1113

  //Steering
  //back = 1610 forward = 1227

  if (THROTTLE_PW < 1422)
    THROTTLE_VALUE = map(THROTTLE_PW, 1122, 1422, 0, 90);
  else if (THROTTLE_PW > 1422)
    THROTTLE_VALUE = map(THROTTLE_PW, 1422, 1962, 90, 180);
  else
    THROTTLE_VALUE = 90;
  THROTTLE_VALUE = constrain(THROTTLE_VALUE, 0, 180);
  ESC_MOTOR.write(THROTTLE_VALUE);

  if (STEERING_VALUE < 1420)
    STEERING_VALUE = map(STEERING_PW, 1225, 1420, 0, 90);
  else if (STEERING_VALUE > 1420)
    STEERING_VALUE = map(STEERING_PW, 1420, 1610, 90, 180);
  else
    STEERING_VALUE = 90;
  STEERING_VALUE = constrain(STEERING_VALUE, 0, 180);
  TURN_SERVO.write(STEERING_VALUE);
}

void Hall_Effect_test() {
  hall_one_state = digitalRead(hall_effect_one);
  hall_two_state = digitalRead(hall_effect_two);
  hall_three_state = digitalRead(hall_effect_three);
  hall_four_state = digitalRead(hall_effect_four);
  Serial8.printf("ONE : %d TWO : %d THREE : %d FOUR : %d\n", hall_one_state, hall_two_state, hall_three_state, hall_four_state);
}

void Hall_effect_calibration() {
  //reset to hall one zero position
  hall_one_state = digitalRead(hall_effect_one);
  hall_two_state = digitalRead(hall_effect_two);
  hall_three_state = digitalRead(hall_effect_three);
  hall_four_state = digitalRead(hall_effect_four);
  Serial8.printf("Calibrating : ONE : %d TWO : %d THREE : %d FOUR : %d\n", hall_one_state, hall_two_state, hall_three_state, hall_four_state);

  while (hall_one_state) {
    hall_one_state = digitalRead(hall_effect_one);
    hall_two_state = digitalRead(hall_effect_two);
    hall_three_state = digitalRead(hall_effect_three);
    hall_four_state = digitalRead(hall_effect_four);
    Serial8.printf("Calibrating : ONE : %d TWO : %d THREE : %d FOUR : %d\n", hall_one_state, hall_two_state, hall_three_state, hall_four_state);
    ESC_MOTOR.write(100);
  }
  ESC_MOTOR.write(90);
  Serial8.println("finished hall effect calibration");
}

void Hall_effect_setup() {
  pinMode(hall_effect_one, INPUT);
  pinMode(hall_effect_two, INPUT);
  pinMode(hall_effect_three, INPUT);
  pinMode(hall_effect_four, INPUT);
}

void pause_and_check_lat_long(){
  TURN_SERVO.write(90);
  ESC_MOTOR.write(90);

  double LAT_ARRAY[20] = { 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0};

  double LONG_ARRAY[20] = { 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0};

  while(LAT_ARRAY[0] == 0 && LONG_ARRAY[0] == 0){
  Serial8.println("averaging gps");
  CurrentCoordinates();

  if((LAT_ARRAY[19]!=LAT_ARRAY[18])||(LONG_ARRAY[19]!=LONG_ARRAY[18])){
  for (int i = 0; i < 19; i++){
        LAT_ARRAY[i] = LAT_ARRAY[i+1];
        LONG_ARRAY[i] = LONG_ARRAY[i+1];
      }
  }
  LAT_ARRAY[19] = CURRENT_LAT;
  LONG_ARRAY[19] = CURRENT_LONG;
    
//     float LAT_ARRAY_ONE = LAT_ARRAY[0];
//     float LAT_ARRAY_TWO = LAT_ARRAY[1];
//     float LAT_ARRAY_THREE = LAT_ARRAY[2];
//     float LAT_ARRAY_FOUR = LAT_ARRAY[3];
//     float LAT_ARRAY_FIVE = LAT_ARRAY[4];
//     float LAT_ARRAY_SIX = LAT_ARRAY[5];
//     float LAT_ARRAY_SEVEN = LAT_ARRAY[6];
//     float LAT_ARRAY_EIGHT = LAT_ARRAY[7];
//     float LAT_ARRAY_NINE = LAT_ARRAY[8];
//     float LAT_ARRAY_TEN = LAT_ARRAY[9];
//     float LAT_ARRAY_ELEVEN = LAT_ARRAY[10];
//     float LAT_ARRAY_TWELVE = LAT_ARRAY[11];
//     float LAT_ARRAY_THIRTEEN = LAT_ARRAY[12];
//     float LAT_ARRAY_FOURTEEN = LAT_ARRAY[13];
//     float LAT_ARRAY_FIFTEEN = LAT_ARRAY[14];
//     float LAT_ARRAY_SIXTEEN = LAT_ARRAY[15];
//     float LAT_ARRAY_SEVENTEEN = LAT_ARRAY[16];
//     float LAT_ARRAY_EIGHTEEN = LAT_ARRAY[17];
//     float LAT_ARRAY_NINETEEN = LAT_ARRAY[18];
//     float LAT_ARRAY_TWENTY = LAT_ARRAY[19];

//     Serial8.printf("LAT:ARRAY:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f\n",LAT_ARRAY_ONE,
//                                                                                              LAT_ARRAY_TWO,
//                                                                                              LAT_ARRAY_THREE,
//                                                                                              LAT_ARRAY_FOUR,
//                                                                                              LAT_ARRAY_FIVE,
//                                                                                              LAT_ARRAY_SIX,
//                                                                                              LAT_ARRAY_SEVEN,
//                                                                                              LAT_ARRAY_EIGHT,
//                                                                                              LAT_ARRAY_NINE,
//                                                                                              LAT_ARRAY_TEN,
//                                                                                              LAT_ARRAY_ELEVEN,
//                                                                                              LAT_ARRAY_TWELVE,
//                                                                                              LAT_ARRAY_THIRTEEN,
//                                                                                              LAT_ARRAY_FOURTEEN,
//                                                                                              LAT_ARRAY_FIFTEEN,
//                                                                                              LAT_ARRAY_SIXTEEN,
//                                                                                              LAT_ARRAY_SEVENTEEN,
//                                                                                              LAT_ARRAY_EIGHTEEN,
//                                                                                              LAT_ARRAY_NINETEEN,
//                                                                                              LAT_ARRAY_TWENTY
//                                                                                             );

// float LONG_ARRAY_ONE = LONG_ARRAY[0];
//     float LONG_ARRAY_TWO = LONG_ARRAY[1];
//     float LONG_ARRAY_THREE = LONG_ARRAY[2];
//     float LONG_ARRAY_FOUR = LONG_ARRAY[3];
//     float LONG_ARRAY_FIVE = LONG_ARRAY[4];
//     float LONG_ARRAY_SIX = LONG_ARRAY[5];
//     float LONG_ARRAY_SEVEN = LONG_ARRAY[6];
//     float LONG_ARRAY_EIGHT = LONG_ARRAY[7];
//     float LONG_ARRAY_NINE = LONG_ARRAY[8];
//     float LONG_ARRAY_TEN = LONG_ARRAY[9];
//     float LONG_ARRAY_ELEVEN = LONG_ARRAY[10];
//     float LONG_ARRAY_TWELVE = LONG_ARRAY[11];
//     float LONG_ARRAY_THIRTEEN = LONG_ARRAY[12];
//     float LONG_ARRAY_FOURTEEN = LONG_ARRAY[13];
//     float LONG_ARRAY_FIFTEEN = LONG_ARRAY[14];
//     float LONG_ARRAY_SIXTEEN = LONG_ARRAY[15];
//     float LONG_ARRAY_SEVENTEEN = LONG_ARRAY[16];
//     float LONG_ARRAY_EIGHTEEN = LONG_ARRAY[17];
//     float LONG_ARRAY_NINETEEN = LONG_ARRAY[18];
//     float LONG_ARRAY_TWENTY = LONG_ARRAY[19];

//     Serial8.printf("LONG:ARRAY:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f\n",LONG_ARRAY_ONE,
//                                                                                              LONG_ARRAY_TWO,
//                                                                                              LONG_ARRAY_THREE,
//                                                                                              LONG_ARRAY_FOUR,
//                                                                                              LONG_ARRAY_FIVE,
//                                                                                              LONG_ARRAY_SIX,
//                                                                                              LONG_ARRAY_SEVEN,
//                                                                                              LONG_ARRAY_EIGHT,
//                                                                                              LONG_ARRAY_NINE,
//                                                                                              LONG_ARRAY_TEN,
//                                                                                              LONG_ARRAY_ELEVEN,
//                                                                                              LONG_ARRAY_TWELVE,
//                                                                                              LONG_ARRAY_THIRTEEN,
//                                                                                              LONG_ARRAY_FOURTEEN,
//                                                                                              LONG_ARRAY_FIFTEEN,
//                                                                                              LONG_ARRAY_SIXTEEN,
//                                                                                              LONG_ARRAY_SEVENTEEN,
//                                                                                              LONG_ARRAY_EIGHTEEN,
//                                                                                              LONG_ARRAY_NINETEEN,
//                                                                                              LONG_ARRAY_TWENTY
//                                                                                          );
  }

    AVERAGE_CURRENT_LAT = LAT_ARRAY[0] + LAT_ARRAY[1] + LAT_ARRAY[2] + LAT_ARRAY[3] + LAT_ARRAY[4]
                          + LAT_ARRAY[5] + LAT_ARRAY[6] + LAT_ARRAY[7] + LAT_ARRAY[8] + LAT_ARRAY[9]
                          + LAT_ARRAY[10] + LAT_ARRAY[11] + LAT_ARRAY[12] + LAT_ARRAY[13] + LAT_ARRAY[14]
                          + LAT_ARRAY[15] + LAT_ARRAY[16] + LAT_ARRAY[17] + LAT_ARRAY[18] + LAT_ARRAY[19];
    AVERAGE_CURRENT_LAT = AVERAGE_CURRENT_LAT / 20;

    AVERAGE_CURRENT_LONG = LONG_ARRAY[0] + LONG_ARRAY[1] + LONG_ARRAY[2] + LONG_ARRAY[3] + LONG_ARRAY[4]
                          + LONG_ARRAY[5] + LONG_ARRAY[6] + LONG_ARRAY[7] + LONG_ARRAY[8] + LONG_ARRAY[9]
                          + LONG_ARRAY[10] + LONG_ARRAY[11] + LONG_ARRAY[12] + LONG_ARRAY[13] + LONG_ARRAY[14]
                          + LONG_ARRAY[15] + LONG_ARRAY[16] + LONG_ARRAY[17] + LONG_ARRAY[18] + LONG_ARRAY[19];
    AVERAGE_CURRENT_LONG = AVERAGE_CURRENT_LONG / 20;
    Serial8.printf("ave lat:%f long:%f\n",AVERAGE_CURRENT_LAT,AVERAGE_CURRENT_LONG);
    for (int i = 0; i <= 19; i++){
        LAT_ARRAY[i] = 0;
        LONG_ARRAY[i] = 0;
      }
    
}
void Compass_setup() {
  Wire.begin();  //I2C device uses first wire on board (SCL0 to pin 19, SDA0 to pin 18)
  COMPASS.init();
  COMPASS.enableDefault();
}

void GPSSetup() {
  // Start the hardware serial communication with the BN220 GPS module
  Serial3.begin(38400);
  Serial3.setTX(14);  // TX3 pin
  Serial3.setRX(15);  // RX3 pin
}

void read_compass() {
  COMPASS.read();
    heading = COMPASS.heading() + compass_offset;
    // heading = fmod(heading,360);
    // if(heading < 0)
    //   heading += 360;
  //  for (int i = 0; i < 4; i++) {
  //   COMPASS_ARRAY[i] = COMPASS_ARRAY[i + 1];
  // }
  // COMPASS_ARRAY[4] = heading;
  // heading = COMPASS_ARRAY[4] * 5
  //           + COMPASS_ARRAY[3] * 4
  //           + COMPASS_ARRAY[2] * 3
  //           + COMPASS_ARRAY[1] * 2
  //           + COMPASS_ARRAY[0] * 1;
  // heading = heading / 15;
  //Serial8.printf("heading:%d\n",heading);
}

void calibrate_compass() {
  //Serial8.printf("reading compass\n");
  //Serial8.printf("calibrating compass\n");
  COMPASS.read();
  //Serial8.printf("COMPASS READ\n");
  running_min.x = min(running_min.x, COMPASS.m.x);
  running_min.y = min(running_min.y, COMPASS.m.y);
  running_min.z = min(running_min.z, COMPASS.m.z);

  running_max.x = max(running_max.x, COMPASS.m.x);
  running_max.y = max(running_max.y, COMPASS.m.y);
  running_max.z = max(running_max.z, COMPASS.m.z);

  // Serial8.printf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",

  //   running_min.x, running_min.y, running_min.z,
  //   running_max.x, running_max.y, running_max.z);
  //Serial8.print("min: {%+6d, %+6d, %+6d}   max: {%+6d, %+6d, %+6d}");
  Serial8.print(running_min.x);
  Serial8.print(" ");  // Add space between values
  Serial8.print(running_min.y);
  Serial8.print(" ");  // Add space between values
  Serial8.print(running_min.z);
  Serial8.print("    ");  // Add spaces for better formatting
  Serial8.print(running_max.x);
  Serial8.print(" ");  // Add space between values
  Serial8.print(running_max.y);
  Serial8.print(" ");  // Add space between values
  Serial8.print(running_max.z);
  Serial8.println();  // Add newline at the end
  //Serial.println("past printing");
  Serial.println(report);

  EE_PROM_MIN_X = running_min.x;
  EE_PROM_MIN_Y = running_min.y;
  EE_PROM_MIN_Z = running_min.z;

  EE_PROM_MAX_X = running_max.x;
  EE_PROM_MAX_Y = running_max.y;
  EE_PROM_MAX_Z = running_max.z;
}

void write_compass_to_EEPROM() {
  //minimum values then maximum values
  EEPROM.put(24,EE_PROM_MIN_X);
  EEPROM.put(28,EE_PROM_MIN_Y);
  EEPROM.put(32,EE_PROM_MIN_Z);


  EEPROM.put(36,EE_PROM_MAX_X); 
  EEPROM.put(40,EE_PROM_MAX_Y);
  EEPROM.put(44,EE_PROM_MAX_Z);
}

void read_EEPROM_VALS(int last_line_read) {
  Serial8.printf("READING EEPROM VALS\n");
  byte data;
  for (int current_address = 0; current_address <= last_line_read; current_address++) {
    data = EEPROM.read(current_address);
    Serial8.printf("Address : %i : ", current_address);
    Serial8.println(data);
    delay(20);
  }
}

void get_EEPROM_VALS(int last_line_read) {
  Serial8.printf("getting EEPROM VALS\n");
  int data;
  for (int current_address = 0; current_address <= last_line_read * 4 - 1; current_address = current_address + 4) {
    EEPROM.get(current_address, data);
    Serial8.printf("Address : %i : ", current_address);
    Serial8.println(data);
    delay(20);
  }
}

void get_COMPASS_VALS_FROM_EEPROM() {
  Serial8.printf("getting compass values from EEPROM\n");
  EEPROM.get(24,MINIMUM_X);
  EEPROM.get(28,MINIMUM_Y);
  EEPROM.get(32,MINIMUM_Z);

  EEPROM.get(36,MAXIMUM_X);
  EEPROM.get(40,MAXIMUM_Y);
  EEPROM.get(44,MAXIMUM_Z);


  // Serial8.printf("min_x : %hi\n",MINIMUM_X);
  // Serial8.printf("min_y : %hi\n",MINIMUM_Y);
  // Serial8.printf("min_z : %hi\n",MINIMUM_Z);
  // Serial8.printf("max_x : %hi\n",MAXIMUM_X);
  // Serial8.printf("max_x : %hi\n",MAXIMUM_Y);
  // Serial8.printf("max_x : %hi\n",MAXIMUM_Z);
}

void update_compass_heading() {
  read_compass();
  //Serial8.printf("unrestricted heading:%f\n",heading);
  if (heading < 0)
    heading += 360;
  if (heading > 360)
    heading -= 360;

  
  for(int i = 0; i <4; i++){
    COMPASS_ARRAY[i] = COMPASS_ARRAY[i+1]; 
  }
  COMPASS_ARRAY[4] = radians(heading); 

  double cartesian_array[5][2]{{0,0},
                               {0,0},
                               {0,0},
                               {0,0},
                               {0,0}};
  
  for (int i=0; i<5; i++){
    cartesian_array[i][0] = cos(COMPASS_ARRAY[i]);
    cartesian_array[i][1] = sin(COMPASS_ARRAY[i]);
  }
  // double temp_one = cartesian_array[0][0];
  // double temp_two = cartesian_array[1][0];
  // double temp_three = cartesian_array[2][0];
  // double temp_four = cartesian_array[3][0];
  // double temp_five = cartesian_array[4][0];
  // Serial8.printf("cartesian: %f:%f:%f:%f:%f\n",temp_one,temp_two,temp_three,temp_four,temp_five);

  double average_x_position = cartesian_array[0][0]
                      +cartesian_array[1][0]
                      +cartesian_array[2][0]
                      +cartesian_array[3][0]
                      +cartesian_array[4][0];
  average_x_position = average_x_position/5;
  //Serial8.printf("aver_x:%f\n",average_x_position);

  double average_y_position = cartesian_array[0][1]
                      +cartesian_array[1][1]
                      +cartesian_array[2][1]
                      +cartesian_array[3][1]
                      +cartesian_array[4][1];
  average_y_position = average_y_position/5;

  double theta = atan2(average_y_position,average_x_position);
  theta = degrees(theta);
  if (theta < 0)
    theta = theta + 360;
  heading = theta;
  //Serial8.printf("theta:%f\n",theta);

  //  temp_one = COMPASS_ARRAY[0];
  //  temp_two = COMPASS_ARRAY[1];
  //  temp_three = COMPASS_ARRAY[2];
  //  temp_four = COMPASS_ARRAY[3];
  //  temp_five = COMPASS_ARRAY[4];
  // Serial8.printf("COMPASS_ARRAY: %f:%f:%f:%f:%f\n",temp_one,temp_two,temp_three,temp_four,temp_five);

  //Serial8.printf("heading:%f theta:%f\n",heading,theta);
  //Serial8.printf("current heading:%f\n",heading);
}


void CurrentCoordinates(){
  // Parse current latitude and longitude from GPS serial messages
  
  // Check if there is data available on the GPS module
  if (Serial3.available()>0) {
    // Read the data from the GPS module and save as a string
    String input_string = Serial3.readStringUntil('\n');
        
    input_string.remove(12,1); // Remove irrelevant characters from the beginning of the string
    short index1 = input_string.indexOf(",N"); // Look for the index of N
    short index2 = input_string.indexOf(",E"); // Look for the index of E
    bool south = false; // Initialize as false, otherwise false positives will occur
    bool west = false; // Initialize as false, otherwise false positives will occur

    // Check the directions of the parsed coordinate
    if (index1 == -1){ // If N isn't found, the coordinate is South
      index1 = input_string.indexOf(",S");
      south = true;
    }
    if (index2 == -1){ // If E isn't found, the coordinate is West
      index2 = input_string.indexOf(",W");
      west = true;
    }
    
    // Extract the latitude value and convert it to a float
    String lat_string = input_string.substring(index1-10, index1);
    double temp_lat; // declare temp_lat variable for assignment later
    
    if (south){ // If coordinate is South, make the value negative
      temp_lat = -1 * (atof(lat_string.c_str()) / 100); // if South, make negative
    }
    else{ // If coordinate is North, keep positive
      temp_lat = atof(lat_string.c_str()) / 100; // if North, make positive
    }
    
    double PREV_LAT = CURRENT_LAT;
    if (temp_lat != 0){
      CURRENT_LAT = temp_lat; // update CURRENT_LAT only if given good data
      // Convert latitude from "degree.minute" to "degree.decimal"
      int degree_lat = (int)(CURRENT_LAT);
      degree_lat = (double)(degree_lat);
      double minute_lat = CURRENT_LAT - degree_lat;
      double decimal_lat = minute_lat * 100 / 60;
      CURRENT_LAT = degree_lat + decimal_lat - lat_offset;
      if(abs(CURRENT_LAT - 37.318314)>0.14492){
        CURRENT_LAT = PREV_LAT;
        Serial8.printf("wrong lat\n");
      }
    }
    
    // Extract the longitude value and convert it to a float
    String long_string = input_string.substring(index2-11, index2);
    double temp_long; // Declare temp_lat variable for assignment later
    
    if (west){ // If coordinate is West, make value negative
      temp_long = -1 * (atof(long_string.c_str()) / 100);
    }
    else{ // If coordinate is East, keep positive
      temp_long = atof(long_string.c_str()) / 100;
    }
    
    double PREV_LONG = CURRENT_LONG;
    if(temp_long  != 0){
      CURRENT_LONG = temp_long; // Update CURRENT_LONG only if given good data
      // Convert longitude from "degree.minute" to "degree.decimal"
      int degree_long = (int)(CURRENT_LONG);
      degree_long = (double)(degree_long);
      double minute_long = CURRENT_LONG - degree_long;
      double decimal_long = minute_long * 100 / 60;
      CURRENT_LONG = degree_long + decimal_long - long_offset;
      if(abs(CURRENT_LONG - (-121.8761343149291))>0.14492){
        CURRENT_LONG = PREV_LONG;
        Serial8.printf("wrong long\n");
      }
    }
  }
  Serial8.printf("LAT:%f:LONG:%f\n",CURRENT_LAT,CURRENT_LONG);
}

void update_limit_states() {
  right_limit_state = digitalRead(right_limit);
  left_limit_state = digitalRead(left_limit);
  rear_limit_state = digitalRead(rear_limit);
  //Serial.printf("right: %d left: %d rear:%d \n", right_limit_state,left_limit_state,rear_limit_state);
}

double calculate_angle_to_lat_long(double goal_lat, double goal_long) {
  //CurrentCoordinates();
  TARGET_LONG = goal_long;
  TARGET_LAT = goal_lat;
  //Serial8.printf("g_lat :%f g_long :%f\n",goal_lat,goal_long);
  RELATIVE_LONG = TARGET_LONG - CURRENT_LONG;
  RELATIVE_LAT = TARGET_LAT - CURRENT_LAT;
  //Serial8.printf("rel_long: %f rel_lat: %f  \n",RELATIVE_LONG,RELATIVE_LAT);
  // if((RELATIVE_LONG > 0) && (RELATIVE_LAT > 0)){
  //         TARGET_HEADING = atan2(RELATIVE_LAT,RELATIVE_LONG);
  //         TARGET_HEADING = TARGET_HEADING * 180/3.1415;
  //         TARGET_HEADING = 90 - TARGET_HEADING ;
  //         //Serial8.printf("quadrant 1");

  //       } else if ((RELATIVE_LONG > 0) && (RELATIVE_LAT < 0)){
  //           TARGET_HEADING = atan2(RELATIVE_LAT,RELATIVE_LONG);
  //           TARGET_HEADING = TARGET_HEADING * 180/3.1415;
  //           TARGET_HEADING = 90 + TARGET_HEADING * -1;
  //           //Serial8.printf("quadrant 4");

  //       } else if ((RELATIVE_LONG < 0) && (RELATIVE_LAT < 0)){
  //           TARGET_HEADING = atan2(RELATIVE_LAT,RELATIVE_LONG);
  //           TARGET_HEADING = TARGET_HEADING * 180/3.1415;
  //           TARGET_HEADING = 90 - TARGET_HEADING;
  //           //Serial8.printf("quadrant 3");

  //       } else if ((RELATIVE_LONG < 0) && (RELATIVE_LAT > 0)){
  //           TARGET_HEADING = atan2(RELATIVE_LAT,RELATIVE_LONG);
  //           TARGET_HEADING = TARGET_HEADING * 180/3.1415;
  //           TARGET_HEADING = 180-TARGET_HEADING + 270;
  //           //Serial8.printf("quadrant 2");
  //       }
  //       return TARGET_HEADING;
  double initial_lat = CURRENT_LAT * M_PI / 180;
  double initial_long = CURRENT_LONG * M_PI / 180;
  double final_lat = TARGET_LAT * M_PI / 180;
  double final_long = TARGET_LONG * M_PI / 180;

  double long_diff = final_long - initial_long;  // Difference between longitudes of two points



  double y = sin(long_diff) * cos(final_lat);
  double x = (cos(initial_lat) * sin(final_lat)) - (sin(initial_lat) * cos(final_lat) * cos(long_diff));
  TARGET_HEADING = atan2(y, x);  // returns as -pi --> pi

  TARGET_HEADING = TARGET_HEADING * (180 / M_PI);      // Convert target heading from radians to degrees (given in -180-180 range)
  TARGET_HEADING = fmod(TARGET_HEADING + 360.0, 360);  // Normalize to 0-360 compass bearing
  // fmod = floating point modulus % --> fmod(x, y) == x % y

  if (TARGET_HEADING < 0) {                 // If target heading becomes negative
    TARGET_HEADING = TARGET_HEADING + 360;  // Flip over 0-360 boundary
  }

  //Serial8.printf("TARGET_HEADING:%f  \n", TARGET_HEADING);
  return TARGET_HEADING;
}

double calculate_angle_to_lat_long_for_average(double goal_lat, double goal_long) {
  //CurrentCoordinates();
  TARGET_LONG = goal_long;
  TARGET_LAT = goal_lat;
  //Serial8.printf("g_lat :%f g_long :%f\n",goal_lat,goal_long);
  RELATIVE_LONG = TARGET_LONG - AVERAGE_CURRENT_LONG;
  RELATIVE_LAT = TARGET_LAT - AVERAGE_CURRENT_LAT;
  //Serial8.printf("rel_long: %f rel_lat: %f  \n",RELATIVE_LONG,RELATIVE_LAT);
  // if((RELATIVE_LONG > 0) && (RELATIVE_LAT > 0)){
  //         TARGET_HEADING = atan2(RELATIVE_LAT,RELATIVE_LONG);
  //         TARGET_HEADING = TARGET_HEADING * 180/3.1415;
  //         TARGET_HEADING = 90 - TARGET_HEADING ;
  //         //Serial8.printf("quadrant 1");

  //       } else if ((RELATIVE_LONG > 0) && (RELATIVE_LAT < 0)){
  //           TARGET_HEADING = atan2(RELATIVE_LAT,RELATIVE_LONG);
  //           TARGET_HEADING = TARGET_HEADING * 180/3.1415;
  //           TARGET_HEADING = 90 + TARGET_HEADING * -1;
  //           //Serial8.printf("quadrant 4");

  //       } else if ((RELATIVE_LONG < 0) && (RELATIVE_LAT < 0)){
  //           TARGET_HEADING = atan2(RELATIVE_LAT,RELATIVE_LONG);
  //           TARGET_HEADING = TARGET_HEADING * 180/3.1415;
  //           TARGET_HEADING = 90 - TARGET_HEADING;
  //           //Serial8.printf("quadrant 3");

  //       } else if ((RELATIVE_LONG < 0) && (RELATIVE_LAT > 0)){
  //           TARGET_HEADING = atan2(RELATIVE_LAT,RELATIVE_LONG);
  //           TARGET_HEADING = TARGET_HEADING * 180/3.1415;
  //           TARGET_HEADING = 180-TARGET_HEADING + 270;
  //           //Serial8.printf("quadrant 2");
  //       }
  //       return TARGET_HEADING;
  double initial_lat = CURRENT_LAT * M_PI / 180;
  double initial_long = CURRENT_LONG * M_PI / 180;
  double final_lat = TARGET_LAT * M_PI / 180;
  double final_long = TARGET_LONG * M_PI / 180;

  double long_diff = final_long - initial_long;  // Difference between longitudes of two points



  double y = sin(long_diff) * cos(final_lat);
  double x = (cos(initial_lat) * sin(final_lat)) - (sin(initial_lat) * cos(final_lat) * cos(long_diff));
  TARGET_HEADING = atan2(y, x);  // returns as -pi --> pi

  TARGET_HEADING = TARGET_HEADING * (180 / M_PI);      // Convert target heading from radians to degrees (given in -180-180 range)
  TARGET_HEADING = fmod(TARGET_HEADING + 360.0, 360);  // Normalize to 0-360 compass bearing
  // fmod = floating point modulus % --> fmod(x, y) == x % y

  if (TARGET_HEADING < 0) {                 // If target heading becomes negative
    TARGET_HEADING = TARGET_HEADING + 360;  // Flip over 0-360 boundary
  }

  //Serial8.printf("TARGET_HEADING:%f  \n", TARGET_HEADING);
  return TARGET_HEADING;
}

void drive_test() {
  Serial8.printf("wait 5 secs to turn on ESC\n");
  delay(5000);

  TURN_SERVO.write(120);
  ESC_MOTOR.write(100);
  delay(1000);
  TURN_SERVO.write(60);
  ESC_MOTOR.write(80);
  delay(1000);
  ESC_MOTOR.write(90);
  TURN_SERVO.write(90);
  Serial8.printf("drive forward for 2 seconds\n");
}

void turn_to_direction(double goal_direction) {
  update_compass_heading();
  current_heading_error = goal_direction - heading;
  align_timer = 0;
  Serial8.printf("goal:%f heading:%f\n",goal_direction,heading);
  while (abs(current_heading_error) > 2) {
    if(align_timer > 10000)
      break;
    update_compass_heading();
    Serial8.println(heading);

    current_heading_error = goal_direction - heading;
    if (current_heading_error > 180)
      current_heading_error -= 360;
    if (current_heading_error < -180)
      current_heading_error += 360;

    if (current_heading_error < 0) {
      TURN_SERVO.write(0);
      Serial8.printf("servo turned\n");
      ESC_MOTOR.write(76);
      Serial8.printf("Starting driving\n");
    } else if (current_heading_error > 0) {
      TURN_SERVO.write(180);
      Serial8.printf("servo turned\n");
      ESC_MOTOR.write(76);
      Serial8.printf("Starting driving\n");
    }
  }
  TURN_SERVO.write(90);
  ESC_MOTOR.write(96);
  delay(1000);
  ESC_MOTOR.write(90);
  Serial8.printf("Finished turning driving\n");
}

void drive_straight_in_direction(int goal_direction) {
  while (true) {
    read_compass();
    heading = COMPASS.heading() + compass_offset;
    Serial8.println(heading);

    current_heading_error = goal_direction - heading;
    Serial8.printf("error: %d\n", current_heading_error);

    if (abs(current_heading_error) < 5) {
      TURN_SERVO.write(90);
      ESC_MOTOR.write(90);
      break;
    }
    //   //delay (500);
  }
}

void swap_lat_long() {
  temp_current_lat_for_swap = CURRENT_LAT;
  CURRENT_LAT = CURRENT_LONG;
  CURRENT_LONG = temp_current_lat_for_swap;
}

void turn_to_point(double goal_lat, double goal_long) {
  calculate_angle_to_lat_long(goal_lat, goal_long);
  turn_to_direction(TARGET_HEADING);
}

double calculate_dis_to_point(double goalLat, double goalLon, double currentLat, double currentLon) {

  //Radius of Earth in m
  const double R = 6371.0 * 1000;

  //convert coordinates into radians
  //Serial8.printf("g_la:%f g_lo:%f c_la:%f c_lo:%f\n", goalLat, goalLon, CURRENT_LAT, CURRENT_LONG);
  goalLat = radians(goalLat);
  goalLon = radians(goalLon);
  currentLat = radians(currentLat);
  currentLon = radians(currentLon);

  //difference in coordinates
  double difference_Lat = goalLat - currentLat;
  double difference_Lon = goalLon - currentLon;

  //apply haversine formula
  //double a = sin(difference_Lat / 2) **2 + cos(goalLat) * cos(currentLat) * sin(difference_Lon /2) **2;
  double a = sin(difference_Lat / 2) * sin(difference_Lat / 2) + cos(goalLat) * cos(currentLat) * sin(difference_Lon / 2) * sin(difference_Lon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = R * c;
  return distance;  // return distance in meters
}

void set_current_waypoint(double target_lat, double target_long) {
  TARGET_LAT = target_lat;
  TARGET_LONG = target_long;
}

void set_target_speed(int speed) {
  TARGET_SPEED = speed;
}

void drive_to_point_using_waypoints(){
  ESC_MOTOR.write(waypoint_array[current_waypoint_target][2]);
  DISTANCE = calculate_dis_to_point(waypoint_array[current_waypoint_target][0], waypoint_array[current_waypoint_target][1], CURRENT_LAT, CURRENT_LONG);
  TARGET_HEADING = calculate_angle_to_lat_long(waypoint_array[current_waypoint_target][0], waypoint_array[current_waypoint_target][1]);
  CURRENT_WAYPOINT_P_ERROR = TARGET_HEADING - heading;
  if (CURRENT_WAYPOINT_P_ERROR > 180)
    CURRENT_WAYPOINT_P_ERROR -= 360;
  if (CURRENT_WAYPOINT_P_ERROR < -180)
    CURRENT_WAYPOINT_P_ERROR += 360;

  holding_turn = 90 - CURRENT_WAYPOINT_P_ERROR * turn_kP;
  if (holding_turn > 150) {
      holding_turn = 150;
    } else if (holding_turn < 30) {
      holding_turn = 30;
    }
    //Serial8.printf("heading error: %f holding:%f\n", CURRENT_WAYPOINT_P_ERROR, holding_turn);
    TURN_SERVO.write(holding_turn);
  
  if(abs(DISTANCE) < 2){ //distance in meters
      if(waypoint_array[current_waypoint_target][5] == 1){
      pause_and_check_lat_long();
      double averaged_heading = calculate_angle_to_lat_long_for_average(waypoint_array[current_waypoint_target][0], waypoint_array[current_waypoint_target][1]);
      turn_to_direction(averaged_heading);
      TURN_SERVO.write(90);
      ESC_MOTOR.write(104);
      delay(5000);
    }
    if(waypoint_array[current_waypoint_target][3] == 1){
      Serial8.println("about to target cone");
      STATE = SEARCH;
      current_hierarchy = 3;
      vision_timeout_timer = 0;
    }
    current_waypoint_target++;
    waypoint_timeout_timer = 0;
    Serial8.printf("current waypoint:%hi\n",current_waypoint_target); 
  }
  if(waypoint_timeout_timer > waypoint_array[current_waypoint_target][4]){
    if(waypoint_array[current_waypoint_target][3] == 1){
      Serial8.println("about to target cone");
      STATE = SEARCH;
      current_hierarchy = 3;
      vision_timeout_timer = 0;
    }
    current_waypoint_target++;
    waypoint_timeout_timer = 0;
    Serial8.printf("current waypoint:%hi\n",current_waypoint_target); 
  }

}

void drive_to_point_PID(double goal_lat, double goal_long, int speed) {
  //turn_to_point (goal_lat,goal_long);
  //currentcoordininates();
  read_compass();
  heading = COMPASS.heading() + compass_offset;
  Serial8.printf("Compass read\n");
  ESC_MOTOR.write(speed);
  volatile double temp_error = 0;
  DISTANCE = calculate_dis_to_point(goal_lat, goal_long, CURRENT_LAT, CURRENT_LONG);
  //Serial8.printf("g_lat:%f g_lon:%f c_lat:%f c_lon:%f\n",goal_lat,goal_long,CURRENT_LAT,CURRENT_LONG);
  Serial8.printf("dis:%f\n", DISTANCE);
  double goal_direction = calculate_angle_to_lat_long(goal_lat, goal_long);
  while (abs(DISTANCE > 3)) {

    read_compass();
    heading = COMPASS.heading() + compass_offset;
    if (heading > 360) {
      heading -= 360;
    } else if (heading < -360)
      heading += 360;

    ESC_MOTOR.write(speed);
    DISTANCE = calculate_dis_to_point(goal_lat, goal_long, CURRENT_LAT, CURRENT_LONG);
    goal_direction = calculate_angle_to_lat_long(goal_lat, goal_long);

    current_heading_error = goal_direction - heading;

    if (current_heading_error > 180)
      current_heading_error -= 360;
    if (current_heading_error < -180)
      current_heading_error += 360;

    temp_error = current_heading_error;

    holding_turn = 90 - temp_error * turn_kP;
    if (holding_turn > 140) {
      holding_turn = 140;
    } else if (holding_turn < 40) {
      holding_turn = 40;
    }
    Serial8.printf("heading error: %f holding:%f\n", current_heading_error, holding_turn);
    TURN_SERVO.write(holding_turn);
    delay(10);
  }
  ESC_MOTOR.write(90);
  TURN_SERVO.write(90);
}

void heading_hold(double time_millis, int speed, double goal_direction) {
  read_compass();
  heading = COMPASS.heading() + compass_offset;
  Serial8.printf("Compass read\n");
  ESC_MOTOR.write(speed);
  volatile double temp_error = 0;

  for (int i = 0; i < time_millis; i++) {
    read_compass();
    heading = COMPASS.heading() + compass_offset;
    if (heading > 360) {
      heading -= 360;
    } else if (heading < -360)
      heading += 360;

    current_heading_error = goal_direction - heading;

    if (current_heading_error > 180)
      current_heading_error -= 360;
    if (current_heading_error < -180)
      current_heading_error += 360;

    temp_error = current_heading_error;

    holding_turn = 90 - temp_error * turn_kP;
    if (holding_turn > 135) {
      holding_turn = 135;
    } else if (holding_turn < 45) {
      holding_turn = 45;
    }
    Serial8.printf("current: %f target :%f error:%f\n", heading, goal_direction, current_heading_error);
    TURN_SERVO.write(holding_turn);
    //delay(10);
  }
  TURN_SERVO.write(90);
  ESC_MOTOR.write(90);
}

void update_sensors() {
  if (limit_switch_timer > limit_delay_millis) {
    update_limit_states();
    limit_switch_timer = 0;
  }

  if (GPS_timer > GPS_delay_millis) {
    CurrentCoordinates();
    GPS_timer = 0;
  }

  if (compass_timer > compass_delay_millis) {
    update_compass_heading();
    compass_timer = 0;
  }

  
}

void limit_switch_pressed(){
  if (right_limit_state == 1 && limit_last_direction == 0) { //if the right limit switch is turned to high, whcih means that the left whisker makes contact with object 
        Serial.printf("right_limit_state\n");
        Serial8.println("turn right");
        TURN_SERVO.write(0); //turn the wheel right 
        ESC_MOTOR.write(75); // drive reverse
        limit_last_direction = 1;
        delay(1500);
        TURN_SERVO.write(90);
        ESC_MOTOR.write(105);
        delay(1500);
  }else if(right_limit_state == 1 && limit_last_direction == 1){
        Serial.printf("right_limit_state\n");
        Serial8.println("turn left");
        TURN_SERVO.write(180); //turn the wheel left
        ESC_MOTOR.write(75); // drive reverse
        limit_last_direction = 0;
        delay(1500);
        TURN_SERVO.write(90);
        ESC_MOTOR.write(105);
        delay(1500);
  }
  if (left_limit_state == 1 && limit_last_direction == 1) { //if the left limit switch is turned to high, whcih means that the right whisker makes contact with object 
        Serial.printf("left_limit_state\n");
        Serial8.println("turn right");
        TURN_SERVO.write(0); //turn the wheel right 
        ESC_MOTOR.write(75); // drive reverse
        limit_last_direction = 0;
        delay(1500);
  }else if (left_limit_state == 1 && limit_last_direction == 0){
        Serial.printf("left_limit_state\n");
        Serial8.println("turn left");
        TURN_SERVO.write(180); //turn the wheel left
        ESC_MOTOR.write(75); // drive reverse
        limit_last_direction = 1;
        delay(1500);
  }
  if (rear_limit_state == 1) { //if the read limit switch is turned to high, whcih means that the rear bumper makes contact with object 
        Serial.printf("rear_limit_state\n");
        TURN_SERVO.write(180); //turn the wheel right 
        ESC_MOTOR.write(105); // drive forward
        delay(1000);
      }
}

void Vision_Setup(){
  pixy.init();
  pixy.setLamp(1,1);
  STATE = SEARCH; // Start vision sensor in search mode
  VISION_ANGLE = ROVER_CENTER; // Start VISION_ANGLE at center of rover
  pixy.setServos(VISION_ANGLE, TILT_ANGLE); // Set vision sensor pan and tilt positions
}

//////////////////////Vision Functions///////////////////////////////
// Update the state of the vision sensor
void TargetCone(enum visionState){
  Serial8.println("Targeting cone");
  waypoint_timeout_timer = 0;
  pixy.ccc.getBlocks(); // Look for a cone
  if(vision_timeout_timer < 50000){

    switch(STATE)
    {
      case REST: // End vision sensor routine
        ESC_MOTOR.write(90); // Stop rover and enter idle state
        Serial8.println("rest");
        current_hierarchy = 4;
        return;
        break;
      case SEARCH: // Cone not found, searching for cone
        SearchMode(); // Enter search mode
        break;
      case ALIGN: // Cone found, aligning rover with cone
        CONE_TIMER = millis(); // Start a timer for intermediate cone check
        AlignRover(1, 2); // Enter alignmet mode
        break;
      case PURSUIT: // Rover aligned with cone, attempting to collide with cone
        CONE_TIMER = millis(); // Start a timer for intermediate cone check
        PursueCone(); // Enter cone pursuit
        break;
      default:
        Serial.println("no state set / broken");
    }
  }
  else{
    Serial8.println("Vision given up");
    current_hierarchy = 4;
    waypoint_timeout_timer = 0;
    return;
  }

}
/*----------------------------------------------------------------------------------------------------------------------*/
// Pan servo until a cone is detected. If no cone found, turn 120 degrees and search again
void SearchMode(){
  Serial8.println("Cone not found");
  VISION_ANGLE = ROVER_CENTER;
  pixy.setServos(VISION_ANGLE, TILT_ANGLE); // Center vision sensor

  // while(VISION_ANGLE != 0 && STATE != ALIGN){ // Pan to the right
  //   Serial8.println("First while loop");
  //   //Serial.println("Pan right");
  //   //VISION_ANGLE -= 5;
  //   pixy.setServos(VISION_ANGLE, TILT_ANGLE);
  //   if(pixy.ccc.getBlocks() > 0){ // If cone detected, enter pursuit mode
  //   Serial8.println("Cone found");
  //     ESC_MOTOR.write(90); // Stop rover
  //     STATE = ALIGN;
  //     return;
  //   }
  // }
  // while(VISION_ANGLE != 1000 && STATE != ALIGN){ // Pan to the left
  // Serial8.println("Second while loop");
  // //Serial.println("Pan left");
  //   //VISION_ANGLE += 5;
  //   pixy.setServos(VISION_ANGLE, TILT_ANGLE);
  //   if(pixy.ccc.getBlocks() > 0){ // If cone detected, enter pursuit mode
  //   Serial.println("Cone detected");
  //     ESC_MOTOR.write(90); // Stop rover
  //     STATE = ALIGN;
  //     return;
  //   }
  // }


// if(just_swapped_direction == false vision_turning_right == false && away_from_center == false){
//   TURN_SERVO.write(180);
//   ESC_MOTOR.write(104);
//   just_swapped_direction = true;
// } else if(vision_turning_right == true && away_from_center == false){
//   TURN_SERVO.write(0);
//   ESC_MOTOR.write(104);
//   just_swapped_direction = true
// }
  TURN_SERVO.write(0);
  ESC_MOTOR.write(81);
  
  if(pixy.ccc.getBlocks() > 0){ // If cone detected, enter pursuit mode
    Serial.println("Cone detected");
    ESC_MOTOR.write(90); // Stop rover
    STATE = ALIGN;
    return;
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Align rover, vision sensor, and cone
void AlignRover(int signature1, int signature2){
  Serial8.println("ALigning rover");
  Serial8.printf("diff:%d centered:%",CAMERA_ROVER_DIFF,OBJECT_CENTER_TO_CAM);
  if(!rightConeCollision && ! leftConeCollision){
  if(pixy.ccc.blocks[0].m_signature == signature1) // If blocks detected are cones
  {  
    CENTER_OF_OBJECT = pixy.ccc.blocks[0].m_x; // Find the center of the object detected
    DISTANCE_TILL_CENTER = CENTER_OF_OBJECT - CAMERA_CENTER; // Distance between object center and camera center
    
    if(abs(DISTANCE_TILL_CENTER) <= VISION_ERROR_MARGIN){ 
      OBJECT_CENTER_TO_CAM = true; // Object is centered to the camera
    }
    else{
      OBJECT_CENTER_TO_CAM = false; // Object is not centered to the camera
    }

    CAMERA_ROVER_DIFF =  VISION_ANGLE - ROVER_CENTER; // Difference between vision sensor angle and rover center  
    Serial8.printf("Camera_diff:%d\n",CAMERA_ROVER_DIFF);
    if(abs(CAMERA_ROVER_DIFF) > ROVER_CENTER_ERROR && OBJECT_CENTER_TO_CAM){ // If rover is not yet aligned with cone
      // Proportional turn based on CAMERA_ROVER_DIFF
      STEERING_VALUE = map(CAMERA_ROVER_DIFF, -200, 200, 0, 180);  
      TURN_SERVO.write(STEERING_VALUE);
      ESC_MOTOR.write(ALIGN_SPEED);
    }
    else if(abs(CAMERA_ROVER_DIFF) <= ROVER_CENTER_ERROR && OBJECT_CENTER_TO_CAM){ // If rover is aligned with cone
      TURN_SERVO.write(90);
      ESC_MOTOR.write(90);
      STATE = PURSUIT; // Enter pursuit mode
    }
    else if(!OBJECT_CENTER_TO_CAM){ // If object is not centered on camera
      if(DISTANCE_TILL_CENTER < 0){ // If object is to the left
        if(VISION_ANGLE >= 1000){ // If cone is found at the edge of vision sensor range, turn anyways
          VISION_ANGLE = 1000; // Cap value of VISION_ANGLE at 1000
          TURN_SERVO.write(180);
          ESC_MOTOR.write(ALIGN_SPEED);
          pixy.setServos(VISION_ANGLE, TILT_ANGLE);
        }
        else{ // Keep panning vision sensor to the left
          VISION_ANGLE += 5;
          pixy.setServos(VISION_ANGLE, TILT_ANGLE);
        }
      }
      else if(DISTANCE_TILL_CENTER > 0){ // If object is to the right
        if(VISION_ANGLE <= 0){ // If cone is found at the edge of vision sensor range, turn anyways
          VISION_ANGLE = 0; // Cap value of VISION_ANGLE at 0
          TURN_SERVO.write(0);
          ESC_MOTOR.write(ALIGN_SPEED);
          pixy.setServos(VISION_ANGLE, TILT_ANGLE);
        }
        else{ // Keep panning vision sensor to the right
          VISION_ANGLE -= 5;
          pixy.setServos(VISION_ANGLE, TILT_ANGLE);
        }
      }
    }
  }
  else{ // If object lost, enter search mode again
    STATE = SEARCH;
  }
  }
  else{
    STATE = REST;
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Vision-guided pursuit of cone
void PursueCone(){
  limit_switch_cone_collision(); // Check for limit switch collision
  Serial8.println("Pursuing Cone");
  if(pixy.ccc.getBlocks() > 0) // If blocks detected are cones
  { 
    if( !rightConeCollision && !leftConeCollision){ // Run until cone is hit
      
      CENTER_OF_OBJECT = pixy.ccc.blocks[0].m_x; // Find the center of the object detected
      DISTANCE_TILL_CENTER = CENTER_OF_OBJECT - CAMERA_CENTER; // Distance between object center and camera center
      if(abs(DISTANCE_TILL_CENTER) <= VISION_ERROR_MARGIN + 150){ 
        OBJECT_CENTER_TO_CAM = true; // Object is centered to the camera, go forward
      }
      else{
        OBJECT_CENTER_TO_CAM = false; // Object is not centered to the camera
        STATE = SEARCH;
        return;
      }
      

  
      // Recalculate TURN_SERVO angle to hit cone
      if(DISTANCE_TILL_CENTER < 30){ // If cone is to the left
        STEERING_VALUE = 130;
      }
      else if(DISTANCE_TILL_CENTER > -30){ // If cone is to the right
        STEERING_VALUE = 50;
      }
      else{ // If cone is close to center
        STEERING_VALUE = 90;
      }
      TURN_SERVO.write(STEERING_VALUE);
      ESC_MOTOR.write(PURSUIT_SPEED);
    }
    else{
      STATE = REST; // End vision sensor loop
  
    }
  }
  else{
    STATE = SEARCH; // Reenter search mode
  }
}

void limit_switch_cone_collision(){
  if (right_limit_state == 1){
    Serial8.println("switch pressed");
    rightConeCollision = true;
  }
    else{
    rightConeCollision = false;
  }
  if(left_limit_state == 1){
    leftConeCollision = true;
  }  else{
    leftConeCollision = false;
  }
}

void setup() {
  // put your setup code here, to run once:

  Serial8.begin(9600);
  Serial8.setTX(Bluetooth_TX);
  Serial8.setRX(Bluetooth_RX);
  Serial8.printf("serial print\n");

  Compass_setup();
  Serial8.printf("COMPASS done\n");
  GPSSetup();
  Serial8.printf("GPS done\n");
  Vision_Setup();
  Serial8.printf("Vision done \n");
  RC_SETUP();
  Serial8.printf("RC done\n");

  ESC_MOTOR.write(90);
  TURN_SERVO.write(90);

  Serial8.printf("wait 5 secs to turn on ESC\n");
  for (int i = 0; i < 100; i++) {
    CurrentCoordinates();
    Serial8.printf("5 sec delay\n");
    delay(1);
  }
  
  while(CURRENT_LAT == 0 || CURRENT_LONG == 0){
    CurrentCoordinates();
  }

  pinMode(right_limit, INPUT_PULLDOWN);
  pinMode(left_limit, INPUT_PULLDOWN);
  pinMode(rear_limit, INPUT_PULLDOWN);

  //Hall_effect_calibration();

  Serial8.printf("starting setup\n");

  //heading_hold(double time_millis,int speed,int goal_direction)       // 90 speed is zero
  //turn_to_point(double goal_lat, double goal_long)                    // latitude, longtidue, just turns to face that direction
  //turn_to_direction(int goal_direction)                               // pick from 0 - 360, convention: 0 is north the angle starts from north going clockwise
  //drive_to_point_PID(double lat, double long, int speed)              // latitude, longitude , speed


  // Serial8.printf("starting heading hold\n");
  //heading_hold(60000, 102, 125);
  ///////////////////////////////////




  // drive_to_point_PID(34.047905010297875, -117.83750406237905,102);
  // drive_to_point_PID(34.047937559528194, -117.83769557014465,102);
  // drive_to_point_PID(34.04846648276984, -117.8376980253724,102);
  // drive_to_point_PID(34.0484095219639, -117.83750406237905,102);
}

void loop() {
  Serial8.printf("finished setup\n");
  //////////////////COMPASS CALIBRATION//////////////////////
  // while(true){
  //   //Serial8.printf("Calibrating\n");
  //   calibrate_compass();
  //   if(digitalRead(left_limit) == HIGH){
  //     write_compass_to_EEPROM();
  //     Serial8.printf("Finished Calibration\n");
  //     waypoint_timeout_timer = 0;
  //     break; // Exit calibration
  //   }
  // delay(50);
  // }
  while(left_limit==HIGH)
    delay(50);
  //////////////////////////////////////////////////////////
  get_COMPASS_VALS_FROM_EEPROM();
  COMPASS.m_min = (LSM303::vector<int16_t>){ MINIMUM_X, MINIMUM_Y, MINIMUM_Z };
  COMPASS.m_max = (LSM303::vector<int16_t>){ MAXIMUM_X, MAXIMUM_Y, MAXIMUM_Z };
  ////////////////////////////////////
  waypoint_timeout_timer = 0;
  while (true) {
    /////////////////////LIMIT_SWITCH////////////////////////////
    //update_limit_states();
    //////////////////////////////////////////////////


    ///////////////////////COMPASS/////////////////////////////
    // Serial8.printf("trying to read compass\n");
    // update_compass_heading();
    // heading = heading - 8;
    // Serial.printf("compass_dir: %f\n",heading);
    //////////////////////////////////////////////////////////

    ///////////////////////GPS/////////////////////////////
    //swap_lat_long();
    ///////////////////////////////////////////////////////
    //CurrentCoordinates();
    //Serial8.printf("Lat:%f Long:%f\n",CURRENT_LAT,CURRENT_LONG);

    //Serial8.printf("Lat: %f:%f:%f:%f:%f\n",LAT_ARRAY[0],LAT_ARRAY[1],LAT_ARRAY[2],LAT_ARRAY[3],LAT_ARRAY[4]);


    // DISTANCE = calculate_dis_to_point(34.046720469303565, -117.84574750833464, CURRENT_LAT, CURRENT_LONG);
    // Serial8.printf("distance %f \n",DISTANCE);
    //calculate_angle_to_lat_long(34.046172077675685, -117.84512228287008);
    //Serial8.printf("target_heading %f\n",TARGET_HEADING);
    ///////////////////////////////////////////////////////

    ////////////////////////BLUETOOTH//////////////////////////////////
    //Serial8.print("reading telemetry\n");
    /////////////////////////////////////////////////////////////////

    ////////////////////////////RC_CAR////////////////////////////////////
    //RC_Drive();
    //Serial8.printf("Throttle: %d Steering: %d: \n",THROTTLE_VALUE,STEERING_VALUE);
    /////////////////////////////////////////////////////////////////

    ///////////////////////////DEAD_MAN///////////////////////////////////
    // DEAD_MAN_VALUE = pulseIn(DEAD_MAN_PIN, HIGH);
    // Serial8.printf("DEAD MAN STATE: %u\n",DEAD_MAN_VALUE);
    /////////////////////////////////////////////////////////////////////

    /////////////////////////////HALL_EFFECT//////////////////////////////
    //Hall_Effect_test();
    /////////////////////////////////////////////////////////////////////



  ///////////////////////////////Main code//////////////////////////////////////////
    
    update_sensors();
    //Serial8.printf("current waypoint:%hi\n",current_waypoint_target); 
    if((left_limit_state == 1) || (right_limit_state == 1) || (rear_limit_state ==1))
      current_hierarchy = 0;
    //use else if statements to assess hierarchy 

    if(current_hierarchy == 0){ //limit switch
      limit_switch_pressed();
      current_hierarchy = 4;
    }else if(current_hierarchy == 3){
        TargetCone(STATE);
    } else if (current_hierarchy == 4){
      drive_to_point_using_waypoints();
    }

    //Serial8.printf("current waypoint:%hi\n",current_waypoint_target);
    if (current_waypoint_target >= len_of_waypoint_array){
     TargetCone(STATE);
     vision_timeout_timer = 0;
    } 
    /////////////////////////////////////////////////////////////////////
    //Serial8.printf("B_L:%f B_R:%f B_B:%f heading:%f lat:%f long:%f\n",left_limit,right_limit,rear_limit,heading,CURRENT_LAT,CURRENT_LONG);
  }
  // put your main code here, to run repeatedly:
  //test
}
