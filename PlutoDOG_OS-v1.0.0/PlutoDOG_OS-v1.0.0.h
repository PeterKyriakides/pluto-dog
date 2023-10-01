/*
  PlutoDOG_OS Header File   |  Version: 1.0.0
  by plutoLABS              |           07/05/2023
 --------------------------------------------------

*/

//**************************************************************************
// EEPROM / Battery Managment System Setup
//**************************************************************************
// Define EEPROM storage reading/writing size
#define EEPROM_SIZE 10

// Define memory location of maximum KNOWN battery voltage;
#define BATT_MAX_ADDR 5

//Define percentage battery cutoff (85% of 8.4V = 7.14)
#define BATT_CUTOFF 0.85 //[%]

// Define the hard cut-off for battery
#define BATT_STOP 6 //[V]

// Initialise dynamic maximum KNOWN battery voltage
float BATT_MAX;

// Initialise battery percentage
float BATT_PERCENTAGE;

//**************************************************************************
// Voltage Sensor Setup
//**************************************************************************
// Define voltage sensor analog pin input
#define VOLT_SENSOR_PIN 32

// Initialise current battery voltage
float BATT_CURRENT = 0.00;

//**************************************************************************
// Memory Managment System Setup
//**************************************************************************
// Define memory location of EEPROM write counter;
#define COUNTER_ADDR 0

// Initialise dynamic EEPROM write counter
int EEPROM_COUNTER;

// Define EEPROM initialisation flag
float EEPROM_isInit;

//**************************************************************************
// Voltage Regulator System Setup [6V Step Down]
//**************************************************************************
// Define enable pin on 6V regulator
#define VRS_PIN 18

// Define boolean to disable regulator system for movement
bool disableMovement = true;

//**************************************************************************
// Heartbeat Diagnostics Setup
//**************************************************************************
// Heartbeat timer interval:
const long heartbeatInterval = 3000;  //in (milliseconds)

//**************************************************************************
// Robotic PS3 Controller System Setup
//**************************************************************************
// Define controller initialisation flag
bool controller_isInit = false;

// Define controller battery status
int controllerBattery = 0;

// Define starting move of standing
bool move_isInit = false;

//**************************************************************************
// Limb Management System Setup:
//**************************************************************************
// Define enable pin on PCA9685 (limb) controller
#define LMS_PIN 19

// Define LMS check flags
bool limbs_isInit = false;
bool limbController_isInit = false;

// Define posture check flag
bool isStanding = false;
bool isWalking = false;

bool leanMode = false;

bool walkForward = false;
bool walkBack = false;
bool walkHalt = false;

bool pivotRight = false;
bool pivotLeft = false;

bool trotRight = false;
bool trotLeft = false;

// Define PCA9685 library using default B000000 (A5-A0) I2C address, and default Wire @400kHz
PCA9685 limbController;

// Define joystick trigger variables
int stickSenseMAX = 100;
int stickSenseMIN = 5;

// Limb Servo Speed Setting
//--------------------------------
unsigned long MAX_SPEED = 10.0; // Min no. of ms per deg || [default = 10]

// Limb Servo Channel Definition:
//--------------------------------
// - REAR RIGHT LEG:
#define shoulderX_RR 0
#define shoulderY_RR 1
#define elbowY_RR 2

// - REAR LEFT LEG:
#define shoulderX_RL 4
#define shoulderY_RL 5
#define elbowY_RL 6

// - FRONT LEFT LEG:
#define shoulderX_FL 8
#define shoulderY_FL 9
#define elbowY_FL 10

// - FRONT RIGHT LEG:
#define shoulderX_FR 12
#define shoulderY_FR 13
#define elbowY_FR 14

// Limb Servo Object Tuning   --> Default: PCA9685_ServoEval pwmServo(102, 512);
//--------------------------------
// SHOULDER_X SERVO OBJECTS: // Tuned for 0 deg --> Lay Postion
PCA9685_ServoEval shoulderX_RR_servo(102, 512);
PCA9685_ServoEval shoulderX_RL_servo(102, 512);
PCA9685_ServoEval shoulderX_FR_servo(102, 512);
PCA9685_ServoEval shoulderX_FL_servo(102, 512);

// SHOULDER_Y SERVO OBJECTS: // Tuned for 0 deg --> Lay Position
PCA9685_ServoEval shoulderY_RR_servo(102, 512);
PCA9685_ServoEval shoulderY_RL_servo(102, 512);
PCA9685_ServoEval shoulderY_FR_servo(102, 512);
PCA9685_ServoEval shoulderY_FL_servo(102, 512);

// ELBOW_Y SERVO OBJECTS:  // Tuned for 0 deg --> Lay Position
PCA9685_ServoEval elbowY_RR_servo(102, 512);
PCA9685_ServoEval elbowY_RL_servo(102, 512);
PCA9685_ServoEval elbowY_FR_servo(102, 512);
PCA9685_ServoEval elbowY_FL_servo(102, 512);

// Predefine Limb Servo Angle Structures
//--------------------------------
float currentAngle[4][3] = {{0, 0, 0},   // Front Right Leg  = {ShoulderX, ShoulderY, ElbowY}
                            {0, 0, 0},   // Rear Right Leg   = {ShoulderX, ShoulderY, ElbowY}
                            {0, 0, 0},   // Rear Left Leg    = {ShoulderX, ShoulderY, ElbowY}
                            {0, 0, 0}};  // Front Left Leg   = {ShoulderX, ShoulderY, ElbowY}
                            
float standAngle[4][3] = {{0, 0, 45},   // FR
                          {0, 0, 45},   // RR
                          {0, 0, 45},   // RL
                          {0, 0, 45}};  // FL

float standAngle_TALL[4][3] = {{0, 30, 90},   // FR
                          {0, 30, 90},   // RR
                          {0, 30, 90},   // RL
                          {0, 30, 90}};  // FL
                          
float layAngle[4][3] = {{0, 0, 0},      // FR
                        {0, 0, 0},      // RR
                        {0, 0, 0},      // RL
                        {0, 0, 0}};     // FL

float sitAngle[4][3] = {{0, 0, 90},     // FR
                        {0, -35, 10},   // RR
                        {0, -35, 10},   // RL
                        {0, 0, 90}};    // FL

float shakeAngle_UP[4][3] = {{0, 80, 90},    // FR
                             {0, -35, 30},   // RR
                             {0, -35, 0},    // RL
                             {0, 0, 70}};    // FL

float shakeAngle_DOWN[4][3] = {{0, 90, 90},     // FR
                               {0, -35, 30},    // RR
                               {0, -35, 0},     // RL
                               {0, 0, 70}};     // FL

float shakeAngle_SIT[4][3] = {{0, 0, 90},      // FR
                              {0, -35, 20},    // RR
                              {0, -35, 0},     // RL
                              {0, 0, 90}};     // FL

float attackAngle[4][3] = {{0, 20, 55},      // FR
                           {0, -20, 30},     // RR
                           {0, -20, 30},     // RL
                           {0, 20, 55}};     // FL

float curiousAngle[4][3] = {{20, 0, 80},      // FR
                            {20, -35, 30},      // RR
                            {-20, -35, 30},      // RL
                            {-20, 0, 80}};     // FL



// ************************ [OLD] LEANING COMMANDS ************************
float leanForward[4][3] = {{0, 15, 25},   // FR
                           {0, 15, 65},   // RR
                           {0, 15, 65},   // RL
                           {0, 15, 25}};  // FL
                           
float forwardRight[4][3] = {{-25, 15, 25},   // FR
                            {-25, 15, 65},   // RR
                            {25, 15, 65},    // RL
                            {25, 15, 25}};   // FL

float forwardLeft[4][3] = {{25, 15, 25},    // FR
                           {25, 15, 65},    // RR
                           {-25, 15, 65},   // RL
                           {-25, 15, 25}};  // FL
                           
float leanBack[4][3] = {{0, -15, 65},   // FR
                        {0, -15, 25},   // RR
                        {0, -15, 25},   // RL
                        {0, -15, 65}};  // FL

float backRight[4][3] = {{-25, -15, 65},  // FR
                         {-25, -15, 25},  // RR
                         {25, -15, 25},   // RL
                         {25, -15, 65}};  // FL

float backLeft[4][3] = {{25, -15, 65},    // FR
                        {25, -15, 25},    // RR
                        {-25, -15, 25},   // RL
                        {-25, -15, 65}};  // FL

float leanRight[4][3] = {{-25, 0, 45},  // FR
                         {-25, 0, 45},  // RR
                         {25, 0, 45},   // RL
                         {25, 0, 45}};  // FL

float leanLeft[4][3] = {{25, 0, 45},    // FR
                        {25, 0, 45},    // RR
                        {-25, 0, 45},   // RL
                        {-25, 0, 45}};  // FL
                        

float yawRight[4][3] = {{-15, 0, 65},  // FR
                        {-15, 0, 65},  // RR
                         {15, 0, 25},   // RL
                         {15, 0, 25}};  // FL
                         
float yawLeft[4][3] = {{15, 0, 25},    // FR
                       {15, 0, 25},    // RR
                      {-15, 0, 65},   // RL
                      {-15, 0, 65}};  // FL



// ************************ WALK FORWARD SEQUENCE ************************
float forwardWalk_A[4][3] = {{10, 5, 30},     // FR
                             {10, -10, 52},   // RR
                             {10, 5, 30},     // RL
                             {10, -10, 50}};  // FL
                
float forwardWalk_B[4][3] = {{10, 10, 52},    // FR
                             {10, -10, 30},   // RR
                             {10, 10, 50},    // RL
                             {10, -10, 30}};  // FL
                  
float forwardWalk_C[4][3] = {{10, -10, 52},   // FR
                             {10, 5, 30},     // RR
                             {10, -10, 50},   // RL   
                             {10, 5, 30}};    // FL
                 
float forwardWalk_D[4][3] = {{10, -10, 30},   // FR
                             {10, 10, 52},    // RR
                             {10, -10, 30},   // RL
                             {10, 10, 50}};   // FL 


// ************************ WALK BACK SEQUENCE ************************
float backWalk_A[4][3] = {{10, -5, 30},   // FR
                          {10, 10, 50},   // RR
                          {10, -5, 30},   // RL
                          {10, 10, 48}};  // FL
                
float backWalk_B[4][3] = {{10, -10, 50},  // FR
                          {10, 10, 30},   // RR
                          {10, -10, 48},  // RL
                          {10, 10, 30}};  // FL
                  
float backWalk_C[4][3] = {{10, 10, 50},   // FR
                          {10, -5, 30},   // RR
                          {10, 10, 48},   // RL   
                          {10, -5, 30}};  // FL
                 
float backWalk_D[4][3] = {{10, 10, 30},   // FR
                          {10, -10, 50},  // RR
                          {10, 10, 30},   // RL
                          {10, -10, 48}}; // FL    


// ************************ TROT RIGHT SEQUENCE ************************
float rightTrot_A[4][3] = {{-5, 0, 30},  // FR
                           {10, 0, 50},   // RR
                           {5, 0, 30},   // RL
                           {-10, 0, 50}};  // FL
                
float rightTrot_B[4][3] = {{-10, 0, 50},  // FR
                           {10, 0, 30},   // RR
                           {10, 0, 50},   // RL
                           {-10, 0, 30}};  // FL
                  
float rightTrot_C[4][3] = {{10, 0, 50},   // FR
                           {-5, 0, 30},   // RR
                           {-10, 0, 50},   // RL   
                           {5, 0, 30}};  // FL
                 
float rightTrot_D[4][3] = {{10, 0, 30},   // FR
                          {-10, 0, 50},  // RR
                          {-10, 0, 30},   // RL
                           {10, 0, 50}}; // FL    


// ************************ TROT LEFT SEQUENCE ************************
float leftTrot_A[4][3] = {{5, 0, 30},  // FR
                        {-10, 0, 50},   // RR
                         {-5, 0, 30},   // RL
                         {10, 0, 50}};  // FL
                
float leftTrot_B[4][3] = {{10, 0, 50},  // FR
                           {-10, 0, 30},   // RR
                           {-10, 0, 50},   // RL
                           {10, 0, 30}};  // FL
                  
float leftTrot_C[4][3] = {{-10, 0, 50},   // FR
                            {5, 0, 30},   // RR
                           {10, 0, 50},   // RL   
                           {-5, 0, 30}};  // FL
                 
float leftTrot_D[4][3] = {{-10, 0, 30},   // FR
                           {10, 0, 50},  // RR
                           {10, 0, 30},   // RL
                          {-10, 0, 50}}; // FL


// ************************ PIVOT RIGHT SEQUENCE ************************
float rightPivot_A[4][3] = {{-5, 0, 30},  // FR
                            {10, 0, 50},   // RR
                            {-5, 0, 30},   // RL
                            {10, 0, 50}};  // FL
                
float rightPivot_B[4][3] = {{-10, 0, 50},  // FR
                            {10, 0, 30},   // RR
                            {-10, 0, 50},   // RL
                            {10, 0, 30}};  // FL
                   
float rightPivot_C[4][3] = {{10, 0, 50},   // FR
                            {-5, 0, 30},   // RR
                            {10, 0, 50},   // RL   
                            {-5, 0, 30}};  // FL
                 
float rightPivot_D[4][3] = {{10, 0, 30},   // FR
                           {-10, 0, 50},  // RR
                           {10, 0, 30},   // RL
                           {-10, 0, 50}}; // FL

// ************************ PIVOT LEFT SEQUENCE ************************
float leftPivot_A[4][3] = {{5, 0, 30},  // FR
                         {-10, 0, 50},   // RR
                          {5, 0, 30},   // RL
                          {-10, 0, 50}};  // FL
                
float leftPivot_B[4][3] = {{10, 0, 50},  // FR
                          {-10, 0, 30},   // RR
                          {10, 0, 50},   // RL
                           {-10, 0, 30}};  // FL
                  
float leftPivot_C[4][3] = {{-10, 0, 50},   // FR
                             {5, 0, 30},   // RR
                            {-10, 0, 50},   // RL   
                            {5, 0, 30}};  // FL
                 
float leftPivot_D[4][3] = {{-10, 0, 30},   // FR
                            {10, 0, 50},  // RR
                            {-10, 0, 30},   // RL
                           {10, 0, 50}}; // FL
