/*
  PlutoDOG_OS   |  Version: 1.0.0
  by plutoLABS  |           07/05/2023
 --------------------------------------
 FIRMWARE NOTES:
  - Code to be used on robot body assembly: [23001-000-000]
  - Co-Ordinate System: RHS --> Dog Front = Positive X-axis
  - ESP32-WROOM DEVKIT: 3.3V System
  - PCA9685(I2C): Outputs = 12-bit --> 4096 servo steps
     : 2.5% of 20ms = 0.5ms      : 12.5% of 20 ms = 2.5ms
     : 2.5% of 4096 = 102 steps  : 12.5% of 4096 = 512 steps
  - Voltage Sensor(0V-16V): Analog output --> 32
     : Output = 12-bit          : ***
  - D36V50F6(6V-SD): ENABLE pin --> 18
    : LOW = ENABLED              : HIGH = DISABLE
    : DISABLE when battery reaches 85% of BATT_MAX
*/

#include <Ps3Controller.h>
#include "EEPROM.h"
#include "PCA9685.h"
#include "plutoDOG_OS-v1.0.0.h"

// Preintialise timer variable [ms]
unsigned long previousMillis = 0; 

void setup() 
{
  // Run PlutoDOG initialisation 
  plutoDOGInit();

  // Run PS3 controller initialisation
  roboticControllerInit();

  // Run PCA9685 (limb) servo-controller initialisation
  limbControllerInit();

  // Limb servo angles zeroise (initialise lying-down)
  limbInit();
  
  delay(1000); // Upload Safety Delay...
}

void loop() 
{
  if(!Ps3.isConnected())
  {
    Serial.println("  *** ERROR: Robotic Controller System DISCONNECTED! ***");
    delay(1000);
    return;
  }

  //heartbeat(); // Was causing delays in walking gait
  walkCommand();
}

void plutoDOGInit()
{
  // PlutoDOG_OS setup commands
  Serial.begin(115200);
  Wire.begin();

  // Setup digital / analog pins
  pinMode(VOLT_SENSOR_PIN, INPUT);
  pinMode(VRS_PIN, OUTPUT);
  pinMode(LMS_PIN, OUTPUT);

  digitalWrite(VRS_PIN, disableMovement); // Start with movement DISABLE
  //digitalWrite(LMS_PIN, disableMovement); // Disable limb movement controller

  // Initialise EEPROM: storage for battery management system (BMS)
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println(" *** ERROR: Failed to initialise EEPROM ***");
    while(1){ Serial.println(" Please reset device! ");Serial.println();delay(750);}
  }
  else
  {
    checkEEPROM(); // Check stored EEPROM data
  }

  // Initialise Memory Managment System: (MMS) for EEPROM tracking
  if(!EEPROM_isInit)
  {
    EEPROM_COUNTER = 0; // Not intialised --> Zeroise (int) EEPROM
    EEPROM.writeInt(COUNTER_ADDR, EEPROM_COUNTER);
    EEPROM.commit();
    updateMMS();
  }
  else
  {
    EEPROM_COUNTER = EEPROM.readInt(COUNTER_ADDR); // EEPROM previously intialised
  }

  // Initialise Battery Management System: (BMS) with stored EEPROM voltage value
  if(!EEPROM_isInit || isnan(EEPROM.readFloat(BATT_MAX_ADDR)))
  {
    BATT_MAX = 0.00; // Not intialised --> Zeroise (float) EEPROM
    EEPROM.writeInt(BATT_MAX_ADDR, BATT_MAX);
    EEPROM.commit();
    updateMMS();
  }
  else
  {
    BATT_MAX = EEPROM.readFloat(BATT_MAX_ADDR); // EEPROM previously intialised
  }
}

void roboticControllerInit()
{
  // Initialise Robotic Controller System (RCS)
  Ps3.attach(controllerEvents);
  Ps3.attachOnConnect(controllerConnect);
  Ps3.begin("50:4C:55:54:4F:22"); // Define controller MAC address [hex --> P:L:U:T:O:22]
}

void controllerConnect()
{
  // On controller connect set flag
  Serial.println("   >>> Controller Connected Successfully! <<<");
  controller_isInit = true;
}

void limbControllerInit()
{
  // PCA9685 PWM Controller setup commands
  if(!limbController_isInit)
  {
    limbController.resetDevices(); // Reset all controller devices on i2c line
    limbController.init(); // Initialises controller module
    limbController.setPWMFrequency(50); // Set PWM freq to 50 Hz

    limbController_isInit = true;// Limb controller initialised!
  }
  
}

void limbInit()
{
  if (!limbs_isInit)
  {
    // Initialise SHOULDER LIMB (x-axis) servos (tuned to lay on init)
    limbController.setChannelPWM(shoulderX_FR, shoulderX_FR_servo.pwmForAngle(0));
    limbController.setChannelPWM(shoulderX_FL, shoulderX_FL_servo.pwmForAngle(0));
    limbController.setChannelPWM(shoulderX_RL, shoulderX_RL_servo.pwmForAngle(0));
    limbController.setChannelPWM(shoulderX_RR, shoulderX_RR_servo.pwmForAngle(0));

    // Saftey Delay
    delay(300);

    // Initialise SHOULDER LIMB (Y-axis) servos (tuned to lay on init)
    limbController.setChannelPWM(shoulderY_FR, shoulderY_FR_servo.pwmForAngle(0));
    limbController.setChannelPWM(shoulderY_FL, shoulderY_FL_servo.pwmForAngle(0));
    limbController.setChannelPWM(shoulderY_RL, shoulderY_RL_servo.pwmForAngle(0));
    limbController.setChannelPWM(shoulderY_RR, shoulderY_RR_servo.pwmForAngle(0));

    // Saftey Delay
    delay(300);

    // Initialise ELBOW LIMB servos (tuned to lay on init)
    limbController.setChannelPWM(elbowY_RR, elbowY_RR_servo.pwmForAngle(0));
    limbController.setChannelPWM(elbowY_RL, elbowY_RL_servo.pwmForAngle(0));
    limbController.setChannelPWM(elbowY_FR, elbowY_FR_servo.pwmForAngle(0));
    limbController.setChannelPWM(elbowY_FL, elbowY_FL_servo.pwmForAngle(0));

    // Saftey Delay
    delay(300);

    limbs_isInit = true; // Limb motors initialised!
  }
}

void checkEEPROM()
{
  int initCounter = 0;
    for (int i = 0; i < EEPROM_SIZE; i++)
    {
      if(byte(EEPROM.read(i)) == 255) 
      {
        initCounter++;
      }
    }
    if(initCounter == 10) EEPROM_isInit = false;
    else EEPROM_isInit = true;
}

void updateMMS()
{
  int counterValue = EEPROM.readInt(COUNTER_ADDR);
  counterValue++;
  EEPROM.writeInt(COUNTER_ADDR, counterValue);
  EEPROM.commit();
}

void readBMS()
{
    // Float for ADC voltage & Input voltage
    float adcVoltage = 0.0;
    
    // Floats for resistor values in divider (in ohms)
    float R1 = 30000.0;
    float R2 = 7500.0; 
    
    // Float for reference voltage = ESP32 --> 3.3V
    float refVoltage = 3.3;
    
    // Integer for ADC value
    int adcValue = 0;
    
    // Read the analog input from voltage sensor
    adcValue = analogRead(VOLT_SENSOR_PIN);
    
    // Determine voltage at ADC input = ESP32 --> 12bit
    adcVoltage  = (adcValue * refVoltage) / 4096.0; 
    
    // Calculate current voltage at divider input
    BATT_CURRENT = (adcVoltage / (R2/(R1+R2)));
}

void updateBMS()
{
  // Update EEPROM if greater voltage has been read
  if(BATT_CURRENT > BATT_MAX)
  {
    BATT_MAX = BATT_CURRENT;
    EEPROM.writeFloat(BATT_MAX_ADDR, BATT_MAX);
    EEPROM.commit();
    updateMMS();
  }
  
  if(BATT_CURRENT <= BATT_CUTOFF * BATT_MAX || BATT_CURRENT < BATT_STOP)
  {
    disableMovement = true;
    //digitalWrite(VRS_PIN, disableMovement); // DISABLE 6V module
    //digitalWrite(LMS_PIN, disableMovement); // DISABLE limb movement controller

    // Hardset battery percentage as <~0%
    BATT_PERCENTAGE = 0.00;
  }
  else
  {
    disableMovement = false;
    //digitalWrite(VRS_PIN, disableMovement); // ENABLE 6V module
    //digitalWrite(LMS_PIN, disableMovement); // ENABLE limb movement controller

    // Calculate battery percentage where zero-level is the cutoff percentage of maximum battery
    BATT_PERCENTAGE = ((BATT_CURRENT - (BATT_CUTOFF * BATT_MAX)) / (BATT_MAX - (BATT_CUTOFF * BATT_MAX))) * 100; // APPROX.
  }
}

// Controller interrupt actions / events
void controllerEvents()
{  
  // Define controller events on button interrupts
  //------------------------
  // Button START Command
  //------------------------
  if(Ps3.event.button_down.start && !move_isInit)
  {
    move_isInit = true;
    isStanding = true;
    moveLimbs(standAngle);
    Serial.println("CONTROLLER COMMAND: STARTING STAND");
  }

  //------------------------
  // Button CROSS Command
  //------------------------
  if(Ps3.event.button_down.cross && move_isInit)
  {
    isStanding = false;
    moveLimbs(sitAngle);
    Serial.println("CONTROLLER COMMAND: SIT");
  }

  //------------------------
  // Button TRIANGE Command
  //------------------------
  if(Ps3.event.button_down.triangle && move_isInit)
  {
    isStanding = true;
    moveLimbs(standAngle);
    Serial.println("CONTROLLER COMMAND: STAND");
  }

  //------------------------
  // Button CIRCLE Command
  //------------------------
  if(Ps3.event.button_down.circle && move_isInit)
  {
    isStanding = false;
    moveLimbs(layAngle);
    Serial.println("CONTROLLER COMMAND: LIE DOWN");
  }

  //------------------------
  // Button SQUARE Command
  //------------------------
  if(Ps3.event.button_down.square && move_isInit)
  {
    isStanding = false;
    shakeSequence();
    Serial.println("CONTROLLER COMMAND: SHAKE");
  }

  //------------------------
  // Left SHOULDER Command
  //------------------------
  if(Ps3.event.button_down.l1 && move_isInit)
  {
    // Define servo max movement speed
    MAX_SPEED = 12.0;
    
    // Define walking commands
    isWalking = true;
    walkForward = false;
    walkBack = false;
    pivotRight = false;
    pivotLeft = false;
    trotRight = false;
    trotLeft = true;
    walkHalt = false;     
  }

  //------------------------
  // Right SHOULDER Command
  //------------------------
  if(Ps3.event.button_down.r1 && move_isInit)
  {    
    // Define servo max movement speed
    MAX_SPEED = 12.0;
    
    // Define walking commands
    isWalking = true;
    walkForward = false;
    walkBack = false;
    pivotRight = false;
    pivotLeft = false;
    trotRight = true;
    trotLeft = false;
    walkHalt = false;    
  }

  //------------------------
  // Up SHOULDER Command(s)
  //------------------------
  if(Ps3.event.button_up.l1 || Ps3.event.button_up.r1 && move_isInit)
  {
    // Reset servo max movement speed
    MAX_SPEED = 10.0;
    
    // Define walking commands
    isWalking = false;
    walkForward = false;
    walkBack = false;
    pivotRight = false;
    pivotLeft = false;
    trotRight = false;
    trotLeft = false;
    walkHalt = true;     
  }
  

  //------------------------
  // Left STICK Command
  //------------------------
  if(Ps3.event.button_down.l3 && move_isInit)
  {
    leanMode = !leanMode;
    isWalking = false;
    walkForward = false;
    walkBack = false;
    walkHalt = true;
  }

  //------------------------
  // Left TRIGGER Command
  //------------------------
  if(Ps3.event.button_down.l2 && move_isInit)
  {
    moveLimbs(yawLeft);
  }

  //------------------------
  // Right TRIGGER Command
  //------------------------
  if(Ps3.event.button_down.r2 && move_isInit)
  {
    moveLimbs(yawRight);
  }

  if(Ps3.event.button_up.l2 || Ps3.event.button_up.r2 && move_isInit)
  {
    moveLimbs(standAngle);
  }


  //------------------------
  // Analog Right-Stick Commands
  //------------------------
  if(move_isInit && isStanding && !leanMode)
  {
    if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 )
    {
      if(Ps3.data.analog.stick.ry <= -stickSenseMIN && move_isInit && isStanding) // Forward Stick
      {
        // Define walking commands
        isWalking = true;
        walkForward = true;
        walkBack = false;
        pivotRight = false;
        pivotLeft = false;
        trotRight = false;
        trotLeft = false;
        walkHalt = false;

        // Define dynamic max speed based on stick input
        MAX_SPEED = ((abs(Ps3.data.analog.stick.ry) - 5) * (5 - 10) / (120 - 5)) + 10;
      }
      if(Ps3.data.analog.stick.ry >= stickSenseMIN && move_isInit && isStanding) // Back Stick
      {
        // Define walking commands
        isWalking = true;
        walkForward = false;
        walkBack = true;
        pivotRight = false;
        pivotLeft = false;
        trotRight = false;
        trotLeft = false;
        walkHalt = false;

        // Define servo max movement speed
        MAX_SPEED = 7.0;
      }
      if(Ps3.data.analog.stick.rx <= -stickSenseMIN && move_isInit && isStanding) // Left Stick
      {
        // Define walking commands
        isWalking = true;
        walkForward = false;
        walkBack = false;
        pivotRight = false;
        pivotLeft = true;
        trotRight = false;
        trotLeft = false;
        walkHalt = false;

        // Define servo max movement speed
        MAX_SPEED = 10.0;
      }
      if(Ps3.data.analog.stick.rx >= stickSenseMIN && move_isInit && isStanding) // Right Stick
      {
        // Define walking commands
        isWalking = true;
        walkForward = false;
        walkBack = false;
        pivotRight = true;
        pivotLeft = false;
        trotRight = false;
        trotLeft = false;
        walkHalt = false;

        // Define servo max movement speed
        MAX_SPEED = 10.0;
      }
      // Auto recentre back to standing once stick released
      if(Ps3.data.analog.stick.rx <= stickSenseMIN  && 
         Ps3.data.analog.stick.rx >= -stickSenseMIN && // Deadzone [-stickSenseMIN<= lx/ly <= stickSenseMIN]
         Ps3.data.analog.stick.ry <= stickSenseMIN  && 
         Ps3.data.analog.stick.ry >= -stickSenseMIN && move_isInit && isWalking) // Centre Stick
      {
        // Define walking commands
        isWalking = false;
        walkForward = false;
        walkBack = false;
        pivotRight = false;
        pivotLeft = false;
        trotRight = false;
        trotLeft = false;
        walkHalt = true;
        
        // Reset servo max movement speed
        MAX_SPEED = 10.0;
      }
    }
  }
  
  //------------------------
  // Analog Left-Stick Commands [Infinite Zones]
  //------------------------

  if(move_isInit && isStanding && leanMode)
  {
    if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 )
    {      
      // Capture joystick position data
      float lx = Ps3.data.analog.stick.lx;
      float ly = Ps3.data.analog.stick.ly;
  
      // Functions to map joystick position to correct values for leaning
      int mappedShoulder_x = (( (lx + 128) / 256) * 50) - 25;         // f(x): [-128,128] --> [-25,25]
      int mappedShoulder_y = -1 * (( (ly + 128) / 256) * 30) + 15;    // f(x): [-128,128] --> [15,-15]
      int mappedShoulder_e1 = -1 * (( (ly + 128) / 256) * 40) + 65;   // f(x): [-128,128] --> [65,25]
      int mappedShoulder_e2 = (( (ly + 128) / 256) * 40) + 25;        // f(x): [-128,128] --> [25,65]
  
      // Creates angle array for servo movement
      float setAngle[4][3] = {{-mappedShoulder_x, mappedShoulder_y, mappedShoulder_e2},
                              {-mappedShoulder_x, mappedShoulder_y, mappedShoulder_e1},
                              {mappedShoulder_x, mappedShoulder_y, mappedShoulder_e1},
                              {mappedShoulder_x, mappedShoulder_y, mappedShoulder_e2}};
      // Move limb servos
      moveLimbs(setAngle);
    }
  }
}

void moveLimbs(float targetAngle[4][3])
{
  bool targetNotReached;
  static unsigned long servo_time;
  do // Move until reach target limb angle
  {
    if ((millis() - servo_time) >= MAX_SPEED) // Non-blocking delay emulate servo speed
    {
      servo_time = millis(); // Save time reference for next update

      // Update ALL servo positions
      for (int leg = 0 ; leg < 4 ; leg++) // Iterate through each leg limb
      {
        for (int joint = 0 ; joint < 3 ; joint++) // Iterate through each joint in limb
        {
          if (currentAngle[leg][joint] < targetAngle[leg][joint])
          {
            moveServo(leg, joint, currentAngle[leg][joint] + 1);
            currentAngle[leg][joint] = currentAngle[leg][joint] + 1;
          }
          else if (currentAngle[leg][joint] > targetAngle[leg][joint])
          {
            moveServo(leg, joint, currentAngle[leg][joint] - 1);
            currentAngle[leg][joint] = currentAngle[leg][joint] - 1;
          }
        }
      }
    }
    targetNotReached = memcmp((const void *)currentAngle, (const void *)targetAngle, sizeof(currentAngle)); // Compares the num memory bytes of two arrays [1 = unequal | 0 = equal]
  } while (targetNotReached);
}

void shakeSequence()
{
  int shakeSpeed = 150;
  int handshakes = 5;
  
  moveLimbs(sitAngle);
  delay(600);
  for (int i = 0; i < handshakes; i++)
  {
  moveLimbs(shakeAngle_UP);
  delay(shakeSpeed);
  moveLimbs(shakeAngle_DOWN);
  delay(shakeSpeed);
  }
  moveLimbs(shakeAngle_SIT);
  delay(100);
  moveLimbs(sitAngle);
}

void walkCommand()
{
  // Enable walking commands only when walking
  if(isWalking)
  {
    if(walkForward) forwardWalk();           // Forward walk sequence
    if(walkBack)    backWalk();              // Backwards walk sequence
    if(pivotRight)  rightPivot();            // Right pivot sequence
    if(pivotLeft)   leftPivot();             // Left pivot sequence
    if(trotRight)   rightTrot();            // Right pivot sequence
    if(trotLeft)    leftTrot();             // Left pivot sequence
    if(walkHalt)    moveLimbs(standAngle);   // Halt walk stand
  }  
}

void forwardWalk()
{
  moveLimbs(forwardWalk_A); // UP    - DOWN
  moveLimbs(forwardWalk_B); // MOVE  - PULL
  moveLimbs(forwardWalk_C); // DOWN  - UP
  moveLimbs(forwardWalk_D); // PULL  - MOVE
}

void backWalk()
{
  moveLimbs(backWalk_A); // UP    - DOWN
  moveLimbs(backWalk_B); // MOVE  - PULL
  moveLimbs(backWalk_C); // DOWN  - UP
  moveLimbs(backWalk_D); // PULL  - MOVE
}

void rightTrot()
{
  moveLimbs(rightTrot_A); // UP    - DOWN
  moveLimbs(rightTrot_B); // MOVE  - PULL
  moveLimbs(rightTrot_C); // DOWN  - UP
  moveLimbs(rightTrot_D); // PULL  - MOVE
}

void leftTrot()
{
  moveLimbs(leftTrot_A); // UP    - DOWN
  moveLimbs(leftTrot_B); // MOVE  - PULL
  moveLimbs(leftTrot_C); // DOWN  - UP
  moveLimbs(leftTrot_D); // PULL  - MOVE
}

void rightPivot()
{
  moveLimbs(rightPivot_A); // UP    - DOWN
  moveLimbs(rightPivot_B); // MOVE  - PULL
  moveLimbs(rightPivot_C); // DOWN  - UP
  moveLimbs(rightPivot_D); // PULL  - MOVE
}

void leftPivot()
{
  moveLimbs(leftPivot_A);  // UP    - DOWN
  moveLimbs(leftPivot_B);  // MOVE  - PULL
  moveLimbs(leftPivot_C);  // DOWN  - UP
  moveLimbs(leftPivot_D);  // PULL  - MOVE
}

void moveServo(int leg, int joint, int pos)
{
  switch (leg)
  {
    case 0: // Front Right Leg
      switch (joint)
      {
        case 0:
          limbController.setChannelPWM(shoulderX_FR, shoulderX_FR_servo.pwmForAngle(-pos));
          break;
        case 1:
          limbController.setChannelPWM(shoulderY_FR, shoulderY_FR_servo.pwmForAngle(pos));
          break;
        case 2:
          limbController.setChannelPWM(elbowY_FR, elbowY_FR_servo.pwmForAngle(-pos));
          break;
      }
      break;
    case 1: // Rear Right Leg
      switch (joint)
      {
        case 0:
          limbController.setChannelPWM(shoulderX_RR, shoulderX_RR_servo.pwmForAngle(pos));
          break;
        case 1:
          limbController.setChannelPWM(shoulderY_RR, shoulderY_RR_servo.pwmForAngle(pos));
          break;
        case 2:
          limbController.setChannelPWM(elbowY_RR, elbowY_RR_servo.pwmForAngle(-pos));
          break;
      }
      break;
    case 2: // Rear Left Leg
      switch (joint)
      {
        case 0:
          limbController.setChannelPWM(shoulderX_RL, shoulderX_RL_servo.pwmForAngle(-pos));
          break;
        case 1:
          limbController.setChannelPWM(shoulderY_RL, shoulderY_RL_servo.pwmForAngle(-pos));
          break;
        case 2:
          limbController.setChannelPWM(elbowY_RL, elbowY_RL_servo.pwmForAngle(pos));
          break;
      }
      break;
    case 3: // Front Right Leg
      switch (joint)
      {
        case 0:
          limbController.setChannelPWM(shoulderX_FL, shoulderX_FL_servo.pwmForAngle(pos));
          break;
        case 1:
          limbController.setChannelPWM(shoulderY_FL, shoulderY_FL_servo.pwmForAngle(-pos));
          break;
        case 2:
          limbController.setChannelPWM(elbowY_FL, elbowY_FL_servo.pwmForAngle(pos));
          break;
      }
      break;
  }
}

void heartbeat()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= heartbeatInterval) 
  {
    // Reset previous timer settings
    previousMillis = currentMillis;

    // Functions that run on heartbeat 
    readBMS();  
    updateBMS();

    // Diagnostics serial output
    for(int i = 0; i < 10; i++){Serial.println();}
    Serial.println("PlutoOS_Version: 1.0.0. -- Data Heartbeat");
    Serial.println("*****************************************");
    
    Serial.println(" Memory Management System [MMS]:");
    Serial.println("-----------------------------------------");
    Serial.print("   MMS Initialised            :  ");if(EEPROM_isInit)Serial.println("✓");else Serial.println("✖");
    Serial.print("   EEPROM Counter             :  ");Serial.println(EEPROM.readInt(COUNTER_ADDR));
    Serial.print("   Current UpTime             :  ");Serial.print(previousMillis/1000);Serial.println(" [s]");
    Serial.println("-----------------------------------------");
    
    Serial.println(" Battery Management System [BMS]:");
    Serial.println("-----------------------------------------");
    Serial.print("   BMS Initialised            :  ");if(EEPROM_isInit)Serial.println("✓");else Serial.println("✖");
    Serial.print("   Current Voltage            :  ");Serial.print(BATT_CURRENT);Serial.println(" [V]");
    Serial.print("   MAX Voltage                :  ");Serial.print(BATT_MAX);Serial.println(" [V]");
    Serial.print("   Battery Percentage         :  ");Serial.print(BATT_PERCENTAGE);Serial.println("%");
    Serial.println("-----------------------------------------");
    
    Serial.println(" Limb Management System [LMS]:");
    Serial.println("-----------------------------------------");
    Serial.print("   Controller Initialised     :  ");if(limbController_isInit)Serial.println("✓");else Serial.println("✖");
    Serial.print("   Motors Initialised         :  ");if(limbs_isInit)Serial.println("✓");else Serial.println("✖");
    Serial.print("   Motor / Controller Enabled :  ");if(disableMovement)Serial.println("✖");else Serial.println("✓");
    Serial.println("-----------------------------------------");

    Serial.println(" Voltage Regulator System [VRS]:");
    Serial.println("-----------------------------------------");
    Serial.print("   Voltage Regulator Enabled  :  ");if(disableMovement)Serial.println("✖");else Serial.println("✓");
    Serial.println("-----------------------------------------");

    Serial.println(" Robotic Controller System [RCS]:");
    Serial.println("-----------------------------------------");
    Serial.print("   Controller Initialised     :  ");if(controller_isInit)Serial.println("✓");else Serial.println("✖");
    controllerBattery = Ps3.data.status.battery;
    Serial.print("   Controller Battery STATUS  :  ");
    if( controllerBattery == ps3_status_battery_charging )      Serial.println("CHARGING");
    else if( controllerBattery == ps3_status_battery_full )     Serial.println("FULL");
    else if( controllerBattery == ps3_status_battery_high )     Serial.println("HIGH");
    else if( controllerBattery == ps3_status_battery_low)       Serial.println("LOW      [Charge Controller!]");
    else if( controllerBattery == ps3_status_battery_dying )    Serial.println("DYING    [Charge Controller!]");
    else if( controllerBattery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN [Charge Controller!]");
    else Serial.println("ERROR!");
  }
}
