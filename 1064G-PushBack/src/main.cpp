#include "vex.h"
using namespace vex;

//Defining Variables
competition Competition;
brain Brain;
controller Controller;

//Intake Motors
motor intakeMotor1 = motor(PORT12, ratio6_1, false);
motor intakeMotor2 = motor(PORT19, ratio6_1, true);

//Drivetrain Motors
motor lf = motor(PORT11, ratio6_1, true);
motor lm = motor(PORT13, ratio6_1, false);
motor lb = motor(PORT14, ratio6_1, true);
motor rf = motor(PORT20, ratio6_1, false);
motor rm = motor(PORT17, ratio6_1, false);
motor rb = motor(PORT18, ratio6_1, true);

motor_group rightDrive = motor_group(rf, rm, rb);
motor_group leftDrive = motor_group(lf, lm, lb);

//Inertial Sensors essential for PID
inertial imu1 = inertial(PORT1);
inertial imu2 = inertial(PORT10);

//Broken limit switch utilized as a button for calibrating sensors
limit lim = limit(Brain.ThreeWirePort.B);

//threewire connections all pneumatic solenoids in this case
digital_out passive = digital_out(Brain.ThreeWirePort.A);
digital_out pto = digital_out(Brain.ThreeWirePort.D);
digital_out schlorb = digital_out(Brain.ThreeWirePort.G);
digital_out wing = digital_out(Brain.ThreeWirePort.C);

//Variables to iterate through a list allowing for different code to be used
int intakestat = 1;
int autovar = 0;
const char* autoNames [] = {"no auton", "left", "right", "skills"};
const char* intakeNames []= {"null", "hopper", "middle", "upper"};

//Drivetrain Variables
double wheelSize = 3.25;
double gearRatio = 0.75;

//Utilized in heading to ensure a clean 360 - allows a seamless change from 360 to 0
float restrain(float num, float min, float max)
{
    while(num > max) num -= (max-min);
    while(num < min) num += (max-min);
    return num;
}

//Heading cannot be averaged by adding and dividing by the amount this function acts as a way to average d
float averageHeading(inertial imu1, inertial imu2){
  //convert to radians
  float a1 = imu1.heading(degrees)*M_PI/180;
  float a2 = imu2.heading(degrees)*M_PI/180;

  float x = cos(a1) + cos(a2);
  float y = sin(a1) + sin(a2);

  float avg = atan2(y, x) * 180.0/M_PI;

  if(avg < 0) avg += 360.0;

  return avg;
}

//PID Start

//Lateral PID Declarations

const double kp = 0.7;
const double ki = 0.01;
const double kd = 1.0;
const double ckp = 0.2;
const double integralLimit = 12;

//Lateral PID function
int driveFor(float distance, float timeoutsec)
{
    rightDrive.resetPosition();
    leftDrive.resetPosition();

    //Defining variables
    float targetDistance = distance;
    float t = 0;
    float error;
    float prevError = 0;
    float derivative;
    float totalError = 0;
    float targetHeading = averageHeading(imu1, imu2);
    float correctError;
    float elapsedTime = 0;
    float dt = 0.01;
    vex::timer timer;

    //PID loop will continue until the necessary value is reached or until the timeout has been reached
    while(elapsedTime < timeoutsec)
    {
        float avgMotorPos = (leftDrive.position(degrees) + rightDrive.position(degrees)) / 2;
        //Calculates how much the drivetrain is moving
        float driveInches = (wheelSize * M_PI) * ((avgMotorPos * gearRatio) / 360);

        //Calculates error * predicts future error
        error = targetDistance - driveInches;
        derivative = (error - prevError) * dt;
        prevError = error;

        //Calculates integral and creates a limit
        totalError = totalError = error / dt;
        if (totalError > integralLimit) totalError = integralLimit;
        if (totalError < -integralLimit) totalError = -integralLimit;

        double lateralMotorPower = (error * kp) + (derivative * kd) + (totalError * ki);
        //Creates small value to correct the heading to the heading set when driving starts
        correctError = restrain(targetHeading - averageHeading(imu1, imu2), -180, 180);
        double correctMotorPower = correctError * ckp;
        //Power motors
        leftDrive.spin(fwd, lateralMotorPower + correctMotorPower, volt);
        rightDrive.spin(fwd, lateralMotorPower - correctMotorPower, volt);
        //Condition to stop the loop must be less than an inch to stop the loop
        if(fabs(error) < 1.0)
        {
            t += 0.01;
        }
        else
        {
            t = 0;
        }
        if(t >= 0.1)
        {
            break;
        }
        elapsedTime = timer.time(sec);
        task::sleep(10);
    }
    leftDrive.stop(hold);
    rightDrive.stop(hold);
    return 0;
}

//Turning PID Declarations

const double tkp = 0.2;
const double tki = 0.01;
const double tkd = 1.5;
const double tIntegralLimit = 10;

int turnTo(float heading, float timeoutsec)
{
    //Define variables
    float targetHeading = heading;
    float tError;
    float tPrevError;
    float tDerivative;
    float tTotalError;
    float t = 0;
    float elapsedTime = 0;
    float dt = 0.01;
    vex::timer timer;
    
    //Start Turning PID Loop
    while(elapsedTime < timeoutsec)
    {   
        //Calculate error & predict error
        tError = restrain(targetHeading - averageHeading(imu1, imu2), -180, 180);
        tDerivative = (tError - tPrevError);
        tPrevError = tError;

        //Calculate integral & set integral limit
        tTotalError = tTotalError + tError * dt;
        if (tTotalError > tIntegralLimit) tTotalError = tIntegralLimit;
        if (tTotalError < -tIntegralLimit) tTotalError = -tIntegralLimit;

        double tMotorPower = (tError * tkp) + (tDerivative * tkd) + (tTotalError * tki);

        rightDrive.spin(reverse, tMotorPower, volt);
        leftDrive.spin(fwd, tMotorPower, volt);
        //To stop loop PID must have less than 0.5 degrees of error
        if(fabs(tError) < 0.5)
        {
            t += 0.01;
        }
        else
        {
            t = 0;
        }
        if(t >= 0.1)
        {
            break;
        }
        elapsedTime = timer.time(sec);
        task::sleep(10);

    }
    rightDrive.stop(hold);
    leftDrive.stop(hold);
    return 0;
}

//PID End

//Tasks
// Constantly runs displaying values on the controller screen
int controllerDisplay()
{
  Controller.Screen.clearScreen();
  while(true)
  {
    Controller.Screen.setCursor(1,1);
    Controller.Screen.print("%s            ", autoNames[autovar]);
    Controller.Screen.setCursor(2,1);
    Controller.Screen.print("Battery %d%%", Brain.Battery.capacity());
    //Controller.Screen.print("Heading %.1f", averageHeading(imu1, imu2));
    Controller.Screen.setCursor(3,1);
    Controller.Screen.print("Intake Status %s        ", intakeNames[intakestat]);
    task::sleep(500);
  }
  return 0;
}

//Autonomous functions
//Different conditions to utilize various mechanisms on the robot
void bottom() {
    pto.set(true);
    passive.set(false);
    intakeMotor2.spin(reverse, 12, volt);
}

void hopper() {
    pto.set(true);
    passive.set(false);
    intakeMotor2.spin(fwd, 12, volt);
}

void upper() {
    pto.set(true);
    passive.set(true);
    intakeMotor1.spin(fwd, 11, volt);
    intakeMotor2.spin(fwd, 11, volt);
}

void middle() {
    pto.set(false);
    pto.set(false);
    intakeMotor1.spin(fwd, 12, volt);
    intakeMotor2.spin(fwd, 12, volt);
}

void stop() {
    intakeMotor1.stop();
    intakeMotor2.stop();
}

//Autonomous + pre auton
//Ambiguous function anything can be used
void pre_auton(void) {
  Brain.Screen.print(1);
}

//Utilizes a switch statement to move between different autonomous routes
void autonomous(void) {
   switch(autovar)
    {
        case 0: //no auton test
        
        /*driveFor(36, 2);
        turnTo(180, 1);
        schlorb.set(true);
        driveFor(24, 1);
        hopper();
        wait(2, sec);
        driveFor(-12, 1);
        turnTo(0, 1);
        driveFor(24, 1);
        upper();
        wait(2, sec);
        driveFor(-24, 1);
        turnTo(315, 1);
        hopper();
        driveFor(40, 1);
        bottom();*/
        hopper();

        break;
        
        case 1: //Left

        driveFor(8, 1);
        turnTo(270, 1);
        driveFor(33, 1);
        turnTo(180, 1);
        schlorb.set(true);
        wait(0.5, sec);
        driveFor(22, 1);
        hopper();
        wait(3, sec);
        driveFor(-12, 1);
        schlorb.set(false);
        stop();
        wait(0.5, sec);
        turnTo(0, 1);
        driveFor(8, 1);
        turnTo(0, 1);
        upper();
        wait(3, sec);
        stop();

        break;

        case 2: //Right

        driveFor(8, 1);
        turnTo(90, 1);
        driveFor(33, 1);
        turnTo(180, 1);
        schlorb.set(true);
        wait(0.5, sec);
        driveFor(24, 1);
        hopper();
        wait(3, sec);
        driveFor(-12, 1);
        schlorb.set(false);
        stop();
        wait(0.5, sec);
        turnTo(0, 1);
        driveFor(8, 1);
        turnTo(0, 1);
        upper();
        wait(3, sec);
        stop();

        break;

        case 3: //skills 20 points

        driveFor(80, 5);
        driveFor(-40, 5);

        break;
    }
}

//Driver Control

void usercontrol(void) {
    // Set stopping mode for drivetrain motors
    leftDrive.setStopping(brake);
    rightDrive.setStopping(brake);
    intakeMotor1.setStopping(coast);
    intakeMotor2.setStopping(coast);

    // Driver control loop
    while (Competition.isDriverControl()) {

        // Drivetrain control
        leftDrive.spin(fwd, Controller.Axis3.position()/8.3333333, volt);
        rightDrive.spin(fwd, Controller.Axis2.position()/8.3333333, volt);

        // Intake control switch utilized to switch between mechanisms
        switch(intakestat)
        {
            case 4: 
            
            //nuked


            break;
            case 3: //upper goal
            
            pto.set(true);
            passive.set(true);

            if(Controller.ButtonR1.pressing()){
                intakeMotor1.spin(fwd, 12, volt);
                intakeMotor2.spin(fwd, 12, volt);
            }
            else{
                intakeMotor1.stop(hold);
                intakeMotor2.stop(hold);
            }

            break;
            case 1: // hopper

            pto.set(true);
            passive.set(false);

            if(Controller.ButtonR1.pressing()){
                intakeMotor2.spin(fwd, 12, volt);
            }
            else if(Controller.ButtonR2.pressing()){
              intakeMotor2.spin(reverse, 12, volt);
            }
            else{
                intakeMotor2.stop(hold);
                intakeMotor1.stop(hold);
            }

            break;
            case 2: //middle goal

            pto.set(false);
            passive.set(false);

            if(Controller.ButtonR1.pressing()){
                intakeMotor1.spin(fwd, 12, volt);
                intakeMotor2.spin(fwd, 12, volt);
            }
            else{
                intakeMotor1.stop(hold);
                intakeMotor2.stop(hold);
            }

            break;
        }
        

        wait(20, msec);
    }
}

//OnEvent Functions
//When the limit switch is pressed calibrate the inertials
void limitSwitchPressed()
{
  imu1.calibrate();
  imu2.calibrate();
  Controller.rumble("-");
  while(imu1.isCalibrating()){wait(100, msec);}
  Controller.rumble("...");
}

//Iterate up in autonomous list
void upPressed()
{
  autovar++;
  if(autovar > 3)
    autovar = 0;
}

//Iterate down in autonomous list
void downPressed()
{
  autovar--;
  if(autovar < 0)
    autovar = 3;
}

//Change intake status haptics utilized through rumbles
void l2Pressed()
{
  intakestat++;
  if (intakestat > 3)
  {
    intakestat = 1;
  }
  if(intakestat == 1){
      Controller.rumble(".");
    }
    else if(intakestat == 2){
      Controller.rumble("..");
    }
    else{
      Controller.rumble("...");
    }
}

void l1Pressed()
{
  intakestat--;
  if(intakestat < 1)
    intakestat = 3;
    if(intakestat == 1){
      Controller.rumble(".");
    }
    else if(intakestat == 2){
      Controller.rumble("..");
    }
    else{
      Controller.rumble("...");
    }
}

//Activates solenoid for the schlorb mech
void bPressed() {
  if(schlorb.value())
  {
    schlorb.set(false);
  }else
  {
    schlorb.set(true);
  }
}

//Activates solenoid for wing mech
void xPressed() {
  if(wing.value())
  {
    wing.set(false);
  }else
  {
    wing.set(true);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  lim.pressed(limitSwitchPressed);
  Controller.ButtonUp.pressed(upPressed);
  Controller.ButtonDown.pressed(downPressed);
  Controller.ButtonL1.pressed(l1Pressed);
  Controller.ButtonL2.pressed(l2Pressed);
  Controller.ButtonB.pressed(bPressed);
  Controller.ButtonX.pressed(xPressed);
  vex::task fishy(controllerDisplay);
  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
