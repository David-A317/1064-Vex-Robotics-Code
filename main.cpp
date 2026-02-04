#include "vex.h"
using namespace vex;

competition Competition;

brain Brain;
controller Controller;

motor intakeTop(PORT5, ratio6_1, true);
motor intakeBottom(PORT10, ratio6_1, false);

motor rf(PORT20, ratio6_1, false);
motor rm(PORT18, ratio6_1, true);
motor rb(PORT19, ratio6_1, false);
motor lf(PORT11, ratio6_1, true);
motor lm(PORT12, ratio6_1, true);
motor lb(PORT13, ratio6_1, false);

motor_group rightDrive = motor_group(rf, rm, rb);
motor_group leftDrive = motor_group(lf, lm, lb);

digital_out wing(Brain.ThreeWirePort.B);
digital_out intake(Brain.ThreeWirePort.A);
digital_out schlorb(Brain.ThreeWirePort.C);

inertial imu = inertial(PORT4);

//Variables to iterate through a list allowing for different code to be used
int intakeLevel = 1;
int autovar = 0;
const char* autoNames [] = {"no auton", "left", "right", "skills_20", "skills", "Left AWP", "Right AWP"};
const char* intakeNames []= {"null", "middle", "upper"};
const char* intakeNames2 [] = {"null", "intake", "outtake"};

//Drivetrain Variables
double wheelSize = 3.25;
double gearRatio = 0.75;

int controllerDisplay()
{
  Controller.Screen.clearScreen();
  Brain.Screen.clearScreen();
  while(true)
  {
    Controller.Screen.setCursor(1,1);
    Controller.Screen.print("%s            ", autoNames[autovar]);
    Controller.Screen.setCursor(2,1);
    Controller.Screen.print("Heading %.1f   ", imu.heading(degrees));
    Controller.Screen.setCursor(3,1);
    Controller.Screen.print("Intake Type: %s        ", intakeNames2[intakeLevel]);
    Controller.Screen.print("Battery %d%%", Brain.Battery.capacity());

    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("%s            ", autoNames[autovar]);
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("Heading %.1f   ", imu.heading(degrees));
    Brain.Screen.setCursor(3,1);
    //Brain.Screen.print("Intake hopper %s        ", intakeNames2[intakelevel]);
    Brain.Screen.print("Battery %d%%", Brain.Battery.capacity());
    
    
    task::sleep(100);
  }
  return 0;
}

//Utilized in heading to ensure a clean 360 - allows a seamless change from 360 to 0
float restrain(float num, float min, float max)
{
    while(num > max) num -= (max-min);
    while(num < min) num += (max-min);
    return num;
}

//Lateral PID Declarations

const double kp = 0.675;
const double ki = 0.02;
const double kd = 0.03;
const double ckp = 0.05;
const double integralLimit = 12;
bool headingLock = false;
double prevHeading;

//Lateral PID function
int driveFor(float distance, float timeoutSec = -1)
{
    if(headingLock == false){
      prevHeading = imu.heading(degrees);
    }

    rightDrive.resetPosition();
    leftDrive.resetPosition();

    //Defining Variables
    double targetDistance = distance;
    double error = 0;
    double prevError = 0;
    double derivative = 0;
    double integral = 0;
    double settleTime = 0;
    vex::timer timer;
    double lastTime = timer.time(sec);

    while (timeoutSec < 0 || timer.time(sec) < timeoutSec){
      //setting dt
      double currentTime = timer.time(sec);
      double dt = currentTime - lastTime;
      lastTime = currentTime;
      if(dt <= 0) dt = 0.01;

      double avgMotorPos = ((leftDrive.position(degrees) + rightDrive.position(degrees)) / 2.0);
      double driveInches = (wheelSize * M_PI) * ((avgMotorPos * gearRatio) / 360.0);

      error = targetDistance - driveInches;

      if(fabs(error) < 6)
      {
        integral += error * dt;
      }

      //integral Limit
      if(integral > integralLimit) integral = integralLimit;
      if(integral < -integralLimit) integral = -integralLimit;

      derivative = (error - prevError) / dt;
      prevError = error;

      double lateralPower = (error * kp) + (integral * ki) + (derivative * kd);

      double headingError = restrain(prevHeading - imu.heading(degrees), -180, 180);
      double headingPower = headingError * ckp;

      //Lateral Power being edited by heading power
      double leftMotorPower = lateralPower + headingPower;
      double rightMotorPower = lateralPower - headingPower;
      
      //clamp voltage to -12 to 12 volts
      leftMotorPower = fmin(fmax(leftMotorPower, -12), 12);
      rightMotorPower = fmin(fmax(rightMotorPower, -12), 12);
      
      //Motors actually drive here
      leftDrive.spin(fwd, leftMotorPower, volt);
      rightDrive.spin(fwd, rightMotorPower, volt);

      //must be within 1.0 inches
      if(fabs(error) < 1.0)
      {
        settleTime += dt;
      }
      else
      {
        settleTime = 0;
      }
        if(settleTime >= 0.15) 
        break;
      task::sleep(10);
    }
    leftDrive.stop(brake);
    rightDrive.stop(brake);
    return 0;
}

//Turning PID Declarations

const double tkp = 0.6;
const double tki = 0.0005;
const double tkd = 0.04;
const double tIntegralLimit = 10;

int turnTo(float heading, float timeoutSec = -1)
{
    //Defining Variables
    double targetHeading = heading;
    double terror = 0;
    double tprevError = 0;
    double tderivative = 0;
    double tintegral = 0;
    double tsettleTime = 0;
    vex::timer timer;
    double lastTime = timer.time(sec);

    while (timeoutSec < 0 || timer.time(sec) < timeoutSec){
      //setting dt
      double currentTime = timer.time(sec);
      double dt = currentTime - lastTime;
      lastTime = currentTime;
      if(dt <= 0) dt = 0.01;

      terror = restrain(targetHeading - imu.heading(degrees), -180, 180);
      if(fabs(terror) < 6)
      {
        tintegral += terror * dt;
      }
      //integral Limit
      if(tintegral > tIntegralLimit) tintegral = tIntegralLimit;
      if(tintegral < -tIntegralLimit) tintegral = -tIntegralLimit;

      tderivative = (terror - tprevError) / dt;
      tprevError = terror;

      double headingPower = (terror * tkp) + (tintegral * tki) + (tderivative * tkd);
      double leftMotorPower = headingPower;
      double rightMotorPower = headingPower;
      //clamp voltage to -12 to 12 volts
      leftMotorPower = fmin(fmax(leftMotorPower, -12), 12);
      rightMotorPower = fmin(fmax(rightMotorPower, -12), 12);

      leftDrive.spin(fwd, leftMotorPower, volt);
      rightDrive.spin(reverse, rightMotorPower, volt);

      //must be within 0.5 degrees
      if(fabs(terror) < 1.0)
      {
        tsettleTime += dt;
      }
      else
      {
        tsettleTime = 0;
      }
        if(tsettleTime >= 0.15) 
        break;
      
      task::sleep(10);
    }
    leftDrive.stop(brake);
    rightDrive.stop(brake);
    prevHeading = heading;
    headingLock = true;
    return 0;
}

//Utilizes a switch statement to move between different autonomous routes
void autonomous(void) {
   switch(autovar)
    {
      leftDrive.setStopping(brake);
      rightDrive.setStopping(brake);
        case 0: //no auton test
        
        driveFor(48, 5);

        break;
        
        case 1: //Left
        
        

        break;

        case 2: //Right

        

        break;

        case 3: //skills 20 points
        
        

        break;

        case 4: //skills

        

        break;

        case 5: //LeftAWP

        

        break;

        case 6: //RightAWP

        

        break;
    }
}

void usercontrol(void) {
    leftDrive.setStopping(brake);
    rightDrive.setStopping(brake);
    intakeTop.setStopping(coast);
    intakeBottom.setStopping(coast);

    while(Competition.isDriverControl())
    {
        leftDrive.spin(fwd, (Controller.Axis3.position() + Controller.Axis1.position())/8.3333333, volt);
        rightDrive.spin(fwd, (Controller.Axis3.position() - Controller.Axis1.position())/8.3333333, volt);

        if(Controller.ButtonR1.pressing() & (intakeLevel == 1)){
            intakeBottom.spin(fwd, 12, volt);
            intakeTop.spin(fwd, 12, volt);
        }
        else if(Controller.ButtonR2.pressing() & (intakeLevel == 1)){
            intakeBottom.spin(reverse, 12, volt);
            intakeTop.spin(reverse, 12, volt);
        }
        else if(Controller.ButtonR1.pressing() & (intakeLevel == 2)){
            intakeBottom.spin(fwd, 12, volt);
            intakeTop.spin(reverse, 12, volt);
        }
        else if(Controller.ButtonR1.pressing() & (intakeLevel == 2)){
            intakeBottom.spin(reverse, 12, volt);
            intakeTop.spin(fwd, 12, volt);
        }
        else
        {
            intakeBottom.stop();
            intakeTop.stop();
        }

    }
}

//Iterate up in autonomous list
void upPressed()
{
  autovar++;
  if(autovar > 6)
    autovar = 0;
}

//Iterate down in autonomous list
void downPressed()
{
  autovar--;
  if(autovar < 0)
    autovar = 6;
}

void l1Pressed()
{
  if(intakeLevel == 1)
  {
    intakeLevel = 2;
  }
  else
  {
    intakeLevel = 1;
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
  Controller.ButtonUp.pressed(upPressed);
  Controller.ButtonDown.pressed(downPressed);
  Controller.ButtonL1.pressed(l1Pressed);
  Controller.ButtonB.pressed(bPressed);
  Controller.ButtonX.pressed(xPressed);
  vex::task fishy(controllerDisplay);
  while (true) {
    wait(100, msec);
  }
}