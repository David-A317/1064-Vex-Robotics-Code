#include "vex.h"
using namespace vex;

competition Competition;

// Definition of Brain and Controller
brain Brain;
controller Controller(primary);

// define your global instances of motors and other devices here
motor lf = motor(PORT4, ratio6_1, false);
motor lm = motor(PORT5, ratio6_1, true);
motor lb = motor(PORT6, ratio6_1, true);
motor_group leftDrive = motor_group(lf, lm, lb);
motor rf = motor(PORT11, ratio6_1, false);
motor rm = motor(PORT2, ratio6_1, false);
motor rb = motor(PORT3, ratio6_1, true);
motor_group rightDrive = motor_group(rf, rm, rb);
motor convey = motor(PORT8, ratio6_1, false);
motor intake = motor(PORT7, ratio6_1, true);

//Misc Definitions
digital_out solenoid = digital_out(Brain.ThreeWirePort.A);
inertial imu = inertial(PORT12);
limit lim = limit(Brain.ThreeWirePort.B);

//threads
task controllerTask;
task intakeTask;

//Variables
int autovar = 0;
int intakeStatus = 0;
const char* autoNames [] = {"no auton", "red+", "red-", "blue+", "blue-", "skills"};

//PID

float restrain(float num, float min, float max)
{
  while(num > max) num-= (num-min);
  while(num < min) num+= (num-min);
  return num;
}

const double kp = 0.8;
const double ki = 0.0;
const double kd = 0.5;
const double ckp = 0.05;

int driveFor(float distance, float timeoutsec)
{
  leftDrive.resetPosition();
  rightDrive.resetPosition();
  float targetDistance = distance;
  float t = 0;
  float error;
  float prevError = 0;
  float derivative;
  float totalError = 0;
  float targetHeading = imu.heading(degrees);
  float correctError;
  float elapsedTime = 0;

  vex::timer timer;

 while(elapsedTime < timeoutsec)
  {
    float leftMotorPos = leftDrive.position(degrees);
    float rightMotorPos = rightDrive.position(degrees);
    float avgMotorPos = (leftMotorPos + rightMotorPos) / 2.0;
    float driveInches = (2.75*M_PI) * ((avgMotorPos * (4.0 / 5.0)) / 360);

    error = targetDistance - driveInches;
    derivative = error - prevError;
    prevError = error;
    totalError += error;
    double lateralMotorPower = (error * kp) + (derivative * kd) + (totalError * ki);
    correctError = restrain(targetHeading - imu.rotation(degrees), -180, 180);
    if (correctError > 180) correctError -= 360;
    if (correctError < -180) correctError += 360;
    double correctMotorPower = correctError * ckp;

    //printf("correctError: %.2f\n", imu.rotation(degrees)); - FOR DEBUGGING

    leftDrive.spin(fwd, lateralMotorPower + correctMotorPower, volt);
    rightDrive.spin(fwd, lateralMotorPower - correctMotorPower, volt);
   
  if (fabs(error) < 0.5)
  {
    t += 0.01;
  }
  else
  {
    t = 0;
  }
 
  if (t >= 0.1)
  {
    break;
  }
 
    elapsedTime = timer.time(sec);
    vex::task::sleep(20);
  }
  leftDrive.stop();
  rightDrive.stop();
  return 0;
}

const double tkp = 0.1;
const double tki = 0.0;
const double tkd = 0.3;

int turnTo(float heading, float timeoutsec)
{
  float targetHeading = heading;
  float tError;
  float tPrevError = 0;
  float tDerivative;
  float tTotalError = 0;
  float t = 0;
  float elapsedTime = 0;
  //float dt = 0.02;
  vex::timer timer;

  while(elapsedTime < timeoutsec)
  {
    tError = restrain(targetHeading - imu.heading(degrees), -180, 180);
    tDerivative = (tError - tPrevError);
    tPrevError = tError;
    tTotalError += tError;
    double tMotorPower = (tError * tkp) + (tDerivative * tkd) + (tTotalError * tki);
    leftDrive.spin(fwd, tMotorPower, volt);
    rightDrive.spin(reverse, tMotorPower, volt);

  if (fabs(tError) < 0.5)
  {
    t += 0.01;
  }
  else
  {
    t = 0;  
  }
   
  if (t >= 0.1)
  {
    break;
  }
   
    elapsedTime = timer.time(sec);
    vex::task::sleep(20);
  }
  leftDrive.stop();
  rightDrive.stop();
  return 0;
}

//Controller Displays

int controllerDisplay()
{
  Controller.Screen.clearScreen();
  while(true)
  {
    int driveTrainTemp = (lf.temperature() + lb.temperature() + rf.temperature() + rb.temperature()) / 4;
    Controller.Screen.setCursor(1,1);
    Controller.Screen.print("%s            ", autoNames[autovar]);
    Controller.Screen.setCursor(2,1);
    Controller.Screen.print("Heading %.1f", imu.heading(degrees));
    Controller.Screen.setCursor(3,1);
    Controller.Screen.print("Motor Temp %.1f", driveTrainTemp);
    vex::task::sleep(500);
  }
  return 0;
}

//Controller Buttons

int intakeSpin()
{
  while (true)
  {
    switch (intakeStatus)
    {
      case 1:
        intake.spin(forward, 12, volt);
        convey.spin(forward, 12, volt);
        break;
     
      case 2:
        intake.spin(reverse, 12, volt);
        convey.spin(reverse, 12, volt);
        break;
     
      default:
        intake.stop(coast);
        convey.stop(coast);
        break;
    }

    vex::task::sleep(20);
  }
  return 0;
}

//Three Wire Buttons

void limitSwitchPressed()
{
  imu.calibrate();
  Controller.rumble("-");
  while(imu.isCalibrating()){wait(100, msec);}
  Controller.rumble("...");
}

void l1Pressed()
{
  if(solenoid.value())
  {
    solenoid.set(false);
  }else
  {
    solenoid.set(true);
  }
}

void upPressed()
{
  autovar++;
  if(autovar > 5)
    autovar = 0;
}

//Autonomous

void autonomous(void)
{
  intakeTask = task(intakeSpin);
  switch (autovar)
  {
    case 0: //no auton

    driveFor(-12, 1);
    solenoid.set(true);
    wait(0.25, sec);
    intakeStatus = 1;
    turnTo(0, 1);
    driveFor(20, 1);
    turnTo(100, 1);
    driveFor(24, 1);
    turnTo(190, 1);
    driveFor(20, 1);
    driveFor(10, 1);
    turnTo(45, 1);
    driveFor(14, 1);
    turnTo(345, 0.25);
    driveFor(-12, 1);
    solenoid.set(false);
    /*driveFor(12, 1);
    turnTo(90, 1);
    driveFor(-84, 3);*/
    /*solenoid.set(true);
    turnTo(0, 1);
    driveFor(24, 1);
    turnTo(270, 1);
    driveFor(24, 1);
    turnTo(180, 1);
    driveFor(24, 1);
    driveFor(12, 1);
    turnTo(315, 1);
    driveFor(12, 1);
    turnTo(45, 1);
    driveFor(-24, 1);
    //third goal
    driveFor(6, 1);
    turnTo(220, 1);
    driveFor(-144, 6);
    solenoid.set(true);
    turnTo(100, 1);
    driveFor(-48, 2);
    solenoid.set(false);
    //Fourth Goal
    driveFor(6, 1);
    turnTo(250, 1);
    driveFor(-72, 1);
    solenoid.set(true);
    turnTo(90, 1);
    driveFor(48, 2);
    turnTo(-225, 1);
    driveFor(-24, 1);
    solenoid.set(false);*/

    break;

    case 2: //red-

    driveFor(-14, 1);
    driveFor(-7, 1);
    driveFor(-7, 1);
    solenoid.set(true);
    wait(0.25, sec);
    intakeStatus = 1;
    wait(0.25, sec);
    driveFor(-6, 1);
    turnTo(270, 1);
    driveFor(24, 1);
    driveFor(6, 1);
    turnTo(0, 1);
    driveFor(16, 1);
    driveFor(2, 1);
    driveFor(-18, 1);
    wait(0.25, sec);
    turnTo(90, 1);
    driveFor(24, 1);
    driveFor(12, 1);
    driveFor(12, 1);

    break;

    case 3: //blue+

    driveFor(-70, 1);
    driveFor(-8, 1);
    solenoid.set(true);
    wait(0.25, sec);
    intakeStatus = 1;
    turnTo(190, 1);
    driveFor(24, 1);
    turnTo(0, 1);
    driveFor(-74, 1);

    break;

    case 4: //blue-

    driveFor(-14, 1);
    driveFor(-7, 1);
    driveFor(-7, 1);
    solenoid.set(true);
    wait(0.25, sec);
    intakeStatus = 1;
    wait(0.25, sec);
    driveFor(-6, 1);
    turnTo(90, 1);
    driveFor(24, 1);
    driveFor(6, 1);
    turnTo(0, 1);
    driveFor(16, 1);
    driveFor(2, 1);
    driveFor(-18, 1);
    wait(0.25, sec);
    turnTo(270, 1);
    driveFor(24, 1);
    driveFor(12, 1);
    driveFor(12, 1);

    break;

    case 5: //skills



    break;
  }
}


void usercontrol(void)
{
  intakeTask.stop();
  while (1)
  {
    leftDrive.spin(fwd, Controller.Axis3.position()/8.3333333, volt);
    rightDrive.spin(fwd, Controller.Axis2.position()/8.3333333, volt);
    //intake
    if(Controller.ButtonR1.pressing())
    {
      intake.spin(fwd, 12, volt);
    }else if(Controller.ButtonR2.pressing())
    {
      intake.spin(reverse, 12, volt);
    }else
    {
      intake.stop(hold);
    }
    //conveyor
    if(Controller.ButtonR1.pressing())
    {
      convey.spin(fwd, 12, volt);
    }else if(Controller.ButtonR2.pressing())
    {
      convey.spin(reverse, 12, volt);
    }else
    {
      convey.stop(hold);
    }

    wait(20, msec);
  }
}

int main()
{
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  controllerTask = task(controllerDisplay);
  lim.pressed(limitSwitchPressed);
  Controller.ButtonUp.pressed(upPressed);
  Controller.ButtonL1.pressed(l1Pressed);
  while (true) {
    vex::task::sleep(100);
  }
}