#include "vex.h"
using namespace vex;

//Defining Motors, electronics, etc
competition Competition;
brain Brain;
controller Controller;

motor lf = motor(PORT7, ratio6_1, true);
motor lm = motor(PORT8, ratio6_1, true);
motor lb = motor(PORT9, ratio6_1, true);
motor rf = motor(PORT13, ratio6_1, false);
motor rm = motor(PORT12, ratio6_1, false);
motor rb = motor(PORT11, ratio6_1, false);

motor intake = motor(PORT6, ratio6_1, true);
motor lever = motor(PORT1, ratio36_1, true);

motor_group rightDrive = motor_group(rf, rm, rb);
motor_group leftDrive = motor_group(lf, lm, lb);

inertial imu1 = inertial(PORT16);
inertial imu2 = inertial(PORT17);

triport expander = triport(PORT10);

digital_out flap = digital_out(expander.D);
digital_out schlorb = digital_out(expander.A);
digital_out wing = digital_out(expander.B);
digital_out liftTop = digital_out(expander.C);
digital_out liftBottom = digital_out(expander.E);

//Variables
int autovar = 0;
int shift = 0;
const char* autoNames [] = {"no auton", "left", "right", "skills"};
double wheelSize = 3.25;
double gearRatio = 0.75;
double maxMotorVoltage = 12;
double target = 0;

float restrain(float num, float min, float max)
{
    while(num > max) num -= (max-min);
    while(num < min) num += (max-min);
    return num;
}

float averageHeading(inertial imu1, inertial imu2){
  //convert to radian
  float inertial1 = imu1.heading(degrees)*M_PI/180;
  float inertial2 = imu2.heading(degrees)*M_PI/180;

  float x = cos(inertial1) + cos(inertial2);
  float y = sin(inertial1) + sin(inertial2);

  float avg = atan2(y, x) * 180.0/M_PI;

  if(avg < 0) avg += 360.0;

  return avg;
}

bool headingLock = false;
double prevHeading;

//Lateral PID
int driveFor(float distance, float timeoutSec = -1)
{
    const double kp = 0.5;
    const double ki = 0.003;
    const double kd = 0.02;
    const double ckp = 0.1;
    const double integralLimit = 12;
    

    if(headingLock == false)
    {
        prevHeading = averageHeading(imu1, imu2);
    }

    rightDrive.resetPosition();
    leftDrive.resetPosition();

    double targetDistance = distance;
    double error = 0;
    double prevError = 0;
    double derivative = 0;
    double integral = 0;
    double settleTime = 0;
    vex::timer timer;
    double lastTime = timer.time(sec);

    while(timeoutSec < 0 || timer.time(sec) < timeoutSec)
    {
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

        if(integral > integralLimit) integral = integralLimit;
        if(integral < -integralLimit) integral = -integralLimit;

        derivative = (error - prevError) / dt;
        prevError = error;

        double lateralPower = (error * kp) + (integral * ki) + (derivative * kd);
        double headingError = restrain(prevHeading - averageHeading(imu1, imu2), -180, 180);
        double headingPower = headingError * ckp;

        double leftMotorPower = lateralPower + headingPower;
        double rightMotorPower = lateralPower - headingPower;

        leftMotorPower = fmin(fmax(leftMotorPower, -maxMotorVoltage), maxMotorVoltage);
        rightMotorPower = fmin(fmax(rightMotorPower, -maxMotorVoltage), maxMotorVoltage);

        leftDrive.spin(fwd, leftMotorPower, volt);
        rightDrive.spin(fwd, rightMotorPower, volt);

        if(fabs(error) < 1.0)
        {
            settleTime += dt;
        }
        else
        {
            settleTime = 0;
        }
        if(settleTime >= 0.2)
        {
            break;
        }
        task::sleep(10);
    }
    leftDrive.stop(coast);
    rightDrive.stop(coast);
    return 0;
}

//Turning PID
int turnTo(float heading, float timeoutSec = -1)
{
    const double kp = 0.35;
    const double ki = 0.0;
    const double kd = 0.03;
    const double integralLimit = 10;
    double targetHeading = heading;
    double error = 0;
    double prevError = 0;
    double derivative = 0;
    double integral = 0;
    double settleTime = 0;
    vex::timer timer;
    double lastTime = timer.time(sec);

    while(timeoutSec < 0 || timer.time(sec) < timeoutSec)
    {
        double currentTime = timer.time(sec);
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        if(dt <= 0)
        {
            dt = 0.01;
        }
        error = restrain(targetHeading - averageHeading(imu1, imu2), -180, 180);
        
        if(fabs(error) < 6)
        {
            integral += error * dt;
        }
        
        if(integral > integralLimit) integral = integralLimit;
        if(integral < -integralLimit) integral = -integralLimit;

        derivative = (error - prevError) / dt;
        prevError = error;

        double headingPower = (error * kp) + (integral * ki) + (derivative * kd);
        double leftMotorPower = headingPower;
        double rightMotorPower = headingPower;

        leftMotorPower = fmin(fmax(leftMotorPower, -maxMotorVoltage), maxMotorVoltage);
        rightMotorPower = fmin(fmax(rightMotorPower, -maxMotorVoltage), maxMotorVoltage);

        leftDrive.spin(fwd, leftMotorPower, volt);
        rightDrive.spin(reverse, rightMotorPower, volt);

        if(fabs(error) < 0.5)
        {
            settleTime += dt;
        }
        else
        {
            settleTime = 0;
        }

        if(settleTime >= 0.1)
        {
            break;
        }
        task::sleep(10);
    }
    leftDrive.stop(brake);
    rightDrive.stop(brake);
    prevHeading = heading;
    headingLock = true;
    return 0;
}

//Lever PID
int leverPID()
{
    const double kp = 0.4;
    double settleTime = 0;
    vex::timer timer;
    double lastTime = timer.time(sec);

    while(true)
    {
        double currentTime = timer.time(sec);
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        if(dt <= 0)
        {
            dt = 0.01;
        }
        double error = (target - lever.position(degrees));

        double motorPower = error * kp;

        lever.spin(fwd, motorPower, volt);

        if(fabs(error) < 5)
        {
            settleTime += dt;
        }
        else
        {
            settleTime = 0;
        }
        if(settleTime >= 0.1)
        {
            lever.stop();
        }

    task::sleep(10);
    }

    return 0;
}

int controllerDisplay()
{
  Controller.Screen.clearScreen();
  Brain.Screen.clearScreen();
  while(true)
  {
    Controller.Screen.setCursor(1,1);
    Controller.Screen.print("%s            ", autoNames[autovar]);
    Controller.Screen.setCursor(2,1);
    Controller.Screen.print("Heading %.1f   ", averageHeading(imu1, imu2));
    Controller.Screen.setCursor(3,1);
    //Controller.Screen.print("Intake hopper %s        ", intakeNames2[intakelevel]);
    Controller.Screen.print("Battery %d%%", Brain.Battery.capacity());
    //Controller.Screen.print(shift);

    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("%s            ", autoNames[autovar]);
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("Heading %.1f   ", averageHeading(imu1, imu2));
    Brain.Screen.setCursor(3,1);
    //Brain.Screen.print("Intake hopper %s        ", intakeNames2[intakelevel]);
    Brain.Screen.print("Battery %d%%", Brain.Battery.capacity());
    
    
    task::sleep(100);
  }
  return 0;
}

//Auto Functions
void middle()
{
    liftBottom.set(false);
    liftTop.set(true);
}

void upper()
{
    liftBottom.set(true);
    liftTop.set(true);
}

void bottom()
{
    liftBottom.set(false);
    liftTop.set(false);
}

void flapOut()
{
    flap.set(true);
    target = 120;
    intake.spin(reverse, 12, volt);

    wait(500, msec);

    target = -10;
    intake.stop();
    wait(0.5, sec);
    flap.set(false);
    
}

void autonomous(void)
{
    leftDrive.setStopping(coast);
    rightDrive.setStopping(coast);
    schlorb.set(false);
    wing.set(true);
    flap.set(false);
    liftTop.set(false);
    liftBottom.set(false);
    switch(autovar)
    {
        case 0: //no auto



        break;

        case 1: //left



        break;

        case 2: //right

        driveFor(18, 1);
        turnTo(90, 1);
        driveFor(31.5, 2);
        turnTo(180, 1);
        schlorb.set(true);
        intake.spin(fwd, 12, volt);
        driveFor(15, 1);
        driveFor(4, 1);
        wait(1, sec);
        driveFor(-15, 1);
        turnTo(0, 1);
        upper();
        driveFor(15, 1);
        flapOut();

        break;

        case 3: //skills

        

        break;
    }
}

void userControl(void)
{
    leftDrive.setStopping(brake);
    rightDrive.setStopping(brake);
    intake.setStopping(coast);
    intake.setStopping(brake);
    schlorb.set(false);
    wing.set(true);
    flap.set(false);
    liftTop.set(false);
    liftBottom.set(false);

    while(Competition.isDriverControl())
    {
        leftDrive.spin(fwd, (Controller.Axis3.position() / 8.333333), volt);
        rightDrive.spin(fwd, (Controller.Axis2.position() / 8.333333), volt);

        if(Controller.ButtonR1.pressing())
        {
            intake.spin(fwd, 12, volt);
        }
        else if(Controller.ButtonR2.pressing())
        {
            intake.spin(reverse, 12, volt);
        }
        else
        {
            intake.stop();
        }
        
        wait(10, msec);
    }
}

void upPressed()
{
    autovar++;
    if(autovar > 3)
    {
        autovar = 0;
    }
}

void downPressed()
{
    autovar--;
    if(autovar < 0)
    {
        autovar = 3;
    }
}

void bPressed()
{
    if(schlorb.value())
    {
        schlorb.set(false);
    }
    else
    {
        schlorb.set(true);
    }
}

void xPressed()
{
    if(wing.value())
    {
        wing.set(false);
    }
    else
    {
        wing.set(true);
    }
}

void aPressed()
{
    if(flap.value())
    {
        flap.set(false);
    }
    else
    {
        flap.set(true);
    }
}

void rightPressed()
{
    autonomous();
}

void l1Pressed()
{
    if(!Controller.ButtonL2.pressing() && Controller.ButtonL1.pressing())
    {
        flap.set(true);
        target = 120;
        intake.spin(reverse, 12, volt);

        wait(500, msec);

        target = -10;
        intake.stop();
        wait(0.5, sec);
        flap.set(false);
    }
    else if(Controller.ButtonL2.pressing())
    {
        liftBottom.set(false);
        liftTop.set(false);
    }
}

void r1Pressed()
{
    if(Controller.ButtonL2.pressing())
    {
        intake.stop();
        liftBottom.set(true);
        liftTop.set(true);
    }
}

void r2Pressed()
{
    if(Controller.ButtonL2.pressing())
    {
        intake.stop();
        liftBottom.set(false);
        liftTop.set(true);
    }
}

void leftPressed()
{
    if(Controller.ButtonL2.pressing())
    {
        Controller.rumble("-");
        imu1. calibrate();
        imu2.calibrate();
        lever.resetPosition();
        while(imu1.isCalibrating())
        {
            wait(100, msec);
        }
        Controller.rumble("...");
    }
}

int main()
{
    Competition.autonomous(autonomous);
    Competition.drivercontrol(userControl);
    Controller.ButtonUp.pressed(upPressed);
    Controller.ButtonDown.pressed(downPressed);
    Controller.ButtonB.pressed(bPressed);
    Controller.ButtonX.pressed(xPressed);
    Controller.ButtonRight.pressed(rightPressed);
    Controller.ButtonL1.pressed(l1Pressed);
    Controller.ButtonR1.pressed(r1Pressed);
    Controller.ButtonR2.pressed(r2Pressed);
    Controller.ButtonLeft.pressed(leftPressed);
    vex::task fishy(controllerDisplay);
    vex::task doggy(leverPID);

    while(true)
    {
        wait(100, msec);
    }
}