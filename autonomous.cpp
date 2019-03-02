#include "main.h"

/**
* Runs the user autonomous code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the autonomous
* mode. Alternatively, this function may be called in initialize or opcontrol
* for non-competition testing purposes.
*
* If the robot is disabled or communications is lost, the autonomous task
* will be stopped. Re-enabling the robot will restart the task, not re-start it
* from where it left off.
*/

pros::Controller master_auton(pros::E_CONTROLLER_MASTER);
//pros::ADIGyro gyro(1);
pros::ADIGyro gyro ('a');
pros::Motor leftFront_auton(20, true);
pros::Motor rightFront_auton(12);
pros::Motor leftBack_auton(9, true);
pros::Motor rightBack_auton(19);
pros::Motor fly_auton(1, false);
pros::Motor flyIntake_auton(14, true);
//check the ports
pros::Motor lift_auton(15, true);
pros::Motor capper_auton(6, true);

bool firstAuton=true;
float gyroprev=0;

void printTaskAuton(void* param)
{
  while(true)
  {
    pros::lcd::print(0, "%f", gyroprev);
	  pros::lcd::print(1, "%f", gyro.get_value());
    pros::delay(2);
  }
}

void flywheelTaskAuton(void* param)
{
while(true)
{
   fly_auton.move_velocity(200);
   while(fly_auton.get_actual_velocity()<180)//threshold
   {
     fly_auton.set_current_limit(2500);
     pros::delay(10);
   }
     while(fly_auton.get_actual_velocity()>=180)//threshold
   {
     fly_auton.set_current_limit(660);//amps
     pros::delay(10);
   }
   pros::delay(2);
}
}
void driveOneSquare(double squares, bool forward)
{
    if(forward==false)
    {
      leftFront_auton.move_velocity(-199.5);
      leftBack_auton.move_velocity(-199.5);
      rightFront_auton.move_velocity(-200);
      rightBack_auton.move_velocity(-200);
      pros::delay(squares*600);
    }
    else
    {
      leftFront_auton.move_velocity(200);
      leftBack_auton.move_velocity(200);
      rightFront_auton.move_velocity(200);
      rightBack_auton.move_velocity(200);
      pros::delay(squares*600);
    }
    leftFront_auton.move_voltage(0);
    leftBack_auton.move_voltage(0);
    rightFront_auton.move_voltage(0);
    rightBack_auton.move_voltage(0);
}
void driveOneSquareSlow(double squares, bool forward)
{
    if(forward==false)
    {
      leftFront_auton.move_velocity(-150);
      leftBack_auton.move_velocity(-150);
      rightFront_auton.move_velocity(-150);
      rightBack_auton.move_velocity(-150);
      pros::delay(squares*600);
    }
    else
    {
      leftFront_auton.move_velocity(150);
      leftBack_auton.move_velocity(150);
      rightFront_auton.move_velocity(150);
      rightBack_auton.move_velocity(150);
      pros::delay(squares*600);
    }
    leftFront_auton.move_voltage(0);
    leftBack_auton.move_voltage(0);
    rightFront_auton.move_voltage(0);
    rightBack_auton.move_voltage(0);
}
void gyroRight(double perc)
{
  gyroprev=gyro.get_value();

  while(gyro.get_value()>(-695)*perc+gyroprev)
  {
    leftFront_auton.move_velocity(200);
    rightFront_auton.move_velocity(-200);
    leftBack_auton.move_velocity(200);
    rightBack_auton.move_velocity(-200);
    pros::delay(2);
  }
  leftFront_auton.move_velocity(0);
  rightFront_auton.move_velocity(0);
  leftBack_auton.move_velocity(0);
  rightBack_auton.move_velocity(0);
}
void gyroLeft(double perc)
{
  gyroprev=gyro.get_value();
  while(gyro.get_value()<695*perc+gyroprev)
  {
    leftFront_auton.move_velocity(-200);
    rightFront_auton.move_velocity(200);
    leftBack_auton.move_velocity(-200);
    rightBack_auton.move_velocity(200);
    pros::delay(2);
  }
  leftFront_auton.move_velocity(0);
  rightFront_auton.move_velocity(0);
  leftBack_auton.move_velocity(0);
  rightBack_auton.move_velocity(0);
}
void autonomous()
{
  gyro.reset();
	pros::delay(1300);
  leftFront_auton.set_brake_mode(MOTOR_BRAKE_HOLD);
  leftBack_auton.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightFront_auton.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightBack_auton.set_brake_mode(MOTOR_BRAKE_HOLD);


  //start flywheel
  pros::Task p (printTaskAuton, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "printing");
  //start flywheel
  pros::Task f (flywheelTaskAuton, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "flywheel");


  lift_auton.tare_position();
  lift_auton.move_absolute(10,200);

  //drive towards cap
  driveOneSquare(1.7, true);

  //intake preload and start driving towards cap and intake ball
  flyIntake_auton.move_velocity(200);
  driveOneSquare(0.25,true);
  pros::delay(600);
  flyIntake_auton.move_voltage(0);

  //back up
  driveOneSquare(1.9, false);
  pros::delay(100);

  gyroRight(1.03);
  pros::delay(400);

  //shoot
  flyIntake_auton.move_velocity(200);
  pros::delay(400);
  flyIntake_auton.move_voltage(0);
  pros::delay(200);

  driveOneSquareSlow(1.7,true);
  pros::delay(200);

  gyroLeft(0.01);
  pros::delay(100);

  //shoot again
  flyIntake_auton.move_velocity(200);
  pros::delay(600);
  flyIntake_auton.move_voltage(0);
  pros::delay(200);

  driveOneSquareSlow(1.2,true);
  pros::delay(200);

  driveOneSquare(0.8,false);
  pros::delay(100);

  gyroLeft(0.6);

  flyIntake_auton.move_velocity(-200);
  driveOneSquare(1.4,true);
  pros::delay(400);
  flyIntake_auton.move_voltage(0);
  pros::delay(100);

  gyroLeft(0.7);
  pros::delay(10);

  driveOneSquare(3.4,true);
  pros::delay(100);

}
