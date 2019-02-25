#include "main.h"
#include "iostream"
#include "stdio.h"
using namespace std;

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

 void printTaskAuton(void* param)
 {
   while(true)
   {
     pros::lcd::print(0, "%f", leftFront_auton.get_position());
 	   pros::lcd::print(1, "%f", leftBack_auton.get_position());
     pros::lcd::print(2, "%f", rightFront_auton.get_position());
     pros::lcd::print(3, "%f", rightBack_auton.get_position());
     pros::delay(2);
   }
 }
 void flywheelTaskAuton(void* param)
 {
   while(true)
   {
     fly_auton.move_velocity(200);
     while(fly_auton.get_actual_velocity()<170)//threshold
     {
       fly_auton.set_current_limit(2500);
       pros::delay(10);
     }
     while(fly_auton.get_actual_velocity()>=170)//threshold
     {
       fly_auton.set_current_limit(670);//amps
       pros::delay(10);
     }
     pros::delay(2);
   }
 }
 void driveOneSquare(double squares, bool forward)
{
    if(forward==false)
    {
      leftFront_auton.move_velocity(-200);
      leftBack_auton.move_velocity(-200);
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
void encoderRightTurn(double perc)
{
  leftFront_auton.tare_position();
  rightFront_auton.tare_position();
  leftBack_auton.tare_position();
  rightBack_auton.tare_position();
  leftFront_auton.move_absolute(1250*perc,200);
  rightFront_auton.move_absolute(-1250*perc,200);
  leftBack_auton.move_absolute(1250*perc,200);
  rightBack_auton.move_absolute(-1250*perc,200);
  pros::delay(400);
}
void encoderLeftTurn(double perc)
{
  leftFront_auton.tare_position();
  rightFront_auton. tare_position();
  leftBack_auton.tare_position();
  rightBack_auton.tare_position();
  leftFront_auton.move_absolute(-1250*perc,200);
  rightFront_auton.move_absolute(1250*perc,200);
  leftBack_auton.move_absolute(-1250*perc,200);
  rightBack_auton.move_absolute(1250*perc,200);
  pros::delay(400);
}
void smoothTurn(double left, double right, double timing)
{
  leftFront_auton.tare_position();
  rightFront_auton. tare_position();
  leftBack_auton.tare_position();
  rightBack_auton.tare_position();

  leftFront_auton.move_velocity(left);
  leftBack_auton.move_velocity(left);
  rightFront_auton.move_velocity(right);
  rightBack_auton.move_velocity(right);
  pros::delay(timing);

  leftFront_auton.move_voltage(0);
  leftBack_auton.move_voltage(0);
  rightFront_auton.move_voltage(0);
  rightBack_auton.move_voltage(0);

}
void capper(bool up)
{
  capper_auton.tare_position();
  if(up)
    capper_auton.move_absolute(-2600, 200);
  else
    capper_auton.move_absolute(2600, 200);
}
void autonomous()
{
  pros::Task p (printTaskAuton, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "printing");

  leftFront_auton.set_brake_mode(MOTOR_BRAKE_HOLD);
  leftBack_auton.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightFront_auton.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightBack_auton.set_brake_mode(MOTOR_BRAKE_HOLD);

  driveOneSquare(3,true);

  /*//kick
  fly_auton.move_velocity(10);
  pros::delay(100);

  //start flywheel
  pros::Task f (flywheelTaskAuton, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "flywheel");

  //drive towards cap
  driveOneSquare(1.7, true);

  //intake preload and start driving towards cap and intake ball
  flyIntake_auton.move_velocity(200);
  driveOneSquare(0.3,true);
  pros::delay(500);
  flyIntake_auton.move_voltage(0);

  //back up
  driveOneSquare(0.3, false);
  pros::delay(10);

  //capper down
  capper(true);
  pros::delay(10);

  //smooth turn towards cap
  //left, right, time
  smoothTurn(-200,-10, 1050);
  pros::delay(10);

  //back up
  driveOneSquare(0.6, false);
  pros::delay(10);

  //capper up
  capper(false);
  pros::delay(200);

  //back up
  driveOneSquare(1.5, true);
  pros::delay(10);

  //turn to drive to pole
  smoothTurn(200, 61, 650);
  pros::delay(10);

  //drive to pole
  driveOneSquare(1.3,true);
  pros::delay(10);

  encoderLeftTurn(0.97);
  pros::delay(10);

  driveOneSquare(0.3,true);
  pros::delay(10);

  //cap up
  capper_auton.tare_position();
  lift_auton.tare_position();
  lift_auton.move_absolute(4300, 200);
  capper_auton.move_absolute(-3000, 200);
  pros::delay(1600);

  //back up
  driveOneSquare(0.75, false);
  pros::delay(10);

  //cap down
  capper_auton.tare_position();
  lift_auton.tare_position();
  capper_auton.move_absolute(3000, 200);
  lift_auton.move_absolute(-4300, 200);
  pros::delay(800);

  encoderLeftTurn(0.6);
  pros::delay(200);

  driveOneSquare(0.9,true);
  pros::delay(200);

  encoderLeftTurn(0.95);
  pros::delay(200);

  driveOneSquare(2.5, true);
  pros::delay(100);

  driveOneSquare(0.4,false);
  pros::delay(100);

  encoderLeftTurn(0.46);
  pros::delay(200);

  flyIntake_auton.move_velocity(200);
  pros::delay(1000);
  flyIntake_auton.move_voltage(0);*/

	//freopen("redback.out", "w", stdout);

  /*cout<<"Left Front: "<<leftFront_auton.get_position()<<endl;
  cout<<"Left Back: "<<leftBack_auton.get_position()<<endl;
  cout<<"Right Front: "<<rightFront_auton.get_position()<<endl;
  cout<<"Right Back: "<<rightBack_auton.get_position()<<endl;*/

}
