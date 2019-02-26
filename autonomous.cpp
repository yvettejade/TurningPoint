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
pros::ADIEncoder left_auton(5,6,false);
pros::ADIEncoder right_auton(7,8,false);
bool firstAuton=true;
int leftPrev=0;
int rightPrev=0;


void printTaskAuton(void* param)
{
 while(true)
 {
   pros::lcd::print(0, "%d", abs(leftPrev-left_auton.get_value()));
	 pros::lcd::print(1, "%d", abs(rightPrev-right_auton.get_value()));
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
    bool leftDone=false, rightDone=false;
    while(1)
    {
      //check if done moving
      if(abs(leftPrev-left_auton.get_value())>=1800*squares) //change to check one square
      {
        leftFront_auton.move_voltage(0);
        leftBack_auton.move_voltage(0);
        leftDone=true;
        leftPrev=left_auton.get_value();
      }
      if(abs(rightPrev-right_auton.get_value())>=1800*squares) //change to check one square
      {
        rightFront_auton.move_voltage(0);
        rightBack_auton.move_voltage(0);
        rightDone=true;
        rightPrev=right_auton.get_value();
      }
      //set movement
      double speedLeft=0, speedRight=0;
      if(leftDone==false&&rightDone==false)
      {
        speedLeft=200;
        speedRight=200;
      }
      else if(leftDone==false&&rightDone==true)
        speedLeft=200;
      else if(leftDone==true&&rightDone==false)
        speedRight=200;
      else
      {
        leftFront_auton.move_velocity(0);
        leftBack_auton.move_velocity(0);
        rightFront_auton.move_velocity(0);
        rightBack_auton.move_velocity(0);
        return;
      }
      //move
      if(forward==false)
      {

        leftFront_auton.move_velocity(-1*speedLeft);
        leftBack_auton.move_velocity(-1*speedLeft);
        rightFront_auton.move_velocity(-1*speedRight);
        rightBack_auton.move_velocity(-1*speedRight);
      }
      if(forward==true)
      {
        leftFront_auton.move_velocity(speedLeft);
        leftBack_auton.move_velocity(speedLeft);
        rightFront_auton.move_velocity(speedRight);
        rightBack_auton.move_velocity(speedRight);
      }
      pros::delay(10);
    }
}
void smoothTurn(double left, double right)
{
  bool leftDone=false, rightDone=false;
  while(1)
  {
    //check if done moving
    if(abs(leftPrev-left_auton.get_value())>=1800*left) //change to check one square
    {
      leftFront_auton.move_voltage(0);
      leftBack_auton.move_voltage(0);
      leftDone=true;
      leftPrev=left_auton.get_value();
    }
    if(abs(rightPrev-right_auton.get_value())>=1800*right) //change to check one square
    {
      rightFront_auton.move_voltage(0);
      rightBack_auton.move_voltage(0);
      rightDone=true;
      rightPrev=right_auton.get_value();
    }
    //set movement
    double speedLeft=0, speedRight=0;
    if(leftDone==false&&rightDone==false)
    {
      speedLeft=200;
      speedRight=200;
    }
    else if(leftDone==false&&rightDone==true)
      speedLeft=200;
    else if(leftDone==true&&rightDone==false)
      speedRight=200;
    else
    {
      leftFront_auton.move_velocity(0);
      leftBack_auton.move_velocity(0);
      rightFront_auton.move_velocity(0);
      rightBack_auton.move_velocity(0);
      return;
    }
    //move
    leftFront_auton.move_velocity(-1*speedLeft);
    leftBack_auton.move_velocity(-1*speedLeft);
    rightFront_auton.move_velocity(-1*speedRight);
    rightBack_auton.move_velocity(-1*speedRight);
    pros::delay(10);
  }
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
  //clear drive motors
  leftPrev=left_auton.get_value();
  rightPrev=right_auton.get_value();

  //start printing
  pros::Task p (printTaskAuton, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "printing");

  //brake drive motors
  leftFront_auton.set_brake_mode(MOTOR_BRAKE_HOLD);
  leftBack_auton.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightFront_auton.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightBack_auton.set_brake_mode(MOTOR_BRAKE_HOLD);

  //kick
  fly_auton.move_velocity(10);
  pros::delay(100);

  //start flywheel
  pros::Task f (flywheelTaskAuton, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "flywheel");

  //drive towards cap
  driveOneSquare(1.7, true);

  //intake preload and start driving towards cap and intake ball
  leftPrev=left_auton.get_value();
  rightPrev=right_auton.get_value();
  flyIntake_auton.move_velocity(200);
  driveOneSquare(0.3,true);
  pros::delay(500);
  flyIntake_auton.move_voltage(0);

  //back up
  leftPrev=left_auton.get_value();
  rightPrev=right_auton.get_value();
  driveOneSquare(0.5, false);

  //capper down
  capper(true);

  //smooth turn towards cap
  //left, right
  leftPrev=left_auton.get_value();
  rightPrev=right_auton.get_value();
  smoothTurn(1.5,0);

  //back up
  leftPrev=left_auton.get_value();
  rightPrev=right_auton.get_value();
  driveOneSquare(0.6, false);

  /*//capper up
  capper(false);
  pros::delay(150);

  //back up
  driveOneSquare(1.5, true);

  //turn to drive to pole
  smoothTurn(-3, -1.5);

  //drive to pole
  driveOneSquare(1.3,true);

  //turn to face pole
  smoothTurn(2,0);

  //drive into pole
  driveOneSquare(0.3,true);

  //cap up
  capper_auton.tare_position();
  lift_auton.tare_position();
  lift_auton.move_absolute(4300, 200);
  capper_auton.move_absolute(-3000, 200);
  pros::delay(1600);

  //back up
  driveOneSquare(0.75, false);

  //cap down
  capper_auton.tare_position();
  lift_auton.tare_position();
  capper_auton.move_absolute(3000, 200);
  lift_auton.move_absolute(-4300, 200);
  pros::delay(600);

  //turn parallel to platform
  smoothTurn(2,0);

  //line up with platform
  driveOneSquare(0.9,true);

  //face platform
  smoothTurn(2,0);

  //drive onto platform
  driveOneSquare(2.5, true);

  //back up to align with flags
  driveOneSquare(0.4,false);

  //turn towards flags
  smoothTurn(0,1);

  //fire
  flyIntake_auton.move_velocity(200);
  pros::delay(1000);
  flyIntake_auton.move_voltage(0);*/

}
