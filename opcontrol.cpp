#include "main.h"
#include "math.h"
using namespace std;

pros::Controller master(pros::E_CONTROLLER_MASTER);
//redo motor ports
pros::Motor leftFront(20, true);
pros::Motor rightFront(12);
pros::Motor leftBack(9, true);
pros::Motor rightBack(19,false);
pros::Motor fly(1, false);
pros::Motor flyIntake(14, true);
//check the ports
pros::Motor lift(15, true);
pros::Motor capper(6, true);

bool coast=true;
bool first=true;
double currentDist=0;
bool forwards=false;
bool armExtend=false;
bool lowCapUp=false;
bool highCapUp=false;
double amps;

void driveTask(void* param)
{
  while(true)
  {
    if(master.get_digital(DIGITAL_X)&&forwards==true)
    {
      forwards=false;
      while(master.get_digital(DIGITAL_X))
      {
        pros::delay(2);
      }
    }
    else if(master.get_digital(DIGITAL_X)&&forwards==false)
    {
      forwards=true;
      while(master.get_digital(DIGITAL_X))
      {
        pros::delay(2);
      }
    }
    if(forwards)
    {
      master.print(0, 0, "%s", "fwd");
      leftFront.move((-1*master.get_analog(ANALOG_RIGHT_Y)));
			leftBack.move((-1*master.get_analog(ANALOG_RIGHT_Y)));
			rightFront.move((-1*master.get_analog(ANALOG_LEFT_Y)));
			rightBack.move((-1*master.get_analog(ANALOG_LEFT_Y)));
    }
    else
    {
      master.print(0, 0, "%s", "bkd");
      leftFront.move((master.get_analog(ANALOG_LEFT_Y)));
			leftBack.move((master.get_analog(ANALOG_LEFT_Y)));
			rightFront.move((master.get_analog(ANALOG_RIGHT_Y)));
			rightBack.move((master.get_analog(ANALOG_RIGHT_Y)));

    }
    pros::delay(2);
  }
}

void brakeTask(void* param)
{
  while(true)
  {
    if(master.get_digital(DIGITAL_Y)&&coast==false)
    {
      master.print(0,0,"%s","coast");
      coast=true;
      leftFront.set_brake_mode(MOTOR_BRAKE_COAST);
      leftBack.set_brake_mode(MOTOR_BRAKE_COAST);
      rightFront.set_brake_mode(MOTOR_BRAKE_COAST);
      rightBack.set_brake_mode(MOTOR_BRAKE_COAST);

      while(master.get_digital(DIGITAL_Y))
      {
        pros::delay(2);
      }
    }
    else if(master.get_digital(DIGITAL_Y)&&coast==true)
    {
      master.print(0,0,"%s","hold");
      coast=false;
      leftFront.set_brake_mode(MOTOR_BRAKE_HOLD);
      leftBack.set_brake_mode(MOTOR_BRAKE_HOLD);
      rightFront.set_brake_mode(MOTOR_BRAKE_HOLD);
      rightBack.set_brake_mode(MOTOR_BRAKE_HOLD);

      while(master.get_digital(DIGITAL_Y))
      {
        pros::delay(2);
      }
    }
    pros::delay(2);
  }
}

void intakeTask(void* param)
{
  while(true)
  {
    if(master.get_digital(DIGITAL_R1))
    {
      flyIntake.move_velocity(200);
    }
    else if(master.get_digital(DIGITAL_R2))
    {
      flyIntake.move_velocity(-200);
    }
    else
    {
      flyIntake.move_voltage(0);
    }
    pros::delay(2);
  }
}

double min(double a, double b)
{
  if(a<=b)
  {
    return a;
  }
  return b;
}
void flywheelTask(void* param)
{
  while(true)
  {
    if(first==true)
    {
        double vel=2;
        fly.move_velocity(2);
        pros::delay(100);
        while(vel<200)
        {
            fly.move_velocity(vel*vel);
            vel=fly.get_actual_velocity();
        }
        first=false;
    }
    fly.move_velocity(200);
  }
}


void armTask(void* param)
{
  while(true)
  {
    //manual lift
    if(master.get_digital(DIGITAL_LEFT))
    {
      lift.move_velocity(200);
    }
    else if(master.get_digital(DIGITAL_DOWN))
    {
      lift.move_velocity(-200);
    }
    else if((lift.get_position()>500&&lift.get_position()<2600))
    {
      lift.move_velocity(3);
    }
    else
    {
      lift.move_velocity(0);
    }

    //manual capper
    if(master.get_digital(DIGITAL_A))
    {
      capper.move_velocity(200);
    }
    else if(master.get_digital(DIGITAL_B))
    {
      capper.move_velocity(-200);
    }
    else
    {
      capper.move_velocity(0);
    }

    //capper encoder
    if(master.get_digital(DIGITAL_L1))
    {
      capper.tare_position();
      lift.tare_position();
      if(!armExtend)//DOWN
      {
        capper.move_absolute(-2600, 200);
        armExtend=true;
      }
      else if(armExtend)
      {
        capper.move_absolute(2600, 200);
        armExtend=false;
      }
      while(master.get_digital(DIGITAL_L1))
        pros::delay(2);
    }
    //low cap
    if(master.get_digital(DIGITAL_RIGHT))
    {
      capper.tare_position();
      lift.tare_position();
      if(!highCapUp)
      {
        leftFront.set_brake_mode(MOTOR_BRAKE_HOLD);
        leftBack.set_brake_mode(MOTOR_BRAKE_HOLD);
        rightFront.set_brake_mode(MOTOR_BRAKE_HOLD);
        rightBack.set_brake_mode(MOTOR_BRAKE_HOLD);

        lift.move_absolute(4300, 200);
        capper.move_absolute(-3000, 200);
        highCapUp=true;
      }
      else if(highCapUp)
      {
        leftFront.set_brake_mode(MOTOR_BRAKE_COAST);
        leftBack.set_brake_mode(MOTOR_BRAKE_COAST);
        rightFront.set_brake_mode(MOTOR_BRAKE_COAST);
        rightBack.set_brake_mode(MOTOR_BRAKE_COAST);

        capper.move_absolute(3000, 200);
        lift.move_absolute(-4300, 200);
        highCapUp=false;
      }
      while(master.get_digital(DIGITAL_RIGHT))
        pros::delay(2);
    }
    //high cap
    if(master.get_digital(DIGITAL_UP))
    {
      capper.tare_position();
      lift.tare_position();
      if(!highCapUp)
      {
        leftFront.set_brake_mode(MOTOR_BRAKE_HOLD);
        leftBack.set_brake_mode(MOTOR_BRAKE_HOLD);
        rightFront.set_brake_mode(MOTOR_BRAKE_HOLD);
        rightBack.set_brake_mode(MOTOR_BRAKE_HOLD);

        lift.move_absolute(3500, 200);
        capper.move_absolute(-4500, 200);
        highCapUp=true;
      }
      else if(highCapUp)
      {
        leftFront.set_brake_mode(MOTOR_BRAKE_COAST);
        leftBack.set_brake_mode(MOTOR_BRAKE_COAST);
        rightFront.set_brake_mode(MOTOR_BRAKE_COAST);
        rightBack.set_brake_mode(MOTOR_BRAKE_COAST);

        capper.move_absolute(4500, 200);
        lift.move_absolute(-3500, 200);
        highCapUp=false;
      }
      while(master.get_digital(DIGITAL_UP))
        pros::delay(2);
    }
    //high cap backwards
    if(master.get_digital(DIGITAL_L2))
    {
      capper.tare_position();
      lift.tare_position();
      if(!highCapUp)
      {
        leftFront.set_brake_mode(MOTOR_BRAKE_HOLD);
        leftBack.set_brake_mode(MOTOR_BRAKE_HOLD);
        rightFront.set_brake_mode(MOTOR_BRAKE_HOLD);
        rightBack.set_brake_mode(MOTOR_BRAKE_HOLD);

        lift.move_absolute(4300, 200);
        capper.move_absolute(-3000, 200);
        highCapUp=true;
      }
      else if(highCapUp)
      {
        leftFront.set_brake_mode(MOTOR_BRAKE_COAST);
        leftBack.set_brake_mode(MOTOR_BRAKE_COAST);
        rightFront.set_brake_mode(MOTOR_BRAKE_COAST);
        rightBack.set_brake_mode(MOTOR_BRAKE_COAST);

        capper.move_absolute(3000, 200);
        lift.move_absolute(-4300, 200);
        highCapUp=false;
      }
      while(master.get_digital(DIGITAL_L2))
        pros::delay(2);
    }
    pros::delay(2);
  }
}
void opcontrol()
{
  pros::delay(1300);

  fly.move_velocity(5);
  pros::delay(100);

  lift.set_brake_mode(MOTOR_BRAKE_HOLD);
  capper.set_brake_mode(MOTOR_BRAKE_HOLD);
  leftFront.set_brake_mode(MOTOR_BRAKE_COAST);
  leftBack.set_brake_mode(MOTOR_BRAKE_COAST);
  rightFront.set_brake_mode(MOTOR_BRAKE_COAST);
  rightBack.set_brake_mode(MOTOR_BRAKE_COAST);

  //TASK
  pros::Task f (flywheelTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "flywheel");

  pros::Task i (intakeTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake");

  pros::Task d (driveTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "drive");

  pros::Task b (brakeTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "braking");

  pros::Task a (armTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "arm");
	while(true)
	{
		pros::delay(2);
	}
}
