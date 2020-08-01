#include <iostream>
#include <string>
using namespace std;

class Robot_Class
{
public:
	int id;
	int no_wheels;
	string robot_name;
	void move_robot();
	void stop_robot();
};

class Robot_Class_Derived: public Robot_Class
{
public:
	void turn_left();
	void turn_right();
};

void Robot_Class::move_robot()
{
	cout<<"Moving Robot"<<endl;
}
void Robot_Class::stop_robot()
{
	cout<<"Stopping Robot"<<endl;
}
void Robot_Class_Derived::turn_left()
{
	cout<<"Robot Turn left"<<endl;
}
void Robot_Class_Derived::turn_right()
{
	cout<<"Robot Turn Right"<<endl;
}
int main()
{
	Robot_Class_Derived robot;
	robot.id = 2;
	robot.robot_name = "Mobile robot";
	cout<<"Robot ID="<<robot.id<<endl;
	cout<<"Robot Name="<<robot.robot_name<<endl;
	robot.move_robot();
	robot.stop_robot();
	robot.turn_left();
	robot.turn_right();
	return 0;
}