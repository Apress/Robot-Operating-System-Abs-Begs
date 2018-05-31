#include <iostream>
#include <string>

using namespace std;

struct Robot_Struct

{
	int id;
	int no_wheels;
	string robot_name;


};

class Robot_Class
{


public:
	int id;
	int no_wheels;

	string robot_name;

	void move_robot();

	void stop_robot();


};

void Robot_Class::move_robot()
{

	cout<<"Moving Robot"<<endl;

}

void Robot_Class::stop_robot()
{

	cout<<"Stopping Robot"<<endl;
}

int main()

{
	Robot_Struct robot_1;
	Robot_Class robot_2;



	robot_1.id = 2;
	robot_1.robot_name = "Mobile robot";


	robot_2.id = 3;
	robot_2.robot_name = "Humanoid robot";


	cout<<"ID="<<robot_1.id<<"\t"<<"Robot Name"<<robot_1.robot_name<<endl;

	cout<<"ID="<<robot_2.id<<"\t"<<"Robot Name"<<robot_2.robot_name<<endl;

	robot_2.move_robot();
	robot_2.stop_robot();

	return 0;


}
