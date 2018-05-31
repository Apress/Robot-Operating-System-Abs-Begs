#include <iostream>

#include <fstream>

#include <string>

using namespace std;


int main()

{


	ofstream out_file;
	string data = "Robot_ID=0";

	cout<<"Write data:"<<data<<endl;

	out_file.open("Config.txt");
	out_file <<data<<endl;
	out_file.close();


	ifstream in_file;
	in_file.open("Config.txt");

	in_file >> data;

	cout<<"Read data:"<<data<<endl;

	in_file.close();

	return 0;



}
				

	


