#include <iostream>

using namespace std;
namespace robot {
	void process(void)
	{
		cout<<"Processing by Robot"<<endl;
	}
}

namespace machine {
	void process(void)
	{
		cout<<"Processing by Machine"<<endl;
	}
}
int main()
{
	robot::process();
	machine::process();
}
