#include "Move.h"
//#include <Windows.h>

int main()
{
	srand(unsigned(time(NULL)));  //set random seed as time
	generatemaze();  
	displaymaze();
	cout << endl;
	Move m;
   // NSTimeInterval time;

	time_t F_start_time;
	time(&F_start_time);
	m.AgentForward(0);
	time_t F_end_time;
	time(&F_end_time);
	cout << "Agent Gets Result with Forward A* algorithm needs " << (F_end_time - F_start_time)<< " s." << endl;
	
	
	time_t B_start_time;
	time(&B_start_time);
	m.AgentBackward();
	time_t B_end_time;
	time(&B_end_time);
	cout << "Agent Gets Result with Backward A* algorithm needs " << (B_end_time - B_start_time) << " s." << endl;


	time_t FA_start_time;
	time(&FA_start_time);
	m.AgentForward(1);
	time_t  FA_end_time;
	time(&FA_end_time);
	cout << "Agent Gets Result with Forward Adaptive A* algorithm needs " << (FA_end_time - FA_start_time) << " ms." << endl;

	system("Pause");
    return 0;
}
