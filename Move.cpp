#include "Move.h"
extern int GridWorld[MAZESIZE][MAZESIZE];
Move::Move()
{
	counter = 0;
	do
	{
		start.first = rand() % MAZESIZE;
		start.second = rand() % MAZESIZE;
	} while (GridWorld[start.first][start.second] == 0);  //set a random unblocked grid as start

	current = start;

	do
	{
		goal.first = rand() % MAZESIZE;
		goal.first = rand() % MAZESIZE;
	} while (goal == start || GridWorld[goal.first][goal.second] == 0);  //set a random unblocked grid as goal

	for (int row_cnt = 0; row_cnt < MAZESIZE; row_cnt++)   //initialize the grid world in agent's mind
		for (int col_cnt = 0; col_cnt < MAZESIZE; col_cnt++)
		{
			AgentWorld[row_cnt][col_cnt].set_f(0);
			AgentWorld[row_cnt][col_cnt].set_g(0);
			AgentWorld[row_cnt][col_cnt].set_h(0);
			AgentWorld[row_cnt][col_cnt].set_position(pair<int, int>(row_cnt, col_cnt));
			AgentWorld[row_cnt][col_cnt].set_from(pair<int, int>(row_cnt, col_cnt));

			B_AgentWorld[row_cnt][col_cnt].set_f(0);
			B_AgentWorld[row_cnt][col_cnt].set_g(0);
			B_AgentWorld[row_cnt][col_cnt].set_h(0);
			B_AgentWorld[row_cnt][col_cnt].set_position(pair<int, int>(row_cnt, col_cnt));
			B_AgentWorld[row_cnt][col_cnt].set_from(pair<int, int>(row_cnt, col_cnt));
		}
	AgentWorld[start.first][start.second].set_g(0);
	AgentWorld[start.first][start.second].set_h(goal.first - start.first + goal.second - start.second);
	AgentWorld[start.first][start.second].update_f();
	Openlist.insert(AgentWorld[start.first][start.second]);
	AgentWorld[goal.first][goal.second].set_g(INT_MAX);
	AgentWorld[goal.first][goal.second].update_f();

	B_AgentWorld[start.first][start.second].set_g(INT_MAX);
	B_AgentWorld[start.first][start.second].set_h(start.first - goal.first + start.second - goal.second);
	B_AgentWorld[start.first][start.second].update_f();
	B_AgentWorld[goal.first][goal.second].set_g(0);
	B_AgentWorld[goal.first][goal.second].update_f();
}

void Move::reinitialize()
{
	current = start;
	for (int row_cnt = 0; row_cnt < MAZESIZE; row_cnt++)   //initialize the grid world in agent's mind
		for (int col_cnt = 0; col_cnt < MAZESIZE; col_cnt++)
		{
			AgentWorld[row_cnt][col_cnt].set_f(0);
			AgentWorld[row_cnt][col_cnt].set_g(0);
			AgentWorld[row_cnt][col_cnt].set_h(0);
			AgentWorld[row_cnt][col_cnt].set_search(0);
			AgentWorld[row_cnt][col_cnt].set_h_new(0);
			AgentWorld[row_cnt][col_cnt].set_position(pair<int, int>(row_cnt, col_cnt));
			AgentWorld[row_cnt][col_cnt].set_from(pair<int, int>(row_cnt, col_cnt));
			AgentWorld[row_cnt][col_cnt].clear_observe();
		}
	AgentWorld[start.first][start.second].set_g(0);
	AgentWorld[start.first][start.second].set_h(goal.first - start.first + goal.second - start.second);
	AgentWorld[start.first][start.second].update_f();
	Openlist.insert(AgentWorld[start.first][start.second]);
	AgentWorld[goal.first][goal.second].set_g(INT_MAX);
	AgentWorld[goal.first][goal.second].update_f();

	B_AgentWorld[start.first][start.second].set_g(INT_MAX);
	B_AgentWorld[start.first][start.second].set_h(start.first - goal.first + start.second - goal.second);
	B_AgentWorld[start.first][start.second].update_f();
	B_AgentWorld[goal.first][goal.second].set_g(0);
	B_AgentWorld[goal.first][goal.second].update_f();
}

void Move::AgentForward(bool adapt)
{
	while (current != goal) //if agent don't reach the goal
	{
		counter++;	
		Forward_ComputePath(adapt);  //calculate the path
		if (Openlist.is_empty())  //if there is no way to get the goal
		{
			cout << "The Agent Can't Reach the Target." << endl;
			return;
		}
		else  //if there is still some probabilities to reach the goal
		{
			for (int cnt = path.size() - 1; cnt > 0 && current == path[cnt] && GridWorld[path[cnt - 1].first][path[cnt - 1].second] == 1; cnt--)
			{
				current = path[cnt - 1];
				int row = current.first;
				int col = current.second;
				//set grid as observed
				AgentWorld[row][col].set_observe();
				if (row - 1 > 0)
					AgentWorld[row - 1][col].set_observe();
				if (row + 1 < MAZESIZE)
					AgentWorld[row + 1][col].set_observe();
				if (col - 1 > 0)
					AgentWorld[row][col - 1].set_observe();
				if (col + 1 < MAZESIZE)
					AgentWorld[row][col + 1].set_observe();
			}
		}
	}
	cout << "The Agent has already Reach the Target." << endl;
}

void Move::AgentBackward()
{
	current = start;
	Openlist.clear();
	Openlist.insert(AgentWorld[goal.first][goal.second]);
	
	while (current != goal) //if agent don't reach the goal
	{
		counter++;
		Backward_ComputePath();  //calculate the path
		if (Openlist.is_empty())  //if there is no way to get the goal
		{
			cout << "The Agent Can't Reach the Target." << endl;
			return;
		}
		else  //if there is still some probabilities to reach the goal
		{
			B_AgentWorld[current.first][current.second].set_g(0);
			B_AgentWorld[current.first][current.second].update_f();
			if (path.size() > 1)
			{
				for (int cnt = 0; cnt<path.size()-1 && current == path[cnt] && GridWorld[path[cnt + 1].first][path[cnt + 1].second] == 1; cnt++)
				{
					current = path[cnt+1];
					int row = current.first;
					int col = current.second;
					//set grid as observed
					B_AgentWorld[row][col].set_observe();
					if (row - 1 > 0)
						B_AgentWorld[row - 1][col].set_observe();
					if (row + 1 < MAZESIZE)
						B_AgentWorld[row + 1][col].set_observe();
					if (col - 1 > 0)
						B_AgentWorld[row][col - 1].set_observe();
					if (col + 1 < MAZESIZE)
						B_AgentWorld[row][col + 1].set_observe();
				}
			}
		}
	}
	cout << "The Agent has already Reach the Target." << endl;
}

int Move::ManhattanDistance(Node x,pair<int,int> p)
{
		return (abs(x.get_position().first - p.first) + abs(x.get_position().second - p.second));
}

void Move::Forward_ComputePath(bool adapt)
{
	Openlist.clear();
	Openlist.insert(AgentWorld[current.first][current.second]);
	AgentWorld[goal.first][goal.second].set_g(INT_MAX);
	while (AgentWorld[goal.first][goal.second].get_g() >= Openlist.extractMin().get_f())   //if g(goal)>f(s) s is the top of openlist
	{
		if (Openlist.is_empty()) //if open list is empty or find the goal
			break;
		else if (Openlist.peekTop().get_position() == goal)
		{
			path.clear();
			path.push_back(goal);
			int row = goal.first;
			int col = goal.second;
			while (row != current.first || col != current.second)
			{
				pair<int, int> p = AgentWorld[row][col].get_from();
				row = p.first;
				col = p.second;
				path.push_back(p);
			}
			break;
		}
		else
		{
			Node s = Openlist.peekTop();
			int lastRow = s.get_position().first;
			int lastCol = s.get_position().second;

			// expand state
			Openlist.remove(s);
			Closelist.push_back(s);

			// four possible directions: up,down,left,right
			const int dirrow[] = { -1,1,0,0 };//up and down
			const int dircol[] = { 0,0,-1,1 };//left and right

			for (int i = 0; i < 4; i++)
			{
				int dirR = dirrow[i];
				int dirC = dircol[i];
				int curRow = lastRow + dirR;
				int curCol = lastCol + dirC;

				//check if valid
				if (curRow < 0 || curRow >= MAZESIZE || curCol < 0 || curCol >= MAZESIZE
					|| (GridWorld[curRow][curCol] == 0 && (AgentWorld[curRow][curCol].get_observe() || ManhattanDistance(AgentWorld[curRow][curCol], current) == 1)))
					continue;
				if (AgentWorld[curRow][curCol].get_search() < counter)
				{
					AgentWorld[curRow][curCol].set_g(INT_MAX);
					AgentWorld[curRow][curCol].set_search(counter);
				}
				if (AgentWorld[curRow][curCol].get_g() > AgentWorld[lastRow][lastCol].get_g() + 1)
				{
					AgentWorld[curRow][curCol].set_g(AgentWorld[lastRow][lastCol].get_g() + 1); //update g(s) of current position
					int index = Openlist.FindbyPosition(pair<int, int>(curRow, curCol));
					if (index != -1)   //if the grid has been expanded then remove it
						Openlist.remove(index);
					if(adapt&&Closelist.size()!=0)
						AgentWorld[curRow][curCol].set_h(AgentWorld[curRow][curCol].get_h_new());
					else
						AgentWorld[curRow][curCol].set_h(ManhattanDistance(AgentWorld[curRow][curCol], goal));
					AgentWorld[curRow][curCol].update_f();
					AgentWorld[curRow][curCol].set_from(s.get_position());
					Openlist.insert(AgentWorld[curRow][curCol]);  //insert current position to the openlist
				}
			}
		}
	}
	if (adapt)
	{
		vector<Node>::iterator itr;
		for (itr = Closelist.begin(); itr != Closelist.end(); itr++)
			AgentWorld[itr->get_position().first][itr->get_position().second].set_h_new(AgentWorld[goal.first][goal.second].get_g() - AgentWorld[itr->get_position().first][itr->get_position().second].get_g());
	}
}

void Move::Backward_ComputePath()
{
	B_AgentWorld[current.first][current.second].set_g(INT_MAX);
	B_AgentWorld[current.first][current.second].set_from(pair<int,int>(current.first, current.second));
	B_AgentWorld[current.first][current.second].update_f();
	Openlist.clear();
	Openlist.insert(AgentWorld[goal.first][goal.second]);
	while (B_AgentWorld[current.first][current.second].get_g() >= Openlist.extractMin().get_f())   //if g(goal)>f(s) s is the top of openlist
	{
		if (Openlist.is_empty()) //if open list is empty or find the goal
			break;
		else if (Openlist.peekTop().get_position() == current)
		{
			path.clear();
			path.push_back(current);
			int row = current.first;
			int col = current.second;
			while (row != goal.first || col != goal.second)
			{
				pair<int, int> p = B_AgentWorld[row][col].get_from();
				row = p.first;
				col = p.second;
				path.push_back(p);
			}
			break;
		}
		else
		{
			Node s = Openlist.peekTop();
			int lastRow = s.get_position().first;
			int lastCol = s.get_position().second;

			// expand state
			Openlist.remove(s);
			Closelist.push_back(s);

			// four possible directions: up,down,left,right
			int dirrow[] = { -1,1,0,0 };//up and down
			int dircol[] = { 0,0,-1,1 };//left and right

			for (int i = 0; i < 4; i++)
			{
				int dirR = dirrow[i];
				int dirC = dircol[i];
				int curRow = lastRow + dirR;
				int curCol = lastCol + dirC;

				//check if valid
				if (curRow < 0 || curRow >= MAZESIZE || curCol < 0 || curCol >= MAZESIZE
					|| (GridWorld[curRow][curCol] == 0 && (B_AgentWorld[curRow][curCol].get_observe() || ManhattanDistance(B_AgentWorld[curRow][curCol], current) == 1)))
					continue;
				if (B_AgentWorld[curRow][curCol].get_search() < counter)
				{
					B_AgentWorld[curRow][curCol].set_g(INT_MAX);
					B_AgentWorld[curRow][curCol].set_search(counter);
				}
				if (B_AgentWorld[curRow][curCol].get_g() > B_AgentWorld[lastRow][lastCol].get_g() + 1)
				{
					B_AgentWorld[curRow][curCol].set_g(B_AgentWorld[lastRow][lastCol].get_g() + 1); //update g(s) of current position
					int index = Openlist.FindbyPosition(pair<int, int>(curRow, curCol));
					if (index != -1)   //if the grid has been expanded then remove it
						Openlist.remove(index);
					B_AgentWorld[curRow][curCol].set_h(ManhattanDistance(B_AgentWorld[curRow][curCol], current));
					B_AgentWorld[curRow][curCol].update_f();
					B_AgentWorld[curRow][curCol].set_from(s.get_position());
					Openlist.insert(B_AgentWorld[curRow][curCol]);  //insert current position to the openlist			
				}
			}
		}
	}
}

