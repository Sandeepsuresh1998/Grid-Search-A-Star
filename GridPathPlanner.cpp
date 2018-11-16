#include "GridPathPlanner.h"
#include <set>
#include <iostream>
#include <queue>
#include <map>
#include <algorithm>

using namespace std;


GridPathPlanner::GridPathPlanner(PartiallyKnownGrid* grid_, xyLoc destination_, bool adaptive_, bool larger_g_) {
	grid = grid_;
	destination = destination_;
	adaptive = adaptive_;
	larger_g = larger_g_;
	num_expansions = 0;
}

GridPathPlanner::~GridPathPlanner(){
}

  
void GridPathPlanner::FindPath(xyLoc start, std::vector<xyLoc> & path) {
	cout << "Called" << endl;
	
	num_expansions = 0;

	// TODO
	// Possible flow:
	// - Initialize data structures / open list
  // - Search until goal is selected for expansion
  // - Extract path
  // - Update heuristic if adaptive

	//Create Grid to hold xyLoc Holder struct. All information can come from here.
	xyLocHolder grid_info[grid->GetWidth()][grid->GetHeight()];	

	//Creating the closed set
	std::set<xyLoc> closed_set;

	//Creating the open set with pairs of f-value and Locations
	priority_queue< pair< int, xyLoc > , vector< pair< int, xyLoc> >, greater< pair< int, xyLoc > > > open_set;


	open_set.push(make_pair(GetHValue(start) + 0, start));
	closed_set.insert(start);
	grid_info[start.x][start.y] = xyLocHolder(kInvalidXYLoc, 0, GetHValue(start), start);


	//Keep iterating until we have no more nodes to explore. 
	//If we find the goal state we break.
	while(!open_set.empty()) {
		//Let's explore the first element in queue and clear it from the open set
		xyLoc current = (open_set.top()).second; 
		open_set.pop();

		//We found goal state
		if(current == destination) {
			path.insert(path.begin(), current);

			//Creating the path
			while(grid_info[current.x][current.y].parent != start) {
					
				//Add where the cell came from
				path.insert(path.begin(), grid_info[current.x][current.y].parent);

				//Update current
				current = grid_info[current.x][current.y].parent;

				if(current == start) {
					cout << "WHOA" << endl;
				}
			}

			//Add the starting cell
			path.insert(path.begin(), start);


			return;

		}

		//Let's explore all neighbors of the current loc. 
		vector<xyLoc> neighbors; 
		//Left
		neighbors.push_back(xyLoc(current.x - 1, current.y));
		
		//Down
		neighbors.push_back(xyLoc(current.x, current.y + 1));
		
		//Right
		neighbors.push_back(xyLoc(current.x + 1, current.y));
	
		//Up
		neighbors.push_back(xyLoc(current.x, current.y - 1));
		

		//Iterate through the neighbors and add them to the lists
		for(int i = 0; i < neighbors.size(); i++) {
			// If it's not a valid location skip it
			if(!(grid->IsValidLocation(neighbors[i]))) {
				continue;
			} else {

				//If location is blocked
				if(!(grid->IsBlocked(neighbors[i]))) {
					//If we spot hasn't been searched before, let's add it to the list
					if(closed_set.find(neighbors[i]) == closed_set.end()) {
						//Add loc to open and closed set				
						//Adding the neighbors to the grid info
						int g_value = grid_info[current.x][current.y].g_val + 1;
						open_set.push(make_pair(g_value + GetHValue(neighbors[i]), neighbors[i]));
						closed_set.insert(neighbors[i]);
						num_expansions += 1;

						//Updating Grid info
						grid_info[neighbors[i].x][neighbors[i].y].parent = current;
						grid_info[neighbors[i].x][neighbors[i].y].g_val = g_value;
						grid_info[neighbors[i].x][neighbors[i].y].h_val = GetHValue(neighbors[i]);
					} else {
						closed_set.insert(neighbors[i]);
						continue;
					}
				} else {
					//Still add this to the closed set so we don't explore it again
					closed_set.insert(neighbors[i]);

				}
			}
		}

	}



}

int GridPathPlanner::GetHValue(xyLoc l) {
	// TODO


	if(adaptive) {
		//Return updated h-value of cell
		

	} else {
		//Return Manhattan Distance from location to destination
		return abs(l.x - destination.x) + abs(l.y - destination.y);
	}
	return 0;
}

int GridPathPlanner::GetNumExpansionsFromLastSearch() {
	
	return num_expansions;
}

