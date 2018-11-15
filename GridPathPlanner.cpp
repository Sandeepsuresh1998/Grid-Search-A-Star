#include "GridPathPlanner.h"
#include <set>
#include <queue>
#include <map>
#include <algorithm>

using namespace std;

GridPathPlanner::GridPathPlanner(PartiallyKnownGrid* grid_, xyLoc destination_, bool adaptive_, bool larger_g_) {
	grid = grid_;
	destination = destination_;
	adaptive = adaptive_;
	larger_g = larger_g_;
}

GridPathPlanner::~GridPathPlanner(){
}

  
void GridPathPlanner::FindPath(xyLoc start, std::vector<xyLoc> & path) {
	// TODO
	// Possible flow:
	// - Initialize data structures / open list
  // - Search until goal is selected for expansion
  // - Extract path
  // - Update heuristic if adaptive

	//Creating the closed set
	std::set<xyLoc> closed_set;

	//Creating the open set with pairs of f-value and Locations
	//This allows for sorting by smallest f-value
	priority_queue< pair< int, xyLoc > , vector< pair< int, xyLoc> >, greater< pair< int, xyLoc > > > open_set;
	open_set.push(make_pair(GetHValue(start) + 0, start)); //No g(s) bc it is the start node
	closed_set.insert(start);

	//Map to remember where each cell came from key = xyloc, value = where xyloc came from
	map < xyLoc, xyLoc > path_map;
	path_map.insert(make_pair(start, start));

	map< xyLoc, int > g_value_map;
	g_value_map.insert(make_pair(start, 0));

	//TODO: We need to be able to trace back the path, so having a map where each thing location came from would be useful

	//Keep iterating until we have no more nodes to explore. 
	//If we find the goal state we break.
	while(!open_set.empty()) {
		//Let's explore the first element in queue and clear it from the open set
		xyLoc current = (open_set.top()).second; 
		open_set.pop();

		cout << "X: " << current.x << " , Y: " << current.y << endl;

		//We found goal state
		if(current == destination) {
			cout << "Got to destination" << endl;
			path.push_back(current);

			//Creating the path
			while(path_map[current] != start) {
				//Add where the cell came from
				path.push_back(path_map[current]);

				//Update current
				current = path_map[current];
			}

			//Add the starting cell
			path.push_back(start);

			//We need to reverse this
			reverse(path.begin(), path.end());

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
						//Add G value to map
						g_value_map.insert(make_pair(neighbors[i], g_value_map[current] + 1));

						//Add loc to open and closed set				
						open_set.push(make_pair(GetHValue(neighbors[i]) + g_value_map[neighbors[i]], neighbors[i]));
						cout << "h-value: " << GetHValue(neighbors[i]) << ", ";
						cout << "g-value: " << g_value_map[neighbors[i]] << ", ";
						cout << "F-value: " <<  GetHValue(neighbors[i]) + g_value_map[neighbors[i]] << endl;
						closed_set.insert(neighbors[i]);
						path_map[neighbors[i]] = current;
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
	// TODO
	return 0;
}

