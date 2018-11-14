#include "GridPathPlanner.h"
#include <set>
#include <queue>

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
	open_set.push(make_pair(GetHValue(start), start));

	// //TODO: We need to be able to trace back the path, so having a map where each thing location came from would be useful


	//Keep iterating until we have no more nodes to explore. 
	//If we find the goal state we break.
	while(!open_set.empty()) {
		xyLoc current = (open_set.top()).second; //Let's explore the first element in queue 
		open_set.pop() // Clear it from open set
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

