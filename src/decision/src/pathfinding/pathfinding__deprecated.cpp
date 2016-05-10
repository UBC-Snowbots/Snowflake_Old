#include <stdio.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <cmath>
#include <algorithm>
#include <string>

#define OCCUPANCY_THRESHOLD 0
#define MOVEMENT_COST 10

using namespace std;

/**
 * A node structure
 */
struct node_t {

	//Location on the grid
	int x;
	int y;

	//A* values
	int g;
	int f;
	int h;

	//Parent node
	node_t *parent;

};


/**
 * A comparator for the min heap
 */
struct compare
{
	bool operator() (node_t& first_node, node_t& second_node)
	{
		return first_node.f > second_node.f;
	}
};

/**
 * Pushes a node into the min heap
 * @param heap     the heap
 * @param new_node the node
 */
void push(vector<node_t*>& heap, node_t* new_node)
{
	heap.push_back(new_node);
	push_heap(heap.begin(), heap.end(), compare());
	return;
}

/**
 * Pops the minimum node from the heap
 * @param  heap the heap
 * @return      the node contaning the smallest value
 */
node_t *pop(vector<node_t>& heap)
{
	node_t *curr_node = heap.front();
	pop_heap(heap.begin(), heap.end(), compare());
	heap.pop_back();
	return curr_node;

}

/**
 * Returns true if the node is in the list
 * @param  node_list [description]
 * @param  node      [description]
 * @return           [description]
 */
bool findNodeInList(vector<node_t>& node_list, node_t *node)
{
	for (int i = 0; i < node_list.size(); i++){
		if ((node_list[i].x == node.x) && (node_list[i].y == node.y)) return true;
	}
	return false;
}


string printNode(node_t node){
	ostringstream words;
	words << "(" << node.x << "," << node.y << ")";
	return words.str();
}


/**
 * Updates the node list if the node has a lower f value than before
 * @param  node_list the list containing the nodes
 * @param  node      the updated node
 * @return           true if the node was modified, false otherwise
 */
bool updateList(vector<node_t>& node_list, node_t node, node_t parent_node)
{
	for (int i = 0; i < node_list.size(); i++)
	{
		if ((node_list[i].x == node.x) && (node_list[i].y == node.y)){
			if (node_list[i].f > node.f)
			{
				node_list[i] = node;
				node_list[i].parent = &parent_node;
				cout << "In update list: Child " << printNode(node) << "Parent " << printNode(parent_node) << endl;
				push_heap(node_list.begin(), node_list.end(), compare());
				return true;
			}
		}
	}
	return false;
}

/**
 * Returns the manhattan distance between two nodes
 * @param  start_point the first node
 * @param  end_point   second node
 * @return             an integer representing the manhattan distance between
 *                        the two nodes
 */
int get_man_distance(node_t start_point, node_t end_point)
{
	return (abs(start_point.x-end_point.x) + abs(start_point.y-end_point.y));
}

void printNodeList(vector<node_t>& node_list, string name)
{
	cout << "Printing list: " << name << endl;
	for (int j = 0; j < node_list.size(); j ++)
	{
		cout << "Node at: " << printNode(node_list[j]) << "with f:" <<node_list[j].f  << "and parent: " << printNode(*node_list[j].parent)<< endl;
	}

}

void printArray(int** map, int width, int height){
	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			cout << map[i][j] << "\t";
		}
		cout << endl;
	}
}

vector<node_t> traceback(vector<node_t>& node_list, node_t start_point, node_t end_point){
	int i = 0;
	vector<node_t> trace;
	node_t terminal;
	do{
		terminal = node_list[i];
		i++;
	} while ((terminal.x != end_point.x) && (terminal.y != end_point.y));
	cout << "Found proper node" << endl;
	trace.push_back(terminal);
	node_t current_node = *(terminal.parent);
	do{
		cout << "Child: " << printNode(current_node) << endl;

		trace.push_back(current_node);
		current_node = *(current_node.parent);
		cout << "Parent: " << printNode(current_node) << endl;	
	} while ((current_node.x != start_point.x) && (current_node.y != start_point.y));
	return trace;
}


geometry_msgs::Point get_next_waypoint(	nav_msgs::OccupancyGrid map, 
										geometry_msgs::Pose2D current_position, 
										geometry_msgs::Pose2D target_position)
{
	geometry_msgs::Point waypoint;
	int width = map.info.width;
	int height = map.info.height;

	//keeps track of the cost of nodes for trackback
	//vector<int> cost (width * height, -1);
	vector<node_t*> open_list;
	vector<node_t*> closed_list;
	int **f_values;
	f_values = new int *[height];
	for (int i = 0; i < height; i++){
		f_values[i] = new int[width];
	}

	node_t starting_point;
	starting_point.x = current_position.x;
	starting_point.y = current_position.y;

	node_t end_goal;
	end_goal.x = target_position.x;
	end_goal.y = target_position.y;

	starting_point.g = 0;
	starting_point.f = 0;
	starting_point.h = 0;

	//cost[starting_point.x*width + starting_point.y] = starting_point.f;
	push(open_list, &starting_point);
	/*while (!open_list.empty())
	{
		node_t curr_node = pop(open_list);
		for (int x = curr_node.x - 1; x <= curr_node.x + 1; x++){
			cout << "X value: " << x << endl;
			if ((x < 0) || (x > map.info.width)){
				cout << "x: " << x << "didn't pass param" << endl;
				continue;
			} 
			for (int y = curr_node.y - 1; y <= curr_node.y + 1; y++){
				if ((y < 0) || (y > map.info.height)){
					cout << "y: " << y << " didn't pass param" << endl;
					continue;
				}
				//don't analyze the node again
				if ((x == curr_node.x) && (y == curr_node.y)){
					cout << "(x,y) of analyzed node: (" << x << "," << y << ")" << endl;
					continue;
				}

				if ((int) map.data[x*width + y] > OCCUPANCY_THRESHOLD) {
					cout << "Occupancy at (" << x << "," << y << ") with: " << (int) map.data[x*width + y] << endl;
					continue;
				}
				node_t child;
				child.x = x;
				child.y = y;

				child.g = curr_node.g + MOVEMENT_COST;
				child.h = get_man_distance(child, end_goal);
				child.f = child.g + child.h;
				//Goto is disgusting but we're in a heavily nested loop
				if ((end_goal.x == x) && (end_goal.y == y)){
					cout << "Have reached end goal with (" << x << "," << y << ")" << endl;
					child.parent = &curr_node;
					push(closed_list, child);
					goto end;
				}

				if (findNodeInList(open_list, child)){
					updateList(open_list, child, curr_node);
				} else if(findNodeInList(closed_list, child)) {
					updateList(closed_list, child, curr_node);
				} else {
					child.parent = curr_node;
					cout << "Pushing open list: Child " << printNode(child) << "Parent " << printNode(curr_node) << endl;
					push(open_list, child);
				}
				f_values[child.x][child.y] = child.f;
			}
		}
		push(closed_list, curr_node);
	}
	//Traceback here
	
	end:
	cout << "end" << endl;
	printNodeList(open_list, "Open list");
	printNodeList(closed_list, "Closed list");
	printArray(f_values, width, height);
	//vector<node_t> trace = traceback(closed_list, starting_point, end_goal);
	//printNodeList(trace, "Trace list");
	*/
}






int main(){

	//Creating occupancy grid
	geometry_msgs::Pose origin;
	origin.position.x = 0;
	origin.position.y = 0;
	origin.position.z = 0;
	origin.orientation.x = 0.0;
	origin.orientation.y = 0.0;
	origin.orientation.z = 0.0;
	origin.orientation.w = 0.0;
	nav_msgs::OccupancyGrid map;
	map.info.resolution = 0.25;
	map.info.width = 20;
	map.info.height = 20;
	map.info.origin = origin;

	//create a sparse 100 x 100 map with nothing in it
	for (int i = 0; i < 20*20; i++){
		map.data.push_back(0);
	}

	//Initial and final position
	geometry_msgs::Pose2D curr_pos;
	curr_pos.x = 2;
	curr_pos.y = 2;
	curr_pos.theta = 0;
	geometry_msgs::Pose2D target_pos;
	target_pos.x = 4;
	target_pos.y = 4;
	target_pos.theta = 0;

	geometry_msgs::Point waypoint = get_next_waypoint(map, curr_pos, target_pos);
	return 0;
}
