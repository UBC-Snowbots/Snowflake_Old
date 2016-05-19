/** 
 * Pathfinding prototype
 * Needs: OccupancyGrid, Pose2D initial_location, Pose2D final_location
 * Returns: A point (x,y) that brings you one step closer from initial to final location
 * Author: Valerian Ratu
 *
 * Subscribes to: Pose2D x 2, Occupancy Grid
 * Publishes to: Point waypoint
 * 
 */


//STD LIBS
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <string>

//ROS MESSAGES
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>

//ROS MAIN LIBS
#include <ros/ros.h>

//The number which we consider to be a minimum for an obstacle
#define OCCUPANCY_THRESHOLD 0

//The movement cost of moving a tile
//TODO: Optimize maybe with robot direction and giving a tile relative cost depening on that
#define MOVEMENT_COST_STR 10
#define MOVEMENT_COST_DIAG 14

using namespace std;

nav_msgs::OccupancyGrid g_map;
geometry_msgs::Pose2D g_start;
geometry_msgs::Pose2D g_end;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	g_map.header = msg->header;
	g_map.info = msg->info;
	g_map.data = msg->data;
}

void poseStartCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	g_start.x = msg->x;
	g_start.y = msg->y;
	g_start.theta = msg->theta;
}

void poseEndCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	g_end.x = msg->x;
	g_end.y = msg->y;
	g_end.theta = msg->theta;
}


/**
 * A node structure
 */
struct node_t {

public: 
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
class compare
{
public:
	bool operator() (node_t* first_node, node_t* second_node)
	{
		return (first_node->f) > (second_node->f);
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
node_t *pop(vector<node_t*>& heap)
{
	node_t *curr_node = heap.front();
	pop_heap(heap.begin(), heap.end(), compare());
	heap.pop_back();
	return curr_node;
}


/**
 * Returns true if the node is in the list
 * @param  node_list the list which may or may not contain list
 * @param  node      the node being searched for
 * @return           true if the node is found, false otherwise
 */
bool findNodeInList(vector<node_t*>& node_list, node_t *node)
{
	for (int i = 0; i < node_list.size(); i++){
		if ((node_list[i]->x == node->x) && (node_list[i]->y == node->y)) return true;
	}
	return false;
}

/**
 * Returns a string which represents the node in (x,y) format
 * @param  node the pointer to the node
 * @return      see above
 */
string printNode(node_t *node){
	ostringstream words;
	words << "(" << node->x << "," << node->y << ")";
	return words.str();
}


/**
 * Updates the node list if the node has a lower f value than before
 * @param  node_list the list containing the nodes
 * @param  node      the updated node
 * @return           true if the node was modified, false otherwise
 */
bool updateList(vector<node_t*>& node_list, node_t *node, node_t *parent_node)
{
	for (int i = 0; i < node_list.size(); i++)
	{
		if ((node_list[i]->x == node->x) && (node_list[i]->y == node->y)){
			if (node_list[i]->f > node->f)
			{
				node_t* deprecated_node = node_list[i];
				node_list[i] = node;
				delete deprecated_node;
				node_list[i]->parent = parent_node;
				//cout << "In update list: Child " << printNode(node) << "Parent " << printNode(parent_node) << endl;
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
int get_man_distance(node_t *start_point, node_t *end_point)
{
	return (abs(start_point->x-end_point->x) + abs(start_point->y-end_point->y));
}

/**
 * Prints the nodes contained in a list
 * @param node_list the vector containing the node pointers
 * @param name      name of the list
 */
void printNodeList(vector<node_t*>& node_list, string name)
{
	cout << "Printing list: " << name << endl;
	for (int j = 0; j < node_list.size(); j ++)
	{
		cout << "Node at: " << printNode(node_list[j]) << " with f: " << node_list[j]->f  << " and parent: " << printNode(node_list[j]->parent) << endl;
	}

}

/**
 * Deallocates all the nodes in a list
 * @param node_list the list containing nodes
 */
void freeList(vector<node_t*>& node_list){
	for (int i = 0; i < node_list.size(); i++){
		delete node_list[i];
	}
}

/**
 * Prints an array of integers
 * Used for testing purposes
 * @param map    the array
 * @param width  the width of the array
 * @param height the height of the array
 */
void printArray(int** map, int width, int height){
	cout << "\t";
	for (int j = 0; j < width; j++){
			cout << j << "\t";
	}
	cout << endl;
	for (int i = 0; i < height; i++){
		cout << i << ": " << "\t";
		for (int j = 0; j < width; j++){
			cout << map[i][j] << "\t";
			
		}
		cout << endl;
	}
}


/**
 * Walks back through a node_list to find the path from start_point to end_point
 * @param node_list		the list containing the all the nodes between start_point and end_point
 * @param start_point 	the node we are starting at
 * @param end_point 	the node we are ending at
 * @return 				a vector containing the node list in reverse order:= [end_point,..., start_point]
 */
vector<node_t*> traceback(vector<node_t*>& node_list, node_t *start_point, node_t *end_point){ 
	int i = 0;
	vector<node_t*> trace;
	node_t *current_node;
	do{
		current_node = node_list[i];
		i++;
	} while (!((current_node->x == end_point->x) && (current_node->y == end_point->y)));
	//cout << "Found proper node: " << printNode(terminal) << endl;
	trace.push_back(current_node);
	while (!((current_node->x == start_point->x) && (current_node->y == start_point->y)))
	{
		//cout << "Child: " << printNode(current_node) << endl;
		current_node = current_node->parent;
		trace.push_back(current_node);

		//cout << "Parent: " << printNode(current_node) << endl;	
	}
	return trace;
}

/**
 * Returns the movement cost to an adjacent tile
 * @param  start the tile we're on 
 * @param  end   the tile we're moving to
 * @return       the movement cost
 */
int getMovementCost(node_t *start, node_t* end){
	if (get_man_distance(start, end) > 1){
		return MOVEMENT_COST_DIAG;
	} 
	return MOVEMENT_COST_STR;
}

/**
 * Returns the translated pose in the frame of the occupancy grid
 * @param  map  the occupancy grid
 * @param  pose the untranslated pose
 * @return      the translated pose
 */
geometry_msgs::Pose2D poseRealToMapTranslator(nav_msgs::OccupancyGrid map, geometry_msgs::Pose2D pose){
	geometry_msgs::Pose2D translated_pose;
	translated_pose.x = (pose.x - map.info.origin.position.x)/map.info.resolution;
	translated_pose.y = (pose.y - map.info.origin.position.y)/map.info.resolution;
	return translated_pose;
}


geometry_msgs::Pose2D poseMapToRealTranslator(nav_msgs::OccupancyGrid map, geometry_msgs::Pose2D pose){
	geometry_msgs::Pose2D translated_pose;
	translated_pose.x = pose.x + map.info.origin.position.x;
	translated_pose.y = pose.y + map.info.origin.position.y;
	return translated_pose;
}

/**
 * Want to return both the waypoint and the path
 */
struct pathfinding_info {
public:	
	geometry_msgs::Point waypoint;
	nav_msgs::Path path;
};



/**
 * Gets the next waypoint to ge to the target position
 * @param  map              an occupancy grid containing the map
 * @param  current_position the current (x,y) position
 * @param  target_position  the desired (x,y) position
 * @return                  the next (x,y) position we want to be in to get to the target position 
 */
pathfinding_info get_next_waypoint(	nav_msgs::OccupancyGrid map, 
										geometry_msgs::Pose2D current_position, 
										geometry_msgs::Pose2D target_position)
{

	int width = map.info.width;
	int height = map.info.height;

	vector<node_t*> open_list;
	vector<node_t*> closed_list;

	node_t *starting_point = new node_t();
	geometry_msgs::Pose2D current_trans = poseRealToMapTranslator(map, current_position);
	starting_point->x = current_trans.x;
    ROS_INFO("Starting point x: %f", current_position.x);
    ROS_INFO("Starting point y:  %f" ,current_position.y);
    ROS_INFO("Starting point trans x: %f", current_trans.x);
    ROS_INFO("Starting point trans y:  %f", current_trans.y);
	starting_point->y = current_trans.y;
	starting_point->parent = starting_point;

	node_t *end_goal = new node_t();
	geometry_msgs::Pose2D target_trans = poseRealToMapTranslator(map, target_position);
	end_goal->x = target_trans.x;
	end_goal->y = target_trans.y;
    ROS_INFO("End point x: %f", target_trans.x);
    ROS_INFO("End point y:  %f", target_trans.y);

	starting_point->g = 0;
	starting_point->f = 0;
	starting_point->h = 0;

	int** new_map = new int *[height];
	for (int i = 0; i < height; i++){
		new_map[i] = new int[width];
	}

	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			new_map[i][j] = map.data[i*width + j];
		}
	}

	push(open_list, starting_point);
	
	//A star
	while (!open_list.empty())
	{
		node_t *curr_node = pop(open_list);
		push(closed_list, curr_node);

		//Iterate through all neighbouring nodes and analyze all valid ones
		for (int x = curr_node->x - 1; x <= curr_node->x + 1; x++){
			if ((x < 0) || (x >= map.info.width)){
				//cout << "x: " << x << "didn't pass param" << endl;
				continue;
			} 
			for (int y = curr_node->y - 1; y <= curr_node->y + 1; y++){
				if ((y < 0) || (y >= map.info.height)){
					//cout << "y: " << y << " didn't pass param" << endl;
					continue;
				}
				//don't analyze the current node
				if ((x == curr_node->x) && (y == curr_node->y)){
					//cout << "(x,y) of analyzed node: (" << x << "," << y << ")" << endl;
					continue;
				}
				//Skip if there's an object here
				if ((int) map.data[y*width + x] > OCCUPANCY_THRESHOLD) {
					//cout << "Occupancy at (" << x << "," << y << ") with: " << (int) map.data[x*width + y] << endl;
					continue;
				}
				node_t *child = new node_t();
				child->x = x;
				child->y = y;

				child->g = curr_node->g + getMovementCost(curr_node, child);

				child->h = get_man_distance(child, end_goal);
				child->f = child->g + child->h;
				//Goto is disgusting but we're in a heavily nested loop
				if ((end_goal->x == x) && (end_goal->y == y)){
					//cout << "Have reached end goal with (" << x << "," << y << ")" << endl;
					child->parent = curr_node;
					push(closed_list, child);
					goto end;
				}

				if (findNodeInList(open_list, child)){
					updateList(open_list, child, curr_node);
				//} else if(findNodeInList(closed_list, child)) {
					//updateList(closed_list, child, curr_node);
				} else {
					child->parent = curr_node;
					//cout << "Pushing open list: Child " << printNode(child) << "Parent " << printNode(curr_node) << endl;
					push(open_list, child);
				}
			}
		}
		
	}

	end: 
	//printNodeList(open_list, "Open list");
	//printNodeList(closed_list, "Closed list");
	//printArray(new_map, width, height);
	vector<node_t*> trace = traceback(closed_list, starting_point, end_goal);
	//printNodeList(trace, "Trace list");
	//Constructs the point given
	geometry_msgs::Point waypoint;	
	nav_msgs::Path *path = new nav_msgs::Path();
	vector<geometry_msgs::PoseStamped> pose_init; 

	//Output the path
	for (int i = 0; i < trace.size(); i++){
		geometry_msgs::PoseStamped *position = new geometry_msgs::PoseStamped();
		position->pose.position.x = trace[trace.size() - 1 - i]->x;
		position->pose.position.y = trace[trace.size() - 1 - i]->y;
		position->pose.position.z = 0;

		pose_init.push_back(*position);
	}
	path->poses = pose_init;

	//If the only item is the end point i.e. current position == target position
	if (trace.size() == 1){
		waypoint.x = trace[0]->x;
		waypoint.y = trace[0]->y;
	} else {
		//Take the second last element, the one after initial point
		waypoint.x = trace[trace.size() - 2]->x;
		waypoint.y = trace[trace.size() - 2]->y;
	}
    
    waypoint.x = (waypoint.x * map.info.resolution + map.info.origin.position.x);
    waypoint.y = (waypoint.y * map.info.resolution + map.info.origin.position.y);
	waypoint.z = 0;
	delete end_goal;
	//Frees the memory
	freeList(open_list);
	freeList(closed_list);

	pathfinding_info *path_info = new pathfinding_info();
	path_info->waypoint = waypoint;
	path_info->path = *(path);

	return *(path_info);
}

int main(int argc, char** argv){

	const string init_pose_topic = "initial_pose";
	const string final_pose_topic = "final_pose";
	const string occ_grid_topic = "occupancy_grid";
	const string point_output_topic = "waypoint";
	const string path_output_topic = "path";
	const string node_name = "pathfinding_node";


	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
    ROS_INFO("DIDNT DIE YET");
	ros::Subscriber poseStartSub = nh.subscribe(init_pose_topic, 10, poseStartCallback);
	ros::Subscriber poseEndSub = nh.subscribe(final_pose_topic, 10, poseEndCallback);
	ros::Subscriber occGridSub = nh.subscribe(occ_grid_topic, 5, mapCallback);
	ros::Publisher pointPub = nh.advertise<geometry_msgs::Point>(point_output_topic, 1);
	ros::Publisher pathPub = nh.advertise<nav_msgs::Path>(path_output_topic, 1);
	ros::Rate loop_rate(5);

	//initialize in order to avoid segfaults
	g_start.x = 0;
	g_start.y = 0;
	g_start.theta = 0;
	g_end.x = 0;
	g_end.y = 0;
	g_end.theta = 0;
	geometry_msgs::Pose origin;
	origin.position.x = 0;
	origin.position.y = 0;
	origin.position.z = 0;
	origin.orientation.x = 0.0;
	origin.orientation.y = 0.0;
	origin.orientation.z = 0.0;
	origin.orientation.w = 0.0;
	g_map.info.resolution = 0.25;
	g_map.info.width = 20;
	g_map.info.height = 20;
	g_map.info.origin = origin;
	for (int i = 0; i < 20*20; i++){
		g_map.data.push_back(0);
	}
    ROS_INFO("Got in to the main loop!");
	while (nh.ok()){
		pathfinding_info path_info = get_next_waypoint(g_map, g_start, g_end);
		pointPub.publish(path_info.waypoint);
		pathPub.publish(path_info.path);
		loop_rate.sleep();
		ros::spinOnce();
	}
}




/*
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

	//Make a barrier (Test Case 1)
	
	for (int i = 2; i < 19; i++){
		map.data[18*map.info.width+i] = 10;
		map.data[map.info.width*i + 18] = 10;
	}
	

	//Test Case 2
	for (int i = 2; i < 8; i++){
		map.data[map.info.width*i + 7] = 10;
	}

	for (int i = 9; i < 18; i++){
		map.data[map.info.width*i + 7] = 10;
	}

	for (int i = 1; i < 5; i++){
		map.data[map.info.width*i + 14] = 10;
	}

	for (int i = 7; i < 20; i++){
		map.data[map.info.width*i + 14] = 10;
	}

	//Initial and final position
	geometry_msgs::Pose2D curr_pos;
	curr_pos.x = 1;
	curr_pos.y = 8;
	curr_pos.theta = 0;
	geometry_msgs::Pose2D target_pos;
	target_pos.x = 18;
	target_pos.y = 18;
	target_pos.theta = 0;
	cout << "In main" << endl;
	geometry_msgs::Point waypoint = get_next_waypoint(map, curr_pos, target_pos);
	cout << "X returned: " << waypoint.x << endl;
	cout << "Y returned: " << waypoint.y << endl;
	return 0;
}
*/