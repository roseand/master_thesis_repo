#include <random>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "cassert"

#define MINUTE 60
#define MAX_DURATION 40 * MINUTE

// NOTE: will do the following:
// * make the solver not check the last vertex (cars disapear when leaving the road)
#define ROAD_EXPERIMENT true

#include "generator.h"
#include "data_csv.h"

struct GraphData
{
    Graph graph;
    std::vector<AgentInfo> agents;
};

static f32
get_makespan(std::vector<Path>* paths)
{
	// find the largest arrival time of any agent at its goal location
	f32 result = 0;

	for (Path path : *paths)
	{
		f32 new_cost = cost_path(path);
		if (new_cost > result)
		{
			result = new_cost;
		}
	}

	return result;
}

static f32
get_avg_makespan(std::vector<Path>* paths)
{
	// find the avg arrival time of all agents

	f32 total = cost(paths);
	f32 avg = total / paths->size();

	return avg;
}

void generate_maps_csv()
{
	Graph g;

	g = generate_highway_graph();
	write_map_csv(g, MapType::Highway);

	g = generate_exit_graph();
	write_map_csv(g, MapType::Exit);

	g = generate_roundabout_graph();
	write_map_csv(g, MapType::Roundabout);

	g = generate_intersection_graph();
	write_map_csv(g, MapType::Intersection);
}

void compute_lookup(Graph g)
{
	std::vector<std::vector<f32>> matrix;
	int size = g.vertices.size();

	for (int i = 0; i < size; i++)
	{
		// initialize for source node
		int source_node = i;

		std::vector<f32> distances; // distance from source to x
		std::vector<b32> visited;

		for (int j = 0; j < size; j++)
		{
			distances.push_back(INF);
			visited.push_back(false);
		}

		distances[source_node] = 0;
		b32 allVisited = false;

		while (!allVisited)
		{
			// find the nearest unvisited vertex
			// first time it is the source_node

			u32 nearestUnvisited;
			f32 nearestDistance = INF;
			bool noMoreNodestoReach = true;

			for (int j = 0; j < size; j++)
			{
				if (distances[j] < nearestDistance &&
					visited[j] == false)
				{
					nearestDistance = distances[j];
					nearestUnvisited = j;
					noMoreNodestoReach = false;
				}
			}

			if (noMoreNodestoReach) break;

			// for current vertex, visit unvisited neighbours
			for (Edge e : g.vertices[nearestUnvisited].edges)
			{
				u32 neighbour_node = e.to;
				if (visited[neighbour_node] == false)
				{
					f32 d = distance(g.vertices[nearestUnvisited].position,
									 g.vertices[neighbour_node].position);

					if (distances[neighbour_node] > distances[nearestUnvisited] + d)
					{
						distances[neighbour_node] = distances[nearestUnvisited] + d;
					}
				}
			}

			visited[nearestUnvisited] = true;

			// check if all nodes visited
			for (b32 v : visited)
			{
				if (v == false)
				{
					allVisited = false;
					break;
				}
				else
				{
					allVisited = true;
				}
			}
		}

		matrix.push_back(distances);
	}

	f32 topSpeed = 10;
	for (u32 i = 0; i < size; i++)
	{
		for (u32 j = 0; j < size; j++)
		{
			matrix[i][j] /= topSpeed;
		}
	}

	distanceLookup = matrix;
}







void exp1_generate_train_data(DataType data_type)
{
  //std::vector<MapType> maps = { MapType::Highway, MapType::Exit, MapType::Roundabout, MapType::Intersection };
	std::vector<MapType> maps = { MapType::Highway };

	std::vector<std::vector<f32>> set_of_speeds = { {0, 10}, { 0, 3.3, 6.6, 10 } };

	u32 min_agents = 8;
	u32 max_agents = 10;
	int n_instances = 100; // change!
	int counter = 1; // change!

	std::vector<std::string> cts = { "A", "B" };
	std::vector<f32> total_costs = { 0,0,0 }; // class A, class B, best

	std::string log_path = "../data_exp1/";

	std::string d;
	switch (data_type)
	{
	case DataType::train:
		d = "train_";
		break;
	case DataType::validation:
		d = "validation_";
		break;
	}

	std::string file_path = log_path + "exp1_" + d + "log.txt"; // change!

	std::ofstream file;
	file.open(file_path);

	for (MapType map : maps)
	{
		Graph g;
		std::string map_name;

		switch (map)
		{
		case MapType::Highway:
			g = generate_highway_graph();
			map_name = "h";
			break;
		case MapType::Exit:
			g = generate_exit_graph();
			map_name = "e";
			break;
		case MapType::Roundabout:
			g = generate_roundabout_v2_graph();
			map_name = "r";
			break;
		case MapType::Intersection:
			g = generate_intersection_graph();
			map_name = "i";
			break;
		case MapType::Entrance:
			g = generate_entrance_graph();
			map_name = "n";
			break;
		}

		compute_lookup(g);

		for (u32 n_agents = min_agents; n_agents <= max_agents; n_agents++)
		{
			for (int i = 1; i <= n_instances; i++)
			{
				std::vector<AgentInfo> a;
				switch (map)
				{
				case MapType::Highway:
					a = generate_highway_agents(n_agents);
					break;
				case MapType::Exit:
					a = generate_exit_agents(n_agents);
					break;
				case MapType::Roundabout:
					a = generate_roundabout_agents(n_agents);
					break;
				case MapType::Intersection:
					a = generate_intersection_agents(n_agents);
					break;
				case MapType::Entrance:
					a = generate_entrance_agents(n_agents);
					break;
				}

				std::vector<f32> current_costs; // changes every instance
				std::vector<Solution> current_solutions;

				for (std::vector<f32> speeds : set_of_speeds)
				{
					g = set_graph_speeds(g, speeds);
					GraphData data = { g,a };

					try{
					Solution solution = high_level(&data.graph, data.agents, ModelTypeDiscreteSpeeds);

					if (solution.paths[0].size() == 0)
					{
						printf("%04d timeout on %d speeds\n", counter, speeds.size());

						// log timeout
						file << std::setfill('0') << std::setw(4) << counter << " timeout on " << speeds.size() << " speeds\n";
						file << "\tid  s  g\n";
						for (AgentInfo ai : a)
						{
							file
								<< "\t" << std::setw(2) << ai.id << " "
								<< std::setw(2) << ai.start_vertex << " "
								<< std::setw(2) << ai.goal_vertex << "\n";
						}
						file << "\n\n";

						// reset i value and retry for this instance counter
						i--;
						goto retry;
					}

					current_solutions.push_back(solution);

					f32 w = 0.95;//0.93;
					f32 avg_makespan = get_avg_makespan(&solution.paths);
					f32 cost = w * avg_makespan + (1 - w) * speeds.size();

					current_costs.push_back(cost);
					}
					catch(...){
						printf("%04d error on %d speeds\n", counter, speeds.size());
						file << std::setfill('0') << std::setw(4) << counter << " error on " << speeds.size() << " speeds\n";
						i--;
						goto retry;
					}

				}
				{
					int min_index = std::distance(
						current_costs.begin(),
						std::min_element(current_costs.begin(), current_costs.end()));
					exp1ClassificationType class_type = static_cast<exp1ClassificationType>(min_index);

					//build file name as: "counter_map_agents", e.g. 0001_h_02
					std::ostringstream ss_name;
					ss_name << std::setfill('0') << std::setw(4) << counter << "_" << map_name << "_" << std::setw(2) << n_agents;
					std::string file_name = ss_name.str();

					// write the instance as csv file into the correct class folder
					exp1_write_train_csv(a, class_type, data_type, file_name);

					// update total costs
					for (int i = 0; i < 2; i++)
					{
						total_costs[i] += current_costs[i];
					}
					total_costs[2] += current_costs[min_index];

					std::cout << file_name << " " << cts[min_index];
					printf("\tA: %.2f | B: %.2f\n", current_costs[0], current_costs[1]);

					// log solution info
					file << file_name << " " << cts[min_index];
					file << "\t\tA:" << current_costs[0] << " | B: " << current_costs[1] << "\n";

					file << "Execution time, Nodes explored, Flowtime\n";
					for (Solution solution : current_solutions)
					{
						f32 execution_t = solution.runtime_info.execution_time;
						u32 nodes_explored = solution.runtime_info.nodes_explored;
						f32 sic = cost(&solution.paths);
						file << execution_t << ", " << nodes_explored << ", " << sic << "\n";
					}

					file << "\n";

					std::vector<int> speed_number = { 2, 4 }; // dumb..

					int x = 0;
					for (Solution solution : current_solutions)
					{
						file << "Solution for " << speed_number[x++] << " speeds:\n";
						for (Path path : solution.paths)
						{
							for (LowLevelNode node : path)
							{
								file << "(" << node.vertex << ", " << node.arrival_time << ", " << node.speed << ")\n";
							}
							file << "\n";
						}
					}

					file << "\n";

					counter++;
				}
			retry:;
			}

			printf("\n\tSummary after %04d:\n", counter - 1);
			printf("\tA: %.2f | B: %.2f | best: %.2f\n\n", total_costs[0], total_costs[1], total_costs[2]);

			// log summary
			file << "\tSummary after " << std::setfill('0') << std::setw(4) << counter - 1 << ":\n";
			file << "\tA:" << total_costs[0] << " | B: " << total_costs[1] << " | best: " << total_costs[2] << "\n\n\n";
		}
	}
	file.close();
}

void exp1_generate_test_data()
{
  //std::vector<MapType> maps = { MapType::Highway, MapType::Exit, MapType::Roundabout, MapType::Intersection };
	std::vector<MapType> maps = { MapType::Entrance };

	std::vector<std::vector<f32>> set_of_speeds = { {0, 10}, { 0, 3.3, 6.6, 10 } };

	u32 min_agents = 2;
	u32 max_agents = 7;
	int n_instances = 20; // change!
	int counter = 851; // change!

	std::vector<std::string> cts = { "A", "B" };
	std::vector<f32> total_costs = { 0,0,0 }; // class A, class B, best

	std::string log_path = "../data_exp1/";

	std::string file_path = log_path + "exp1_test_log.txt"; // change!
	std::ofstream file;
	file.open(file_path);

	std::string test_file_path = log_path + "test_data.csv";
	std::ofstream file2;
	file2.open(test_file_path);
	file2 << "file,A,B\n";

	for (MapType map : maps)
	{
		Graph g;
		std::string map_name;

		switch (map)
		{
		case MapType::Highway:
			g = generate_highway_graph();
			map_name = "h";
			break;
		case MapType::Exit:
			g = generate_exit_graph();
			map_name = "e";
			break;
		case MapType::Roundabout:
			g = generate_roundabout_v2_graph();
			map_name = "r";
			break;
		case MapType::Intersection:
			g = generate_intersection_graph();
			map_name = "i";
			break;
		case MapType::Entrance:
			g = generate_entrance_graph();
			map_name = "n";
			break;
		}

		compute_lookup(g);

		for (u32 n_agents = min_agents; n_agents <= max_agents; n_agents++)
		{
			for (int i = 1; i <= n_instances; i++)
			{
				std::vector<AgentInfo> a;
				switch (map)
				{
				case MapType::Highway:
					a = generate_highway_agents(n_agents);
					break;
				case MapType::Exit:
					a = generate_exit_agents(n_agents);
					break;
				case MapType::Roundabout:
					a = generate_roundabout_agents(n_agents);
					break;
				case MapType::Intersection:
					a = generate_intersection_agents(n_agents);
					break;
				case MapType::Entrance:
					a = generate_entrance_agents(n_agents);
					break;
				}

				std::vector<f32> current_costs; // changes every instance
				std::vector<Solution> current_solutions;

				for (std::vector<f32> speeds : set_of_speeds)
				{
					g = set_graph_speeds(g, speeds);
					GraphData data = { g,a };

					try{
					Solution solution = high_level(&data.graph, data.agents, ModelTypeDiscreteSpeeds);

					if (solution.paths[0].size() == 0)
					{
						printf("%04d timeout on %d speeds\n", counter, speeds.size());

						// log timeout
						file << std::setfill('0') << std::setw(4) << counter << " timeout on " << speeds.size() << " speeds\n";
						file << "\tid  s  g\n";
						for (AgentInfo ai : a)
						{
							file
								<< "\t" << std::setw(2) << ai.id << " "
								<< std::setw(2) << ai.start_vertex << " "
								<< std::setw(2) << ai.goal_vertex << "\n";
						}
						file << "\n\n";

						// reset i value and retry for this instance counter
						i--;
						goto retry;
					}

					current_solutions.push_back(solution);

					f32 w = 0.95;
					f32 avg_makespan = get_avg_makespan(&solution.paths);
					f32 cost = w * avg_makespan + (1 - w) * speeds.size();

					current_costs.push_back(cost);
					}
					catch(...){
						printf("%04d error on %d speeds\n", counter, speeds.size());
						file << std::setfill('0') << std::setw(4) << counter << " error on " << speeds.size() << " speeds\n";
						i--;
						goto retry;
					}

				}
				{
					int min_index = std::distance(
						current_costs.begin(),
						std::min_element(current_costs.begin(), current_costs.end()));
					exp1ClassificationType class_type = static_cast<exp1ClassificationType>(min_index);

					//build file name as: "counter_map_agents", e.g. 0001_h_02
					std::ostringstream ss_name;
					ss_name << std::setfill('0') << std::setw(4) << counter << "_" << map_name << "_" << std::setw(2) << n_agents;
					std::string file_name = ss_name.str();

					file2
						<< file_name << ","
						<< current_costs[0] << ","
						<< current_costs[1] << "\n";

					// write the instance as csv file
					exp1_write_test_csv(a, file_name);

					// update total costs
					for (int i = 0; i < 2; i++)
					{
						total_costs[i] += current_costs[i];
					}
					total_costs[2] += current_costs[min_index];

					std::cout << file_name << " " << cts[min_index];
					printf("\tA: %.2f | B: %.2f\n", current_costs[0], current_costs[1]);

					// log solution info
					file << file_name << " " << cts[min_index];
					file << "\t\tA:" << current_costs[0] << " | B: " << current_costs[1] << "\n";

					file << "Execution time, Nodes explored, Flowtime\n";
					for (Solution solution : current_solutions)
					{
						f32 execution_t = solution.runtime_info.execution_time;
						u32 nodes_explored = solution.runtime_info.nodes_explored;
						f32 sic = cost(&solution.paths);
						file << execution_t << ", " << nodes_explored << ", " << sic << "\n";
					}

					file << "\n";

					std::vector<int> speed_number = { 2, 4 }; // dumb..

					int x = 0;
					for (Solution solution : current_solutions)
					{
						file << "Solution for " << speed_number[x++] << " speeds:\n";
						for (Path path : solution.paths)
						{
							for (LowLevelNode node : path)
							{
								file << "(" << node.vertex << ", " << node.arrival_time << ", " << node.speed << ")\n";
							}
							file << "\n";
						}
					}

					file << "\n";

					counter++;
				}
			retry:;
			}

			printf("\n\tSummary after %04d:\n", counter - 1);
			printf("\tA: %.2f | B: %.2f | best: %.2f\n\n", total_costs[0], total_costs[1], total_costs[2]);

			// log summary
			file << "\tSummary after " << std::setfill('0') << std::setw(4) << counter - 1 << ":\n";
			file << "\tA:" << total_costs[0] << " | B: " << total_costs[1] << " | best: " << total_costs[2] << "\n\n\n";
		}
	}
	file.close();
	file2.close();
}







// start TS code


static Vec2
normalize(Vec2 v)
{
	return v * (1 / length(v));
}

struct AgentPlan {
	f32 radius;
	std::vector<f32> timeStamps;
	std::vector<Vec2> positions;
	std::vector<f32> speeds;
};

struct ExecutableSolution {
	std::vector<AgentPlan> agentPlans;
};


enum EModelType
{
	DiscreteTime,
	ContinuousTime,
	DiscreteSpeeds
};


//specifically a type 2 edge
struct TPG_Edge {
	u32 cost = 0;
	u32 to = 0;
};

struct TPG_Vertex_Lookup {
	u32 pathIndex = 0;
	u32 stepIndex = 0;
};

//type 1 edges are handled with parent-child relations
struct TPG_Vertex {
	u32 TPGVertexID = 0;
	s32 realVertexID = -1;
	f32 arrivalTime = 0;
	f32 speed = 0;
	Vec2 pos = { 0,0 };
	std::vector<TPG_Edge> edges = {};
	s32 parentID = 0;
	s32 childID = 0;
	std::vector<u32> dependencies; //vertices with t2 edges to this vertex
	b32 finished = 0; //tells whether this vertex has a time assigned to 
	b32 isGhostVertex = 0;
};

struct TPG_SubPath {
	b32 initiated = 0;
	std::vector<u32> vertexIDs;
	std::vector<u32> dependencies;
	f32 length = 0;
	TPG_SubPath* nextSubpath = nullptr;
};

struct TPG
{
	std::vector<std::vector<TPG_Vertex>> paths;
};

static TPG tpg;
static std::vector<TPG_Vertex_Lookup> LUT;


static TPG_Vertex* GetVertex(u32 id) {
	TPG_Vertex_Lookup lu = LUT.at(id);
	return &tpg.paths[lu.pathIndex][lu.stepIndex];
}

//Calculate acceleration with known travel distance and travel time
static f32 CalcAccKnownTime(f32 dist, f32 time, f32 startVelocity) {
	return 2.0 * (dist - startVelocity * time) / (time * time);
}

//Calculate acceleration to travel a distance with given velocities. Also returns time it takes to travel distance
static f32 CalcAcc(f32 dist, f32 startVelocity, f32 targetVelocity, f32* outTime) {
	f32 stepSize = 0.1f;
	f32 a, d, t;
	f32 velDiff = abs(startVelocity - targetVelocity);

	if (velDiff < 0.0001) {
		if (outTime)*outTime = dist / startVelocity;
		return 0;
	}

	if (startVelocity < targetVelocity) {
		for (a = 0; a < 1000; a += stepSize) {
			t = velDiff / a;
			d = t * startVelocity + t * t * a / 2;
			if (d < dist) break;
		}

		//top, mid and bot refers to acceleration values of interest, a span that gets narrower and narrower
		f32 top = a;
		f32 bot = a - stepSize;
		f32 mid;
		for (int i = 0; i < 10; i++) {
			mid = (top + bot) / 2.0;
			t = velDiff / mid;
			d = t * startVelocity + t * t * mid / 2;
			if (d < dist) top = mid;
			else bot = mid;
		}
		if (outTime)*outTime = t;
		return mid;
	}
	else {
		for (a = 0; a > -1000; a -= stepSize) {
			t = abs(velDiff / a);
			d = t * startVelocity + t * t * a / 2;
			if (d < dist) break;
		}

		f32 bot = a;
		f32 top = a + stepSize;
		f32 mid;
		for (int i = 0; i < 10; i++) {
			mid = (top + bot) / 2.0f;
			t = abs(velDiff / mid);
			d = t * startVelocity + t * t * mid / 2;
			if (d > dist) top = mid;
			else bot = mid;
		}

		if (outTime)*outTime = t;
		return mid;
	}
}

//solve the problem 0=ax^2 + bx + c, returns the smallest solution if there is one
static f32 secondDegreeEqSolver(f32 a, f32 b, f32 c) {

	f32 result;
	f32 x1, x2, discriminant;
	discriminant = b * b - 4 * a * c;

	if (discriminant > 0) {
		x1 = (-b + sqrt(discriminant)) / (2 * a);
		x2 = (-b - sqrt(discriminant)) / (2 * a);
		result = x1;
	}

	else if (discriminant == 0) {
		x1 = (-b + sqrt(discriminant)) / (2 * a);
		result = x1;
	}

	else {
		result = -1;
	}

	return result;
}

//Calculate time to travel a distance given acceleration and velocity. returns -1 if dist is not reached
static f32 CalcTime(f32 dist, f32 acc, f32 startVel) {
	if (abs(acc) == 0 && abs(startVel) == 0) {
		return -1;
	}
	else if (abs(acc) < 0.000001) {
		return dist / startVel;
	}
	else return secondDegreeEqSolver(acc / 2, startVel, -1 * dist);
	/*
	f32 stepSize = 0.1;
	f32 t;
	f32 d;

	d = 0.1f;
	t = 0;
	while (d < dist) {
		t += stepSize;
		d = startVel * t + acc * t * t / 2.0f;
		if (t > 1000) {
			return -1;
		}
	}

	f32 top, mid, bot;
	top = t;
	bot = 0;

	for (int i = 0; i < 30; i++) {
		mid = (bot + top) / 2.0;
		d = startVel * mid + acc * mid * mid / 2.0f;

		if (d < dist) {
			bot = mid;
		}
		else {
			top = mid;
		}
	}

	if (abs(d-dist)>0.001) return -1;
	return mid;
	*/
}

//calculates time to travel when distance, start vel and final vel is known
static f32 CalcTimeSimple(f32 dist, f32 startVel, f32 stopVel) {
	return dist / (0.5 * (startVel + stopVel));
}

static f32 GetLatestDependency(TPG_SubPath* subPath) {
	f32 latestDependency = 0;
	for (u32 dependency : subPath->dependencies)
		latestDependency = std::fmax(latestDependency, GetVertex(dependency)->arrivalTime);
	return latestDependency;
}

static f32 GetLatestDependency(TPG_Vertex* v) {
	f32 latestDependency = 0;
	for (u32 dependency : v->dependencies)
		latestDependency = std::fmax(latestDependency, GetVertex(dependency)->arrivalTime);
	return latestDependency;
}

static b32 IsEntryAllowed(TPG_SubPath* subPath, f32 entryTime, f32 entrySpeed, f32 maxAcc) {

	if (subPath == nullptr) return 1;
	f32 timeToTraverseSubpath = CalcTime(subPath->length, -0.9 * maxAcc, entrySpeed);

	if (timeToTraverseSubpath < 0) return true;

	f32 exitSpeed = entrySpeed - timeToTraverseSubpath * maxAcc;
	f32 exitTime = entryTime + timeToTraverseSubpath;
	//iteratively check with next subpath if entry is allowed
	return IsEntryAllowed(subPath->nextSubpath, exitTime, exitSpeed, maxAcc);
}

static b32 IsAcclerationAllowed(TPG_Vertex* v, f32 entryTime, f32 entrySpeed, f32 acc) {
	if (v->childID == -1 || (v->dependencies.size() == 0 && entrySpeed == 0)) return true;
	f32 latestDependency = GetLatestDependency(v);
	if (entryTime < latestDependency) return false;

	//assume max deacceleration. are proceeding vertices feasible?
	TPG_Vertex* vChild = GetVertex(v->childID);
	f32 dist = length(v->pos - vChild->pos);
	f32 timeToChild = CalcTime(dist, acc, entrySpeed);
	f32 timeAtChild, speedAtChild;

	if (timeToChild < 0) {
		speedAtChild = 0;
		timeToChild = dist / ((entrySpeed + speedAtChild) / 2);
		timeAtChild = entryTime + timeToChild;
	}
	else {
		timeAtChild = entryTime + timeToChild;
		speedAtChild = entrySpeed + timeToChild * acc;
	}

	return IsAcclerationAllowed(GetVertex(v->childID), timeAtChild, speedAtChild, acc);
}

static f32 OptimizePath(TPG_Vertex* v, f32 maxAcc, f32 maxSpeed) {

	if (v->isGhostVertex) {
		TPG_Vertex* parent = GetVertex(v->parentID);
		f32 parentSpeed = parent->speed;
		f32 distToParent = distance(parent->pos, v->pos);
		v->arrivalTime = parent->arrivalTime + distToParent / parentSpeed;
		return 0;
	}

	TPG_Vertex* vChild = GetVertex(v->childID);
	f32 dist = length(vChild->pos - v->pos);
	f32 timeToChild, timeAtChild, speedAtChild = 0;

	f32 allowedTimeAtChild = 0, allowedSpeedAtChild = 0;

	f32 knownAllowedAcc = 0;
	b32 validExit = 0;
	if (v->speed > 0) {
		timeToChild = dist / ((v->speed + 0) / 2);
		f32 testAcc = -1 * v->speed / timeToChild;
		if (abs(testAcc) < maxAcc) {
			knownAllowedAcc = testAcc;
			validExit = IsAcclerationAllowed(vChild, timeToChild + v->arrivalTime, 0, -0.95 * maxAcc);
			if (validExit) {
				allowedTimeAtChild = timeToChild + v->arrivalTime;
				allowedSpeedAtChild = 0;
			}
		}
	}

	if (v->TPGVertexID == 97) {
		int x = 0;
	}
	f32 topAcc = maxAcc, botAcc = maxAcc * -1, acc;
	u32 counter = 0;
	while (counter++ < 30 || !validExit) {

		acc = (topAcc + botAcc) / 2;
		timeToChild = CalcTime(dist, acc, v->speed);
		timeAtChild = v->arrivalTime + timeToChild;
		speedAtChild = v->speed + timeToChild * acc;

		if (timeToChild < 0) {
			botAcc = acc;
			continue;
		}
		if (speedAtChild > maxSpeed) {
			topAcc = acc;
			continue;
		}
		if (speedAtChild < 0) {
			botAcc = acc;
			continue;
		}

		b32 isAccAllowed = IsAcclerationAllowed(vChild, timeAtChild, speedAtChild, -0.95 * maxAcc);
		if (isAccAllowed) {
			allowedTimeAtChild = timeAtChild;
			allowedSpeedAtChild = speedAtChild;
			botAcc = acc;
			knownAllowedAcc = acc;
			validExit = 1;
		}
		else {
			topAcc = acc;
		}
	}

	b32 test = IsAcclerationAllowed(vChild, allowedTimeAtChild, allowedSpeedAtChild, -0.95 * maxAcc);

	vChild->arrivalTime = allowedTimeAtChild;
	vChild->speed = allowedSpeedAtChild;
	return OptimizePath(vChild, maxAcc, maxSpeed);
}

static TPG Generate_TPG(Solution solution, Graph graph, f32 agentRadius, f32 maxAcceleration, f32 maxSpeed) {
	tpg.paths.clear();
	LUT.clear();

	u32 lastVertexID = 0;
	//create initial TPG vertices
	for (u32 i = 0; i < solution.paths.size(); i++) {
		//init paths with empty array
		tpg.paths.push_back({});
		for (u32 j = 0; j < solution.paths[i].size(); j++) {
			LowLevelNode node = solution.paths[i][j];
			//don't add nodes if the vertex id is the same as the previous one
			if (!j == 0) {
				if (node.vertex == solution.paths[i][j - 1].vertex) continue;
			}
			TPG_Vertex tpg_v;
			tpg_v.realVertexID = node.vertex;
			tpg_v.TPGVertexID = lastVertexID++;
			tpg_v.arrivalTime = node.arrival_time;
			tpg_v.pos = graph.vertices[node.vertex].position;//
			//create type 1 edges between parent and child
			if (j == 0) {
				tpg_v.parentID = -1;
			}
			else {
				tpg_v.parentID = tpg.paths[i].back().TPGVertexID;
				tpg.paths[i].back().childID = tpg_v.TPGVertexID;
			}

			if (j == solution.paths[i].size() - 1) tpg_v.childID = -1;

			tpg.paths[i].push_back(tpg_v);
		}
	}
	//create type 2 edges
	for (u32 i = 0; i < tpg.paths.size(); i++) {
		for (u32 j = 0; j < tpg.paths[i].size(); j++) {
			TPG_Vertex outerVertex = tpg.paths[i][j];
			for (u32 k = 0; k < tpg.paths.size(); k++) {
				if (k == i) continue;
				for (u32 l = 0; l < tpg.paths[k].size(); l++) {
					TPG_Vertex innerVertex = tpg.paths[k][l];
					if (innerVertex.arrivalTime < outerVertex.arrivalTime) continue;
					if (innerVertex.realVertexID != outerVertex.realVertexID) continue;

					TPG_Edge edge;
					edge.to = innerVertex.TPGVertexID;
					edge.cost = 0;
					tpg.paths[i][j].edges.push_back(edge);
					break;
				}
			}
		}
	}
	//create safety markers
	for (u32 i = 0; i < tpg.paths.size(); i++) {
		for (u32 j = 0; j < tpg.paths[i].size(); j = j + 3) {
			f32 safetyMarkerDist = agentRadius * 1.5;

			if (j == tpg.paths[i].size() - 1) {
				TPG_Vertex ghostTail;
				ghostTail.isGhostVertex = 1;
				ghostTail.TPGVertexID = lastVertexID++;
				ghostTail.realVertexID = -1;
				ghostTail.childID = -1;

				ghostTail.parentID = tpg.paths[i][j].TPGVertexID;
				tpg.paths[i][j].childID = ghostTail.TPGVertexID;

				Vec2 offset = { safetyMarkerDist ,0 };
				ghostTail.pos = tpg.paths[i][j].pos + offset;
				tpg.paths[i].push_back(ghostTail);
				break;
			}

			TPG_Vertex v1, v2;
			v1.TPGVertexID = lastVertexID++;
			v2.TPGVertexID = lastVertexID++;
			v1.realVertexID = -1;
			v2.realVertexID = -1;

			v1.parentID = tpg.paths[i][j].TPGVertexID;
			v1.childID = v2.TPGVertexID;
			tpg.paths[i][j].childID = v1.TPGVertexID;

			v2.parentID = v1.TPGVertexID;
			v2.childID = tpg.paths[i][j + 1].TPGVertexID;
			tpg.paths[i][j + 1].parentID = v2.TPGVertexID;

			Vec2 dir = normalize(tpg.paths[i][j + 1].pos - tpg.paths[i][j].pos);
			f32 dist = distance(tpg.paths[i][j + 1].pos, tpg.paths[i][j].pos);

			v1.pos = tpg.paths[i][j].pos + dir * safetyMarkerDist;
			v2.pos = tpg.paths[i][j + 1].pos - dir * safetyMarkerDist;

			tpg.paths[i].insert(tpg.paths[i].begin() + j + 1, { v1,v2 });
		}
	}

	//build a lookup table for easier access of vertices. to find vertice fetch lookup with index of the vertex id you seek
	u32 lookupSize = 0;
	for (u32 i = 0; i < tpg.paths.size(); i++) lookupSize += tpg.paths[i].size();
	LUT.resize(lookupSize);
	for (u32 i = 0; i < tpg.paths.size(); i++) {
		for (u32 j = 0; j < tpg.paths[i].size(); j++) {
			TPG_Vertex_Lookup v = { i,j };
			u32 vertexID = tpg.paths[i][j].TPGVertexID;
			LUT[vertexID] = v;
		}
	}

	//reassign t2 edges and set up depedencies
	for (u32 i = 0; i < tpg.paths.size(); i++) {
		for (u32 j = 0; j < tpg.paths[i].size(); j++) {
			std::vector<TPG_Edge> edges;
			if (tpg.paths[i][j].edges.size() == 0 || tpg.paths[i][j].realVertexID == -1) continue;

			//remove edges from original owner and add them to child
			edges = tpg.paths[i][j].edges;

			tpg.paths[i][j].edges.clear();
			tpg.paths[i][j + 1].edges = edges;


			for (u32 e = 0; e < edges.size(); e++) {
				//find edge targets parents and reassign edges to point at them
				TPG_Vertex* newTarget;
				newTarget = GetVertex(edges[e].to);
				newTarget = GetVertex(newTarget->parentID);
				tpg.paths[i][j + 1].edges[e].to = newTarget->TPGVertexID;
				newTarget->dependencies.push_back(tpg.paths[i][j + 1].TPGVertexID);
			}
		}
	}

	//arrival time initiation
	std::vector<b32> finishedPaths = {};
	std::vector<u32> pathProgression = {};
	u32 numPathsFinished = 0;
	u32 pathIndex = 0;
	for (u32 i = 0; i < tpg.paths.size(); i++) {
		finishedPaths.push_back(0);
		pathProgression.push_back(0);
	}
	while (numPathsFinished < tpg.paths.size()) {
		pathIndex++;
		pathIndex = pathIndex % tpg.paths.size();
		if (finishedPaths[pathIndex]) continue;

		TPG_Vertex* vertex;
		for (u32 i = pathProgression[pathIndex] + 1; i < tpg.paths[pathIndex].size(); i++) {
			vertex = &tpg.paths[pathIndex][i];

			if (vertex->finished) continue;

			//check if all dependencies are resolved, return the latest one
			f32 maxDependency = 0.0f;
			b32 dependenciesResolved = 1;
			std::vector<u32> dependencies = vertex->dependencies;
			b32 hasDependencies = !dependencies.empty();
			for (u32 j : dependencies) {
				if (GetVertex(j)->finished) {
					maxDependency = std::fmax(maxDependency, GetVertex(j)->arrivalTime);
				}
				else {
					dependenciesResolved = 0;
					break;
				}
			}

			//if all dependencies are resolved, 
			if (dependenciesResolved) {

				tpg.paths[pathIndex][i].finished = 1;//set vertex finished
				pathProgression[pathIndex] = i;

				TPG_Vertex* parentVertex = GetVertex(vertex->parentID);
				f32 dist = distance(vertex->pos, parentVertex->pos);
				f32 speed = 0.1;
				f32 minTravelTime = dist / speed;
				vertex->arrivalTime = std::fmax(parentVertex->arrivalTime + minTravelTime, maxDependency);

				if (tpg.paths[pathIndex].size() - 1 == i) {
					numPathsFinished++;
					finishedPaths[pathIndex] = 1;
				}
			}
			else break;
		}
	}

	//optimization
	for (u32 i = 0; i < 15; i++) {
		for (u32 j = 0; j < tpg.paths.size(); j++) {
			OptimizePath(&tpg.paths[j][0], maxAcceleration, maxSpeed);
		}
	}

	//confirm correctness
	for (u32 i = 0; i < tpg.paths.size(); i++) {
		for (u32 j = 0; j < tpg.paths[i].size(); j++) {
			TPG_Vertex* v = &tpg.paths[i][j];
			f32 latestDependency = GetLatestDependency(v);

			if (v->arrivalTime < latestDependency) {
				int x = 0;
			}
			if (v->speed > maxSpeed) {
				int x = 0;
			}
		}
	}

	return tpg;
}

//todo: must create new graph from tpg with safety markers
static std::vector<Path> TPGtoPathbuffer(TPG t) {
	LowLevelNode n;
	std::vector<Path> buf;
	for (u32 i = 0; i < t.paths.size(); i++) {
		buf.push_back({});
		for (u32 j = 0; j < t.paths[i].size() - 1; j++) { //dont use last vertex as thats always a ghost tail
			if (t.paths[i][j].realVertexID == -1) continue;
			n.arrival_time = t.paths[i][j].arrivalTime;
			n.vertex = t.paths[i][j].realVertexID;
			n.speed = t.paths[i][j].speed;
			buf[i].push_back(n);
		}
	}
	return buf;
}

static ExecutableSolution TPGtoExecutable(TPG t, f32 agentRadius) {
	ExecutableSolution exec;
	for (std::vector<TPG_Vertex> p : t.paths) {
		AgentPlan plan;
		plan.radius = agentRadius;
		for (TPG_Vertex v : p) {
			if (v.isGhostVertex) continue;
			Vec2 pos = v.pos;
			plan.positions.push_back(pos);
			plan.timeStamps.push_back(v.arrivalTime);
			plan.speeds.push_back(v.speed);
		}
		exec.agentPlans.push_back(plan);
	}
	return exec;
}

static f32 GetMakespan(Solution s, EModelType model) {
	f32 makespan = 0;
	switch (model) {
	case DiscreteTime:
		for (Path path : s.paths) makespan = fmax(path.size(), makespan);
		break;

	case ContinuousTime:
		for (Path path : s.paths) makespan = fmax(path.back().arrival_time, makespan);
		break;
	case DiscreteSpeeds:
		for (Path path : s.paths) makespan = fmax(path.back().arrival_time, makespan);
		break;
	}
	return makespan;
}

static f32 GetSIC(Solution s, EModelType model) {
	f32 sic = 0;
	switch (model) {
	case DiscreteTime:
		for (Path path : s.paths) sic += path.size();
		break;
	case ContinuousTime:
		for (Path path : s.paths) sic += path.back().arrival_time;
		break;
	case DiscreteSpeeds:
		for (Path path : s.paths) sic += path.back().arrival_time;
		break;
	}

	return sic;
}


void ts_times()
{
	std::vector<std::string> instances = {
		"0238_n_04",
		"0291_n_04",
		"0324_n_05",
		"0396_n_05",
		"0482_n_06",
		"0469_n_06" };
	u32 i = 0;

	std::vector<u32> agent_count = { 4, 4, 5, 5, 6, 6 };
	u32 k = 0;

	std::vector<std::vector<u32>> agent_start_locations = {
		{0, 16, 15, 5},
		{28, 2, 4, 18},
		{19, 5, 6, 3, 14},
		{18, 5, 4, 28, 19},
		{2, 19, 29, 4, 14, 20},
		{6, 19, 28, 1, 29, 5} };
	std::vector<std::vector<u32>> agent_goal_locations = {
		{13, 27, 13, 27},
		{27, 27, 27, 27},
		{27, 13, 13, 13, 27},
		{13, 13, 27, 13, 27},
		{27, 27, 27, 13, 13, 27},
		{13, 27, 27, 27, 13, 13} };

	std::vector<std::vector<f32>> set_of_speeds = { {0, 10}, { 0, 3.3, 6.6, 10 } };
	f32 start_speed = 10;
	f32 goal_speed = 0;

	Graph g = generate_entrance_graph();
	compute_lookup(g);

	std::string file_path = "../Thesis_Code/ts_times.csv";
	std::ofstream file;
	file.open(file_path);
	file << "instance,sic_a,sic_b,fun_a,fun_b,ex_a,ex_b,ts_sic_a,ts_sic_b\n";


	for (std::string instance : instances)
	{
		printf("instance %2d / 12", i+1);

		std::vector<f32> CTDS_sics;
		std::vector<f32> CTDS_ML_costs;
		std::vector<f32> CTDS_ex_times;

		std::vector<f32> TS_costs;
		//std::vector<f32> TS_ex_time;

		// get agents
		std::vector<AgentInfo> a;
		for (u32 agent_id = 0; agent_id < agent_count[k]; agent_id++)
		{
			a.push_back({
				agent_id,
				agent_start_locations[i][agent_id],
				start_speed,
				agent_goal_locations[i][agent_id],
				goal_speed,
				AGENT_RADIUS });
		}


		
		for (std::vector<f32> speeds : set_of_speeds)
		{
			g = set_graph_speeds(g, speeds);
			GraphData data = { g,a };


			// solve in CTDS model
			Solution solution = high_level(&data.graph, data.agents, ModelTypeDiscreteSpeeds);

			f32 sic = cost(&solution.paths);

			f32 w = 0.95;
			f32 avg_makespan = get_avg_makespan(&solution.paths);
			f32 ml_cost = w * avg_makespan + (1 - w) * speeds.size();

			CTDS_sics.push_back(sic);
			CTDS_ML_costs.push_back(ml_cost);
			CTDS_ex_times.push_back(solution.runtime_info.execution_time);


			// solve in TS
			TPG tpg = Generate_TPG(solution, g, AGENT_RADIUS, 2, 10);
			ExecutableSolution exec = TPGtoExecutable(tpg, 1);
			f32 execSIC = 0;// , execMakespan = 0;
			for (AgentPlan plan : exec.agentPlans) {
				execSIC += plan.timeStamps.back();
				//execMakespan = fmax(plan.timeStamps.back(), execMakespan);
			}

			TS_costs.push_back(execSIC);

		}


		// write info to file
		file
			<< instance << ","
			<< CTDS_sics[0] << ","
			<< CTDS_sics[1] << ","
			<< CTDS_ML_costs[0] << ","
			<< CTDS_ML_costs[1] << ","
			<< CTDS_ex_times[0] << ","
			<< CTDS_ex_times[1] << ","
			<< TS_costs[0] << ","
			<< TS_costs[1] << "\n";


		i++;
		k++;
	}


	file.close();
}


// end TS code












int main()
{
	exp1_generate_train_data(DataType::train);
  //exp1_generate_train_data(DataType::validation);
  //exp1_generate_test_data();

}
