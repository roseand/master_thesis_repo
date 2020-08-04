#pragma once

#include "solver.h"

/*#define AGENT_RADIUS 1.0
#define AGENT_DIAMETER 2.0 * AGENT_RADIUS
#define LANE_WIDTH 2.0 * AGENT_DIAMETER
#define COLUMN_WIDTH 2.0 * AGENT_DIAMETER
#define EDGE_COST 1.0
#define ROAD_LENGTH 8*/

#define AGENT_RADIUS 1.0
#define LANE_WIDTH 5.0
#define POSITION_WIDTH 2.0 * LANE_WIDTH
#define EDGE_COST 1.0

//#define HIGHWAY_SIZE 32
#define HIGHWAY_SIZE 28
//#define EXIT_SIZE 36
#define EXIT_SIZE 32
#define ROUNDABOUT_SIZE 36
#define ROUNDABOUT_V2_SIZE 64
#define INTERSECTION_SIZE 68

enum class HighwayType
{
	verticalLR,
	verticalRL,
	horizontalDU,
	horizontalUD
};

static Graph
set_graph_speeds(Graph g, std::vector<f32> speeds)
{
	for (int i = 0; i < g.vertices.size(); i++)
	{
		g.vertices[i].speeds = speeds;
	}

	return g;
}



static Graph
add_single_highway(
	Graph g,
	u32 road_length,
	HighwayType highway_type,
	u32 base_id,
	f32 mid,
	f32 start)
{
	f32 y_vertical = mid;
	f32 x_horizontal = mid;

	for (u32 i = 0; i < road_length; i++)
	{
		u32 id = base_id + i;

		// add vertices
		switch (highway_type)
		{
			case HighwayType::verticalLR:
			{
				f32 x_vertical = start + i * POSITION_WIDTH;

				g.vertices[id] = { id, {x_vertical, y_vertical} };
			}
			break;
			case HighwayType::verticalRL:
			{
				f32 x_vertical = start - i * POSITION_WIDTH;

				g.vertices[id] = { id, {x_vertical, y_vertical} };
			}
			break;
			case HighwayType::horizontalDU:
			{
				f32 y_horizontal = start + i * POSITION_WIDTH;

				g.vertices[id] = { id, {x_horizontal, y_horizontal} };
			}
			break;
			case HighwayType::horizontalUD:
			{
				f32 y_horizontal = start - i * POSITION_WIDTH;

				g.vertices[id] = { id, {x_horizontal, y_horizontal} };
			}
			break;
		}

		// final nodes (last in lane) do not have edges exiting them
		if (i == road_length - 1) break;

		// add forward edges
		u32 forward_id = id + 1;

		g.vertices[id].edges.push_back({ forward_id, EDGE_COST });
	}

	return g;
}

static Graph
add_double_highway(
	Graph g,		  
	u32 road_length,
	HighwayType highway_type,
	u32 base_id,  
	f32 mid,   
	f32 start)
{
	for (u32 i = 0; i < road_length; i++)
	{
		u32 id_r = base_id + i;
		u32 id_l = base_id + road_length + i;

		// add vertices
		switch (highway_type)
		{
			case HighwayType::verticalLR:
			{
				f32 x = start + i * POSITION_WIDTH;

				f32 y_r = mid - 0.5 * LANE_WIDTH;
				f32 y_l = mid + 0.5 * LANE_WIDTH;

				g.vertices[id_r] = { id_r, {x, y_r} };
				g.vertices[id_l] = { id_l, {x, y_l} };
			}
			break;
			case HighwayType::verticalRL:
			{
				f32 x = start - i * POSITION_WIDTH;

				f32 y_r = mid + 0.5 * LANE_WIDTH;
				f32 y_l = mid - 0.5 * LANE_WIDTH;

				g.vertices[id_r] = { id_r, {x, y_r} };
				g.vertices[id_l] = { id_l, {x, y_l} };
			}
				break;
			case HighwayType::horizontalDU:
			{
				f32 y = start + i * POSITION_WIDTH;

				f32 x_r = mid + 0.5 * LANE_WIDTH;
				f32 x_l = mid - 0.5 * LANE_WIDTH;

				g.vertices[id_r] = { id_r, {x_r, y} };
				g.vertices[id_l] = { id_l, {x_l, y} };
			}
				break;
			case HighwayType::horizontalUD:
			{
				f32 y = start - i * POSITION_WIDTH;

				f32 x_r = mid - 0.5 * LANE_WIDTH;
				f32 x_l = mid + 0.5 * LANE_WIDTH;

				g.vertices[id_r] = { id_r, {x_r, y} };
				g.vertices[id_l] = { id_l, {x_l, y} };
			}
				break;
		}

		// final nodes (last in lane) do not have edges exiting them
		if (i == road_length - 1) break;

		// add forward and switch edges
		u32 forward_bot_id = id_r + 1;
		u32 forward_top_id = id_l + 1;

		g.vertices[id_r].edges.push_back({ forward_bot_id, EDGE_COST });
		g.vertices[id_r].edges.push_back({ forward_top_id, EDGE_COST });

		g.vertices[id_l].edges.push_back({ forward_top_id, EDGE_COST });
		g.vertices[id_l].edges.push_back({ forward_bot_id, EDGE_COST });
	}

	return g;
}

static Graph
add_exit(Graph g)
{
	u32 id_base = HIGHWAY_SIZE;

	u32 exit_length = 4;

	//std::vector<f32> x_exit = { 130, 135, 140, 140 };
	std::vector<f32> x_exit = { 110, 115, 120, 120 };
	std::vector<f32> y_exit = { 25, 20, 10, 0 };

	for (u32 i = 0; i < exit_length; i++)
	{
		u32 id_exit = id_base + i;

		// add vertices
		g.vertices[id_exit] = { id_exit, {x_exit[i], y_exit[i]} };

		// final node does not have an exiting edge
		if (i == exit_length - 1) break;

		// add forward edges
		u32 forward_exit_id = id_exit + 1;

		g.vertices[id_exit].edges.push_back({ forward_exit_id, EDGE_COST });

	}

	// connect highway and exit
	//g.vertices[12].edges.push_back({ 32, EDGE_COST });
	g.vertices[10].edges.push_back({ 28, EDGE_COST });

	return g;
}

static Graph
add_roundabout(Graph g)
{
	std::vector<u32> ids = { 32, 33, 34, 35 };
	std::vector<f32> xs = { 55, 80, 80, 55 };
	std::vector<f32> ys = { 55, 55, 80, 80 };

	// add 4 roundabout positions
	for (int i = 0; i < 4; i++)
	{
		g.vertices[ids[i]] = { ids[i], {xs[i], ys[i]} };
	}

	// connect highways and roundabout
	int nr_of_connections = 12;
	std::vector<u32> from_ids = { 21, 33, 14, 13, 34, 30, 29, 35, 6,  5, 32, 22 };
	std::vector<u32> to_ids   = { 33, 14, 13, 34, 30, 29, 35,  6, 5, 32, 22, 21 };

	for (int i = 0; i < nr_of_connections; i++)
	{
		g.vertices[from_ids[i]].edges.push_back({ to_ids[i], EDGE_COST });
	}

	return g;
}

static Graph
add_roundabout_v2(Graph g)
{
	{
		std::vector<u32> ids = { 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50 ,51, 52, 53, 54, 55 };
		std::vector<f32> xs = { 45, 55, 60, 65, 75, 80, 80, 80, 75, 65, 60, 55, 45, 40, 40, 40 };
		std::vector<f32> ys = { 45, 40, 40, 40, 45, 55, 60, 65, 75, 80, 80, 80, 75, 65, 60, 55 };

		// add outer ring
		for (int i = 0; i < 16; i++)
		{
			g.vertices[ids[i]] = { ids[i], {xs[i], ys[i]} };
		}

		// connect outer ring
		int nr_of_connections = 16;
		std::vector<u32> from_ids = { 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50 ,51, 52, 53, 54, 55 };
		std::vector<u32> to_ids = { 41, 42, 43, 44, 45, 46, 47, 48, 49, 50 ,51, 52, 53, 54, 55, 40 };
		for (int i = 0; i < nr_of_connections; i++)
		{
			g.vertices[from_ids[i]].edges.push_back({ to_ids[i], EDGE_COST });
		}
	}

	{
		std::vector<u32> ids = { 56, 57, 58, 59, 60, 61, 62, 63 };
		std::vector<f32> xs = { 45, 55, 65, 75, 75, 65, 55, 45 };
		std::vector<f32> ys = { 55, 45, 45, 55, 65, 75, 75, 65 };


		// add inner ring
		for (int i = 0; i < 8; i++)
		{
			g.vertices[ids[i]] = { ids[i], {xs[i], ys[i]} };
		}

		// connect inner ring
		int nr_of_connections = 8;
		std::vector<u32> from_ids = { 56, 57, 58, 59, 60, 61, 62, 63 };
		std::vector<u32> to_ids = { 57, 58, 59, 60, 61, 62, 63, 56 };
		for (int i = 0; i < nr_of_connections; i++)
		{
			g.vertices[from_ids[i]].edges.push_back({ to_ids[i], EDGE_COST });
		}
	}

	{
		// connect highways and roundabout
		int nr_of_connections = 20;
		std::vector<u32> from_ids = { 3,7,54,23,27,42,13,17,46,33,37,50,63,53,57,41,59,45,61,49 };
		std::vector<u32> to_ids = { 55,54,56,43,42,58,47,46,60,51,50,62,53,8,41,28,45,18,49,38 };

		for (int i = 0; i < nr_of_connections; i++)
		{
			g.vertices[from_ids[i]].edges.push_back({ to_ids[i], EDGE_COST });
		}
	}

	return g;
}

static Graph
add_intersection(Graph g)
{
	u32 base_id = 48;
	f32 x_start = 40;
	f32 y_start = 40;
	f32 x_offset = 1;
	f32 y_offset = 1;

	// add 16 grid* positions
	// *corner nodes are offset
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			f32 x = x_start + j * LANE_WIDTH;
			f32 y = y_start + i * LANE_WIDTH;

			if (base_id == 48) { x -= x_offset; y -= y_offset; }
			if (base_id == 51) { x += x_offset; y -= y_offset; }
			if (base_id == 60) { x -= x_offset; y += y_offset; }
			if (base_id == 63) { x += x_offset; y += y_offset; }

			g.vertices[base_id] = { base_id, {x, y} };
			base_id++;
		}
	}

	std::vector<u32> ids = { 64, 65, 66, 67 };
	std::vector<f32> xs = { 46, 47.5, 49, 47.5 };
	std::vector<f32> ys = { 47.5, 46, 47.5, 49 };

	// add 4 middle positions
	for (int i = 0; i < 4; i++)
	{
		g.vertices[ids[i]] = { ids[i], {xs[i], ys[i]} };
	}

	// connect highways and intersection
	//
	// connecting order:
	// - in1, in2, in3, in4
	// and for each:
	//		- straight left
	//		- straight right
	//		- right
	//		- left

	// incoming lane 1 connections
	int in1_nr_of_connections = 15;

	std::vector<u32> in1_from_ids =
	{
		31, 50, 54, 58, 62,
		27, 51, 55, 59, 63,
		51,
		50, 65, 64, 56 };

	std::vector<u32> in1_to_ids =
	{
		50, 54, 58, 62, 46,
		51, 55, 59, 63, 44,
		20,
		65, 64, 56, 10 };

	for (int i = 0; i < in1_nr_of_connections; i++)
	{
		g.vertices[in1_from_ids[i]].edges.push_back({ in1_to_ids[i], EDGE_COST });
	}

	// incoming lane 2 connections
	int in2_nr_of_connections = 13;

	std::vector<u32> in2_from_ids =
	{
		19, 59, 58, 57,
		15, 63, 62, 61, 60,

		59, 66, 65, 49 };

	std::vector<u32> in2_to_ids =
	{
		59, 58, 57, 56,
		63, 62, 61, 60, 8,

		66, 65, 49, 34 };

	for (int i = 0; i < in2_nr_of_connections; i++)
	{
		g.vertices[in2_from_ids[i]].edges.push_back({ in2_to_ids[i], EDGE_COST });
	}

	// incoming lane 3 connections
	int in3_nr_of_connections = 13;

	std::vector<u32> in3_from_ids =
	{
		43, 61, 57, 53,
		39, 60, 56, 52, 48,

		61, 67, 66, 55 };

	std::vector<u32> in3_to_ids =
	{
		61, 57, 53, 49,
		60, 56, 52, 48, 32,

		67, 66, 55, 22 };

	for (int i = 0; i < in3_nr_of_connections; i++)
	{
		g.vertices[in3_from_ids[i]].edges.push_back({ in3_to_ids[i], EDGE_COST });
	}

	// incoming lane 4 connections
	int in4_nr_of_connections = 11;

	std::vector<u32> in4_from_ids =
	{
		7, 52, 53, 54,
		3, 48, 49, 50,

		52, 64, 67 };

	std::vector<u32> in4_to_ids =
	{
		52, 53, 54, 55,
		48, 49, 50, 51,

		64, 67, 62 };

	for (int i = 0; i < in4_nr_of_connections; i++)
	{
		g.vertices[in4_from_ids[i]].edges.push_back({ in4_to_ids[i], EDGE_COST });
	}

	return g;
}



static Graph
generate_highway_graph()
{
	Graph g;
	g.vertices.resize(HIGHWAY_SIZE);

	//u32 road_length = 16;
	u32 road_length = 14;
	u32 base_id = 0;
	f32 mid = 32.5;
	f32 start = 0;

	g = add_double_highway(g, road_length, HighwayType::verticalLR, base_id, mid, start);

	// remove switch edges from 2nd-to-last nodes
	//g.vertices[14].edges.pop_back();
	g.vertices[12].edges.pop_back();
	//g.vertices[30].edges.pop_back();
	g.vertices[26].edges.pop_back();

	return g;
}

static Graph
generate_exit_graph()
{
	Graph g;
	g.vertices.resize(EXIT_SIZE);

	//u32 road_length = 16;
	u32 road_length = 14;
	u32 base_id = 0;
	f32 mid = 32.5;
	f32 start = 0;

	g = add_double_highway(g, road_length, HighwayType::verticalLR, base_id, mid, start);
	g = add_exit(g);

	// remove switch edges from 2nd-to-last nodes
	//g.vertices[14].edges.pop_back();
	g.vertices[12].edges.pop_back();
	//g.vertices[30].edges.pop_back();
	g.vertices[26].edges.pop_back();

	return g;
}

static Graph
generate_roundabout_graph()
{
	Graph g;
	g.vertices.resize(ROUNDABOUT_SIZE);

	int highway_count = 8;

	std::vector<u32> road_length = { 6, 2, 6, 2, 6, 2, 6, 2 };
	std::vector<HighwayType> hw_type = {
		HighwayType::verticalLR,
		HighwayType::verticalRL,
		HighwayType::verticalRL,
		HighwayType::verticalLR,
		HighwayType::horizontalDU,
		HighwayType::horizontalUD,
		HighwayType::horizontalUD,
		HighwayType::horizontalDU };
	std::vector<u32> base_id = { 0, 6, 8, 14, 16, 22, 24, 30 };
	std::vector<f32> mid = { 65, 70, 70, 65, 70, 65, 65, 70 };
	std::vector<f32> start = { 0, 50, 135, 85, 0, 50, 135, 85 };

	for (int i = 0; i < highway_count; i++)
	{
		g = add_single_highway(g, road_length[i], hw_type[i], base_id[i], mid[i], start[i]);
	}

	g = add_roundabout(g);

	return g;
}

static Graph
generate_roundabout_v2_graph()
{
	Graph g;
	g.vertices.resize(ROUNDABOUT_V2_SIZE);

	int highway_count = 8;

	std::vector<u32> road_length = { 4, 2, 4, 2, 4, 2, 4, 2 };
	std::vector<HighwayType> hw_type = {
		HighwayType::verticalLR,
		HighwayType::verticalRL,
		HighwayType::verticalRL,
		HighwayType::verticalLR,
		HighwayType::horizontalDU,
		HighwayType::horizontalUD,
		HighwayType::horizontalUD,
		HighwayType::horizontalDU };
	std::vector<u32> base_id = { 0, 8, 10, 18, 20, 28, 30, 38 };
	std::vector<f32> mid = { 57.5, 65, 62.5, 55, 62.5, 55, 57.5, 65 };
	std::vector<f32> start = { 0, 30, 120, 90, 0, 30, 120, 90 };

	for (int i = 0; i < highway_count; i++)
	{
		if (i % 2 == 0)
		{
			g = add_double_highway(g, road_length[i], hw_type[i], base_id[i], mid[i], start[i]);
		}
		else
		{
			g = add_single_highway(g, road_length[i], hw_type[i], base_id[i], mid[i], start[i]);
		}
	}

	g = add_roundabout_v2(g);

	return g;
}

static Graph
generate_intersection_graph()
{
	Graph g;
	g.vertices.resize(INTERSECTION_SIZE);

	int highway_count = 8;

	std::vector<u32> road_length = { 4, 2, 4, 2, 4, 2, 4, 2 };
	std::vector<HighwayType> hw_type = {
		HighwayType::verticalLR,
		HighwayType::verticalRL,
		HighwayType::verticalRL,
		HighwayType::verticalLR,
		HighwayType::horizontalDU,
		HighwayType::horizontalUD,
		HighwayType::horizontalUD,
		HighwayType::horizontalDU };
	std::vector<u32> base_id = { 0, 8, 12, 20, 24, 32, 36, 44 };
	std::vector<f32> mid = { 42.5, 52.5, 52.5, 42.5, 52.5, 42.5, 42.5, 52.5 };
	std::vector<f32> start = { 0, 30, 95, 65, 0, 30, 95, 65 };
	
	for (int i = 0; i < highway_count; i++)
	{
		g = add_double_highway(g, road_length[i], hw_type[i], base_id[i], mid[i], start[i]);
	}

	g = add_intersection(g);

	// remove switch edges from 2nd-to-last nodes
	std::vector<int> node_ids = { 8, 10, 20, 22, 32, 34, 44, 46 };
	for (int n_id : node_ids)
	{
		g.vertices[n_id].edges.pop_back();
	}

	return g;
}



u32 highway_map_goal(int random)
{
	u32 id = -1;

	switch (random)
	{
		case 0:
			//id = 15;
			id = 13;
			break;
		case 1:
			//id = 31;
			id = 27;
			break;
	}

	return id;
}

static std::vector<AgentInfo> generate_highway_agents(u32 agent_count)
{
	std::vector<AgentInfo> agents;

	//std::vector<u32> start_vertices = { 0, 1, 2, 3, 4, 5, 6, 7, 16, 17, 18, 19, 20, 21, 22, 23 };
	std::vector<u32> start_vertices = { 0, 1, 2, 3, 4, 5, 6, 7, 14, 15, 16, 17, 18, 19, 20, 21 };
	
	f32 start_speed = 10;
	f32 goal_speed = 0;

	int random = -1;
	u32 goal_id = -1;

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::mt19937 generator(seed);

	std::shuffle(start_vertices.begin(), start_vertices.end(), generator);
	std::uniform_int_distribution<int> goal_distribution(0, 1);

	for (u32 agent_id = 0; agent_id < agent_count; agent_id++)
	{
		random = goal_distribution(generator);
		goal_id = highway_map_goal(random);

		agents.push_back({
			agent_id,
			start_vertices[agent_id],
			start_speed,
			goal_id,
			goal_speed,
			AGENT_RADIUS });
	}

	return agents;
}



u32 exit_map_goal(int random)
{
	u32 id = -1;

	switch (random)
	{
	case 0:
		//id = 15;
		id = 13;
		break;
	case 1:
		//id = 31;
		id = 27;
		break;
	case 2:
		//id = 35;
		id = 31;
		break;
	}

	return id;
}

static std::vector<AgentInfo> generate_exit_agents(u32 agent_count)
{
	std::vector<AgentInfo> agents;

	//std::vector<u32> start_vertices = { 0, 1, 2, 3, 4, 5, 6, 7, 16, 17, 18, 19, 20, 21, 22, 23 };
	std::vector<u32> start_vertices = { 0, 1, 2, 3, 4, 5, 6, 7, 14, 15, 16, 17, 18, 19, 20, 21 };

	f32 start_speed = 10;
	f32 goal_speed = 0;

	int random = -1;
	u32 goal_id = -1;

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::mt19937 generator(seed);

	std::shuffle(start_vertices.begin(), start_vertices.end(), generator);
	std::uniform_int_distribution<int> goal_distribution(0, 2);

	for (u32 agent_id = 0; agent_id < agent_count; agent_id++)
	{
		random = goal_distribution(generator);
		goal_id = exit_map_goal(random);

		agents.push_back({
			agent_id,
			start_vertices[agent_id],
			start_speed,
			goal_id,
			goal_speed,
			AGENT_RADIUS });
	}

	return agents;
}



u32 roundabout_map_goal(int random)
{
	u32 id = -1;

	switch (random)
	{
	case 0:
		//id = 7;
		id = 9;
		break;
	case 1:
		//id = 15;
		id = 19;
		break;
	case 2:
		//id = 23;
		id = 29;
		break;
	case 3:
		//id = 31;
		id = 39;
		break;
	}

	return id;
}

static std::vector<AgentInfo> generate_roundabout_agents(u32 agent_count)
{
	std::vector<AgentInfo> agents;

	//std::vector<u32> start_vertices = { 0, 1, 2, 3, 8, 9, 10, 11, 16, 17, 18, 19, 24, 25, 26, 27 };
	std::vector<u32> start_vertices = { 0, 1, 4, 5, 10, 11, 14, 15, 20, 21, 24, 25, 30, 31, 34, 35 };

	f32 start_speed = 0;
	f32 goal_speed = 10;

	int random = -1;
	u32 goal_id = -1;

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::mt19937 generator(seed);

	std::shuffle(start_vertices.begin(), start_vertices.end(), generator);
	std::uniform_int_distribution<int> goal_distribution(0, 3);

	for (u32 agent_id = 0; agent_id < agent_count; agent_id++)
	{
		random = goal_distribution(generator);
		goal_id = roundabout_map_goal(random);

		agents.push_back({
			agent_id,
			start_vertices[agent_id],
			start_speed,
			goal_id,
			goal_speed,
			AGENT_RADIUS });
	}

	return agents;
}



u32 intersection_map_goal_l1(int random)
{
	u32 id = -1;

	switch (random)
	{
	case 0:
		//id = 9;
		id = 11;
		break;
	case 1:
		//id = 11;
		id = 21;
		break;
	case 2:
		//id = 21;
		id = 45;
		break;
	case 3:
		//id = 23;
		id = 47;
		break;
	//case 4:
	//	id = 45;
	//	break;
	//case 5:
	//	id = 47;
	//	break;
	}

	return id;
}

u32 intersection_map_goal_l2(int random)
{
	u32 id = -1;

	switch (random)
	{
	case 0:
		//id = 9;
		id = 9;
		break;
	case 1:
		//id = 11;
		id = 11;
		break;
	case 2:
		//id = 33;
		id = 35;
		break;
	case 3:
		//id = 35;
		id = 45;
		break;
	//case 4:
	//	id = 45;
	//	break;
	//case 5:
	//	id = 47;
	//	break;
	}

	return id;
}

u32 intersection_map_goal_l3(int random)
{
	u32 id = -1;

	switch (random)
	{
	case 0:
		//id = 9;
		id = 9;
		break;
	case 1:
		//id = 11;
		id = 23;
		break;
	case 2:
		//id = 21;
		id = 33;
		break;
	case 3:
		//id = 23;
		id = 35;
		break;
	//case 4:
	//	id = 33;
	//	break;
	//case 5:
	//	id = 35;
	//	break;
	}

	return id;
}

u32 intersection_map_goal_l4(int random)
{
	u32 id = -1;

	switch (random)
	{
	case 0:
		//id = 21;
		id = 21;
		break;
	case 1:
		//id = 23;
		id = 23;
		break;
	case 2:
		//id = 33;
		id = 33;
		break;
	case 3:
		//id = 35;
		id = 47;
		break;
	//case 4:
	//	id = 45;
	//	break;
	//case 5:
	//	id = 47;
	//	break;
	}

	return id;
}

static std::vector<AgentInfo> generate_intersection_agents(u32 agent_count)
{
	std::vector<AgentInfo> agents;

	std::vector<u32> start_vertices = { 0, 1, 4, 5, 12, 13, 16, 17, 24, 25, 28, 29, 36, 37, 40, 41 };

	f32 start_speed = 0;
	f32 goal_speed = 10;

	int random = -1;
	u32 goal_id = -1;

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::mt19937 generator(seed);

	std::shuffle(start_vertices.begin(), start_vertices.end(), generator);
	std::uniform_int_distribution<int> goal_distribution(0, 3);

	for (u32 agent_id = 0; agent_id < agent_count; agent_id++)
	{
		u32 start_id = start_vertices[agent_id];
		random = goal_distribution(generator);

		if (start_id == 24 || start_id == 25 || start_id == 28 || start_id == 29) { goal_id = intersection_map_goal_l1(random); }
		if (start_id == 12 || start_id == 13 || start_id == 16 || start_id == 17) { goal_id = intersection_map_goal_l2(random); }
		if (start_id == 36 || start_id == 37 || start_id == 40 || start_id == 41) { goal_id = intersection_map_goal_l3(random); }
		if (start_id == 0 || start_id == 1 || start_id == 4 || start_id == 5) { goal_id = intersection_map_goal_l4(random); }
		
		agents.push_back({
			agent_id,
			start_id,
			start_speed,
			goal_id,
			goal_speed,
			AGENT_RADIUS });
	}

	return agents;
}



static std::vector<AgentInfo> handpick_agents()
{
	std::vector<AgentInfo> agents;

	f32 start_speed = 0;
	f32 goal_speed = 10;

	//std::vector<u32> ss = { 24,25,28,29 };
	//std::vector<u32> gs = { 9,11,11,11 };

	//std::vector<u32> ss = { 0,1,4,5 };
	//std::vector<u32> gs = { 21,21,21,21 };

	std::vector<u32> ss = { 4,20 };
	std::vector<u32> gs = { 31,15 };

	for (u32 i = 0; i < ss.size(); i++)
	{
		agents.push_back({ i, ss[i], start_speed, gs[i], goal_speed, AGENT_RADIUS });
	}

	return agents;
}

















static Graph
add_entrance(Graph g)
{
	u32 id_base = HIGHWAY_SIZE;

	u32 entrance_length = 4;

	//std::vector<f32> x_exit = { 110, 115, 120, 120 };
	//std::vector<f32> y_exit = { 25, 20, 10, 0 };
	std::vector<f32> x_entrance = { 40, 40, 45, 50 };
	std::vector<f32> y_entrance = { 0, 10, 20, 25 };

	for (u32 i = 0; i < entrance_length; i++)
	{
		u32 id_entrance = id_base + i;

		// add vertices
		g.vertices[id_entrance] = { id_entrance, {x_entrance[i], y_entrance[i]} };

		// final node does not have an exiting edge
		if (i == entrance_length - 1) break;

		// add forward edges
		u32 forward_entrance_id = id_entrance + 1;

		g.vertices[id_entrance].edges.push_back({ forward_entrance_id, EDGE_COST });

	}

	// connect highway and exit
	//g.vertices[12].edges.push_back({ 32, EDGE_COST });
	g.vertices[31].edges.push_back({ 6, EDGE_COST });

	return g;
}



static Graph
generate_entrance_graph()
{
	Graph g;
	g.vertices.resize(EXIT_SIZE);

	//u32 road_length = 16;
	u32 road_length = 14;
	u32 base_id = 0;
	f32 mid = 32.5;
	f32 start = 0;

	g = add_double_highway(g, road_length, HighwayType::verticalLR, base_id, mid, start);
	g = add_entrance(g);

	// remove switch edges from 2nd-to-last nodes
	//g.vertices[14].edges.pop_back();
	g.vertices[12].edges.pop_back();
	//g.vertices[30].edges.pop_back();
	g.vertices[26].edges.pop_back();

	return g;
}




u32 entrance_map_goal(int random)
{
	u32 id = -1;

	switch (random)
	{
	case 0:
		//id = 15;
		id = 13;
		break;
	case 1:
		//id = 31;
		id = 27;
		break;
	}

	return id;
}

static std::vector<AgentInfo> generate_entrance_agents(u32 agent_count)
{
	std::vector<AgentInfo> agents;

	//std::vector<u32> start_vertices = { 0, 1, 2, 3, 4, 5, 6, 7, 16, 17, 18, 19, 20, 21, 22, 23 };
	//std::vector<u32> start_vertices = { 0, 1, 2, 3, 4, 5, 6, 7, 14, 15, 16, 17, 18, 19, 20, 21 };
	std::vector<u32> start_vertices = { 0, 1, 2, 3, 4, 5, 6, 14, 15, 16, 17, 18, 19, 20, 28, 29 };

	f32 start_speed = 10;
	f32 goal_speed = 0;

	int random = -1;
	u32 goal_id = -1;

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::mt19937 generator(seed);

	std::shuffle(start_vertices.begin(), start_vertices.end(), generator);
	std::uniform_int_distribution<int> goal_distribution(0, 1);

	for (u32 agent_id = 0; agent_id < agent_count; agent_id++)
	{
		random = goal_distribution(generator);
		goal_id = entrance_map_goal(random);

		agents.push_back({
			agent_id,
			start_vertices[agent_id],
			start_speed,
			goal_id,
			goal_speed,
			AGENT_RADIUS });
	}

	return agents;
}










