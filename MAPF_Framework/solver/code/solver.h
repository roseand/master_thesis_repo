#if __llvm__
#include <float.h>
#include <algorithm>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <vector>
#include <queue>
#include <list>
#include <functional>
#include <chrono>

typedef int32_t b32;
typedef double f32;
typedef double f64;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#define global_variable static

#if SIMULATOR_DEBUG
#ifdef __linux__
#include <assert.h>
#define Assert(expression) assert(expression);
#else
#define Assert(expression) if(!(expression)) *(int*)0 = 0
#endif
#else
#define Assert(expression) ;
#endif

#define ArrayCount(array) (sizeof(array) / sizeof((array)[0]))

#define Kilobytes(value) ((value) * 1024LL)
#define Megabytes(value) (Kilobytes(value) * 1024LL)

#define InvalidCodePath Assert(!"Invalid code path!")

#define IsCharacter(c) ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'))
#define IsDigit(c) (c >= '0' && c <= '9')
#define IsWhitespace(c) (c == ' ' || c == '\n' || c == '\r')

#define FloatEq(f1, f2) (fabs(f1 - f2) <= DBL_EPSILON)
#define PI 3.1415

static void
skip_whitespace(u32* i, char* str){  while(IsWhitespace(str[*i])) *i = *i + 1; if (IsWhitespace(str[*i])) *i = *i + 1; }

#if _MSC_VER
#define INIT_MAIN_CLOCK(ID) u64 clock_main_##ID = 0; u32 hits_main_##ID = 0;
#define MAIN_CLOCK_START(ID) u64 start_main_##ID = __rdtsc(); hits_main_##ID++;
#define MAIN_CLOCK_END(ID) clock_main_##ID += __rdtsc() - start_main_##ID;

#define INIT_CLOCK(ID) u64 clock_##ID = 0; u32 hits_##ID = 0; f32 percentage_##ID = 0;
#define CLOCK_START(ID) u64 start_##ID = __rdtsc(); hits_##ID++;
#define CLOCK_END(ID, Main)                                             \
    clock_##ID += __rdtsc() - start_##ID;                               \
    percentage_##ID = (f32)clock_##ID / (f32)clock_main_##Main;
#else
#define INIT_MAIN_CLOCK(ID) ;
#define MAIN_CLOCK_START(ID) ;
#define MAIN_CLOCK_END(ID) ;

#define INIT_CLOCK(ID) ;
#define CLOCK_START(ID) ;
#define CLOCK_END(ID, Main) ;
#endif

#include "vec2.h"

enum ModelType
{
    ModelTypeDiscreteTime,
    ModelTypeContinuousTime,
    ModelTypeDiscreteSpeeds
};

struct Edge
{
    u32 to;
    f32 cost;
};
    
struct Vertex
{
    u32 id;
    Vec2 position;
    u32 speed;
    std::vector<f32> speeds;
    std::vector<Edge> edges;
};

struct Graph
{
    std::vector<Vertex> vertices;
};

struct LowLevelNode
{
    u32 vertex;
    f32 arrival_time;
    f32 speed;
    f32 safe_interval_end;
    f32 fscore;
    LowLevelNode* parent;
};

static Vertex*
get_vertex(Graph* graph, u32 id)
{
    return &graph->vertices.at(id);
}

static Vertex*
get_vertex(Graph* graph, LowLevelNode* node)
{
    return get_vertex(graph, node->vertex);
}

static void
print_sipp_node(LowLevelNode* node)
{
    printf("{\n\tvertex: %d;\n\tarrival time: %f\n\tspeed: %f\n}", node->vertex, node->arrival_time, node->speed);
}

typedef std::vector<LowLevelNode> Path;

enum ActionType
{
    ACTION_TYPE_WAIT,
    ACTION_TYPE_MOVE
};

struct Interval
{
    f32 start;
    f32 end;
};

typedef Interval SafeInterval;
#define INF 10000000
#define MAX_QUEUE_SIZE 1000000
#define MAX_SUCCESSORS 1000

#define UNDEFINED_INTERVAL {INF, INF}

static Interval
intersection(Interval i1, Interval i2)
{
    return {fmax(i1.start, i2.start), fmin(i1.end, i2.end)};
}

static b32
interval_exists(Interval i, b32 inclusive = true)
{
    if ((FloatEq(i.start, INF) || i.start >= INF) &&
        (FloatEq(i.end, INF) || i.end >= INF))
    {
        return false;
    }
    else if (inclusive)
    {
        return FloatEq(i.start, i.end) || i.start < i.end;
    }
    else
    {
        return i.start < i.end;
    }
}

static b32
in(f32 x, Interval i)
{
    return (x >= i.start || FloatEq(x, i.start)) && (x <= i.end || FloatEq(x, i.end));
}

static b32
intervals_equal(Interval i1, Interval i2)
{
    return (FloatEq(i1.start, i2.start) &&
            FloatEq(i1.end, i2.end));
}

struct Action
{
    Interval interval;
    
    ActionType type;
    union
    {
        u32 wait_vertex;
        struct
        {
            u32 from;
            u32 to;
        } move;
    };
};

static b32
actions_equal(Action a1, Action a2)
{
    b32 result = false;

    if (intervals_equal(a1.interval, a2.interval) &&
        a1.type == a2.type)
    {
        if (a1.type == ACTION_TYPE_MOVE)
        {
            result = a1.move.from == a2.move.from && a1.move.to == a2.move.to;
        }
        else if (a1.type == ACTION_TYPE_WAIT)
        {
            result = a1.wait_vertex == a2.wait_vertex;
        }
        else InvalidCodePath;
    }
    
    return result;
}

struct Conflict
{
    b32 cardinal;
    b32 semi_cardinal;
    
    Interval interval;
    
    u32 agent_1_id;
    Action action_1;
    
    u32 agent_2_id;
    Action action_2;    
};

static b32
conflicts_equal(Conflict c1, Conflict c2)
{
    b32 result = false;

    if (intervals_equal(c1.interval, c2.interval) &&
        c1.agent_1_id == c2.agent_1_id &&
        c1.agent_2_id == c2.agent_2_id)
    {
        result = (actions_equal(c1.action_1, c2.action_1) &&
                  actions_equal(c1.action_2, c2.action_2));
    }
       
    return result;
}

struct Constraint
{
    Interval interval;
    u32 agent_id;
    ActionType type;
    u32 from;
    u32 to;
    u32 time;
};

static std::vector<Constraint>
create_constraints(Conflict conflict, ModelType model_type)
{
    std::vector<Constraint> constraints;
    constraints.resize(2);
    switch (model_type)
    {
        case ModelTypeDiscreteSpeeds:
        {
        constraints[0] = {conflict.interval,
                          conflict.agent_1_id, conflict.action_1.type,
                          conflict.action_1.move.from, conflict.action_1.move.to};
        constraints[1] = {conflict.interval,
                          conflict.agent_2_id, conflict.action_2.type,
                          conflict.action_2.move.from, conflict.action_2.move.to};
        }
        break;

        case ModelTypeDiscreteTime:
        {
            constraints[0] = {conflict.interval,
                              conflict.agent_1_id, conflict.action_1.type,
                              conflict.action_1.move.from, conflict.action_1.move.to,
                              (u32)conflict.interval.end};
            constraints[1] = {conflict.action_2.interval,
                              conflict.agent_2_id, conflict.action_2.type,
                              conflict.action_2.move.from, conflict.action_2.move.to,
                              (u32)conflict.interval.end};
        }
        break;
        
        default: InvalidCodePath;
    }
    return constraints;
}

static b32
constraints_equal(Constraint c1, Constraint c2)
{
    b32 result = false;
    
    if (intervals_equal(c1.interval, c2.interval) &&
        c1.agent_id == c2.agent_id &&
        c1.type == c2.type)
    {
        if (c1.type == ACTION_TYPE_MOVE)
        {
            result = c1.from == c2.from && c1.to == c2.to;
        }
        else if (c1.type == ACTION_TYPE_WAIT)
        {
            result = c1.from == c2.from;
        }
        else InvalidCodePath;
    }

    return result;
}

struct CBSNode
{
    b32 valid;
    CBSNode* parent;
    Constraint constraint;
    f32 cost;
};

struct FunctionProfile
{
    u64 start;
    u64 clock;
};

struct SegmentProfile
{
    u64 start;
    u64 clock;
};

struct Profiling
{
    FunctionProfile cbs;
    FunctionProfile create_cbs_node;
    FunctionProfile find_conflict;
    FunctionProfile sipp;
    FunctionProfile get_successors_ds_sipp;
    FunctionProfile add_astar_node;
    FunctionProfile action_time;
    FunctionProfile create_action_intervals;
    FunctionProfile constraint_sets_equal;

    FunctionProfile create_cbs_node_visited;
    
    FunctionProfile sipp_successors;
    FunctionProfile sipp_add_visited;
    FunctionProfile sipp_reconstruct_path;
    FunctionProfile sipp_iter_successor;
};

#if 0
static void
start_count(FunctionProfile* function_prof)
{
    function_prof->start = __rdtsc();
}

static void
end_count(FunctionProfile* function_prof)
{
    function_prof->clock += __rdtsc() - function_prof->start;
}
#else
#define start_count(_) ;
#define end_count(_) ;
#endif

struct RuntimeInfo
{
    u32 nodes_explored;
    u32 cardinal_nodes_explored;
    u32 semi_cardinal_nodes_explored;
    std::vector<Conflict> explored_conflicts;
    u32 visited_pruned;
    f32 execution_time;
    Profiling profiling;
};

static void
print_cbs_node(CBSNode* node)
{
    while (node->parent)
    {
        Constraint c = node->constraint;
        if (c.type == ACTION_TYPE_WAIT)
        {
            printf("{WAIT %d [%f, %f] %d} ", c.agent_id, c.interval.start, c.interval.end, c.from);
        }
        else
        {
            printf("{MOVE %d [%f, %f] %d %d} ", c.agent_id, c.interval.start, c.interval.end, c.from, c.to);
        }
        node = node->parent;
    }
    printf("\n");
}

struct NodeList
{
    NodeList* next;
    LowLevelNode* node;
};

struct NodeListPool
{
    NodeList* slots;
    u32 count;
};    

//Insert a node in the queue. Sorted on h-value.
static void
add_low_level_node(RuntimeInfo* runtime_info, NodeListPool* pool, NodeList* queue, LowLevelNode* node)
{
    start_count(&runtime_info->profiling.add_astar_node);

    NodeList* new_node = &pool->slots[pool->count++];
    new_node->next = 0;
    new_node->node = node;
    if (!queue->next)
    {
        queue->next = new_node;
    }
    else
    {
        NodeList* prev = 0;
        NodeList* first = queue->next;
        while(first && first->node->fscore < node->fscore)
        {
            prev = first;
            first = first->next;
        }

        if (!first)
        {
            prev->next = new_node;
        }
        else if (first == queue->next)
        {
            queue->next = new_node;
        }
        else
        {
            prev->next = new_node;
        }
        new_node->next = first;
    }
    
    end_count(&runtime_info->profiling.add_astar_node);
}

static LowLevelNode
create_ds_sipp_node(LowLevelNode* parent, u32 vertex,
                    f32 earliest_arrival_time, f32 safe_interval_end,
                    f32 hvalue, f32 speed)
{
    LowLevelNode new_node;
    new_node.parent = parent;
    new_node.vertex = vertex;
    new_node.arrival_time = earliest_arrival_time;
    new_node.safe_interval_end = safe_interval_end;
    new_node.fscore = earliest_arrival_time + hvalue;
    new_node.speed = speed;
    return new_node;
}

static LowLevelNode
create_astar_node(LowLevelNode* parent, u32 vertex,
                  f32 arrival_time,
                  f32 hvalue, u32 speed)
{
    LowLevelNode new_node;
    new_node.parent = parent;
    new_node.vertex = vertex;
    new_node.arrival_time = arrival_time;
    new_node.fscore = arrival_time + hvalue;
    new_node.speed = speed;
    return new_node;
}

struct TimeNode
{
    TimeNode* next;
    f32 time;
};

static b32
has_visited(TimeNode* node, f32 time)
{
    b32 result = false;
    while (node)
    {
        if (FloatEq(time, node->time))
        {
            result = true;
            break;
        }
        node = node->next;
    }
    return result;
}

static b32
can_wait_forever(std::vector<SafeInterval> safe_intervals, f32 from_time)
{
    b32 result = false;
    if (safe_intervals.size() == 0)
    {
        result = true;
    }
    else
    {
        for (SafeInterval interval : safe_intervals)
        {
            if ((interval.start <= from_time || FloatEq(interval.start, from_time)) && FloatEq(interval.end, INF))
            {
                result = true;
                break;
            }
        }
    }
    return result;
}

struct ComputeSafeIntervalsResult
{
    std::vector<std::vector<SafeInterval>> vertex;
    std::vector<std::vector<std::vector<SafeInterval>>> edge;
};

static void
remove_interval(std::vector<SafeInterval>& intervals, Interval conflict_interval)
{
    std::vector<SafeInterval> intervals_to_add;
    std::vector<u32> intervals_to_remove;
    std::vector<SafeInterval>::iterator it = intervals.begin();
    for (u32 safe_interval_index = 0;
           safe_interval_index < intervals.size();
           safe_interval_index++)
    {
        SafeInterval safe_interval = intervals[safe_interval_index];
        SafeInterval split_interval = intersection(conflict_interval, safe_interval);
        if (interval_exists(split_interval, false))
        {
            SafeInterval lower_interval = {safe_interval.start, split_interval.start};
            SafeInterval upper_interval = {split_interval.end, safe_interval.end};

            if (!interval_exists(lower_interval, false))
            {
                if (!interval_exists(upper_interval, false))
                {
                    intervals_to_remove.push_back(safe_interval_index);
                }
                else
                {
                    intervals_to_remove.push_back(safe_interval_index);
                    intervals_to_add.push_back(upper_interval);
                }
            }
            else if (!interval_exists(upper_interval, false))
            {
                intervals_to_remove.push_back(safe_interval_index);
                intervals_to_add.push_back(lower_interval);
            }
            else
            {
                intervals_to_remove.push_back(safe_interval_index);
                intervals_to_add.push_back(lower_interval);
                intervals_to_add.push_back(upper_interval);
            }
        }
    };

    for (u32 remove_index : intervals_to_remove)
    {
        intervals.erase(intervals.begin() + remove_index);
    }
    
    intervals.insert(intervals.begin(), intervals_to_add.begin(), intervals_to_add.end());
}

static u32
get_edge_index(u32 vertex_count, u32 from, u32 to)
{
    return vertex_count * from + to;
}

#define GetNode(pool_id, count) &pool_id[count++]

static b32
check_speed_change(Vertex* current, f32 current_speed, f32 next_speed)
{
    b32 result = false;

    std::vector<f32> speeds = current->speeds;

    std::vector<f32>::iterator it_current = std::find(speeds.begin(), speeds.end(), current_speed);
    std::vector<f32>::iterator it_next = std::find(speeds.begin(), speeds.end(), next_speed);

    int index_current = std::distance(speeds.begin(), it_current);
    int index_next = std::distance(speeds.begin(), it_next);

    int abs_diff = std::abs(index_current - index_next);
    if (abs_diff <= 1) result = true;

    return result;
}

static b32
check_maneuver_angle(Vertex* prev, Vertex* current, Vertex* next)
{
    // node id up to which every move is possible
    // in terms of angle change
    u32 id_possible = 40;//47;
    if (current->id <= id_possible) return true;

    b32 result = false;

    Vec2 A = prev->position;
    Vec2 B = current->position;
    Vec2 C = next->position;

    Vec2 BA = operator-(A, B);
    Vec2 BC = operator-(C, B);

    f32 angle_rad = acos(dot(BA, BC) / (length(BA) * length(BC)));
    f32 angle_deg = angle_rad * 180 / PI;

    // maneuver angle limit in the intersection part 
    f32 deg_possible = 95;
    if (angle_deg >= 95) result = true;

    return result;
}

static b32
action_possible(Vertex* prev, Vertex* current, f32 current_speed, Vertex* next, f32 next_speed)
{
	//TODO: implement this
    //return true;

    //b32 speed_OK = check_speed_change(current, current_speed, next_speed);
    b32 speed_OK = true;
    b32 angle_OK = true;//check_maneuver_angle(prev, current, next);

    b32 result = speed_OK && angle_OK;

    return result;


/*#if 0
    if (FloatEq(current_speed, current->speeds.back()) && FloatEq(next_speed, 0))
    {
        return false;
    }
    else
    {
        return true;
    }
#endif*/
}

static f32
action_time(RuntimeInfo* runtime_info, Vertex* current, f32 current_speed, Vertex* next, f32 next_speed)
{
    start_count(&runtime_info->profiling.action_time);
    f32 result = INF;
    
	f32 d = distance(next->position, current->position);
	result = 2 * d / (current_speed+next_speed)
   
	//old
	//f32 d = next->position.x - current->position.x;
    //if (FloatEq(current_speed, 0) && FloatEq(next_speed, 0))
    //{
    //    result = 2 * d / current->speeds[1];
    //}
    //else
    //{
    //    result = 2.0f * d / (current_speed + next_speed);
    //}

    end_count(&runtime_info->profiling.action_time);
    return result;
}

static u32
get_successors_ds_sipp(RuntimeInfo* runtime_info,
                       LowLevelNode* successors,
                       Graph* graph,
                       LowLevelNode* current_node, u32 goal, f32 goal_speed,
                       f32 (*heuristic)(Graph*, u32, u32),
                       ComputeSafeIntervalsResult* safe_intervals,
                       std::vector<std::vector<f32>>* visited)
{
    start_count(&runtime_info->profiling.get_successors_ds_sipp);

    u32 successor_count = 0;
    
    u32 current_vertex_id = current_node->vertex;
    Vertex* current_vertex = get_vertex(graph, current_vertex_id);
    f32 current_speed = current_node->speed;
    Vertex* prev = 0;
    if (current_node->parent)
    {
        prev = get_vertex(graph, current_node->parent);
    }

    Vertex* goal_vertex = get_vertex(graph, goal);
    
	//loop through current vertex edges
    for (Edge& edge : current_vertex->edges)
    {
        u32 neighbour_to = edge.to;
        Vertex* neighbour = get_vertex(graph, neighbour_to);
        //f32 hvalue = action_time(runtime_info, current_vertex, current_speed, goal_vertex, goal_speed);
        f32 hvalue = heuristic(graph, edge.to, goal_vertex->id);

        for (f32& neighbour_speed : neighbour->speeds)
        {
            if (action_possible(prev, current_vertex, current_speed, neighbour, neighbour_speed))
            {
                f32 neighbour_cost = action_time(runtime_info, current_vertex, current_speed, neighbour, neighbour_speed);

                f32 earliest_departure_time;
                f32 latest_departure_time;
                if (FloatEq(current_speed, 0))
                {
                    earliest_departure_time = current_node->arrival_time;
                    latest_departure_time = current_node->safe_interval_end;
                }
                else
                {
                    earliest_departure_time = latest_departure_time = current_node->arrival_time;
                }
                f32 earliest_arrival_time = earliest_departure_time + neighbour_cost;
                f32 latest_arrival_time = latest_departure_time + neighbour_cost;
                SafeInterval arrival_interval = {earliest_arrival_time, latest_arrival_time};

                for (SafeInterval& vertex_interval : safe_intervals->vertex[neighbour_to])
                {
                    SafeInterval safe_arrival_interval_vertex = intersection(arrival_interval, vertex_interval);
                    if (interval_exists(safe_arrival_interval_vertex))
                    {
                        SafeInterval safe_departure_interval_vertex = {safe_arrival_interval_vertex.start - neighbour_cost,
                                                                       safe_arrival_interval_vertex.end - neighbour_cost};

                        for (SafeInterval& edge_interval : safe_intervals->edge[current_node->vertex][neighbour_to])
                        {
                            SafeInterval safe_arrival_interval_edge = intersection(safe_arrival_interval_vertex, edge_interval);
                            SafeInterval safe_departure_interval_edge = intersection(safe_departure_interval_vertex, edge_interval);

                            if (!interval_exists(safe_arrival_interval_edge) ||
                                !interval_exists(safe_departure_interval_edge))
                            {
                                continue;
                            }
                            else if (in(safe_departure_interval_edge.start + neighbour_cost, safe_arrival_interval_edge))
                            {
                                earliest_arrival_time = safe_departure_interval_edge.start + neighbour_cost;
                            }
                            else if (in(safe_arrival_interval_edge.start - neighbour_cost, safe_departure_interval_edge))
                            {
                                earliest_arrival_time = safe_arrival_interval_edge.start;
                            }
                            else
                            {
                                continue;
                            }

                            b32 visited_time = false;
                            for (f32& time : (*visited)[neighbour_to])
                            {
                                if (FloatEq(earliest_arrival_time, time))
                                {
                                    visited_time = true;
                                    break;
                                }
                            }

                            if (!visited_time)
                            {
                                Assert(successor_count < MAX_SUCCESSORS);
                                successors[successor_count++] = create_ds_sipp_node(
                                    current_node, neighbour_to,
                                    earliest_arrival_time, vertex_interval.end,
                                    hvalue, neighbour_speed);
                            }
                        }
                    }
                }
            }
        }
    }

    end_count(&runtime_info->profiling.get_successors_ds_sipp);
    return successor_count;
}

static b32
move_valid(std::vector<Constraint>* constraints, Vertex* from, Vertex* to, u32 t)
{
    b32 result = true;
    
    for (Constraint& constraint : *constraints)
    {
        switch (constraint.type)
        {
            case ACTION_TYPE_MOVE:
            {
                if (constraint.from == from->id && constraint.to == to->id && constraint.time == t)
                {
                    result = false;
                    break;
                }
            }
            break;

            case ACTION_TYPE_WAIT:
            {
                if (constraint.from == from->id && constraint.time == t)
                {
                    result = false;
                    break;
                }
            }
            break;

            default: InvalidCodePath;
        }
    }

    return result;
}

static u32
get_successors_astar(RuntimeInfo* runtime_info,
                     LowLevelNode* successors,
                     Graph* graph,
                     LowLevelNode* current_node, u32 goal,
                     f32 (*heuristic)(Graph*, u32, u32),
                     std::vector<std::vector<f32>>* visited,
                     std::vector<Constraint>* constraints)
{
    start_count(&runtime_info->profiling.get_successors_ds_sipp);

    u32 successor_count = 0;
    
    u32 current_vertex_id = current_node->vertex;
    Vertex* current_vertex = get_vertex(graph, current_node);
    f32 current_speed = current_node->speed;
    u32 arrival_time = (u32)current_node->arrival_time + 1;
    Vertex* prev = 0;
    if (current_node->parent)
    {
        prev = get_vertex(graph, current_node->parent);
    }

    Vertex* goal_vertex = get_vertex(graph, goal);

    if (current_speed == 0 && move_valid(constraints, current_vertex, current_vertex, arrival_time))
    {
        b32 visited_time = false;
        for (f32 time : (*visited)[current_vertex_id])
        {
            if (FloatEq(arrival_time, time))
            {
                visited_time = true;
                break;
            }
        }

        if (!visited_time)
        {
            Assert(successor_count < MAX_SUCCESSORS);
            f32 hvalue = heuristic(graph, current_vertex_id, goal);
            successors[successor_count++] = create_astar_node(
                current_node, current_vertex_id,
                arrival_time, hvalue, 0);
        }
    }
    
    for (Edge& edge : current_vertex->edges)
    {
        u32 neighbour_to = edge.to;
        Vertex* neighbour = get_vertex(graph, neighbour_to);
        u32 neighbour_speed = neighbour->speed;
        f32 hvalue = heuristic(graph, neighbour_to, goal);
        u32 neighbour_cost = 1;

        if (move_valid(constraints, current_vertex, neighbour, arrival_time))
        {
            b32 visited_time = false;
            for (f32 time : (*visited)[neighbour_to])
            {
                if (FloatEq(arrival_time, time))
                {
                    visited_time = true;
                    break;
                }
            }

            if (!visited_time)
            {
                Assert(successor_count < MAX_SUCCESSORS);
                successors[successor_count++] = create_astar_node(
                    current_node, neighbour_to,
                    arrival_time, hvalue, neighbour_speed);
            }
        }
    }
    
    end_count(&runtime_info->profiling.get_successors_ds_sipp);
    return successor_count;
}

struct ActionIntervals
{
    Interval wait;
    Interval move;
};

static ActionIntervals
create_action_intervals(RuntimeInfo* runtime_info,
                        f32 arrival_time, Vertex* vertex, f32 speed, f32 safe_interval_end,
                        f32 next_arrival_time, Vertex* next_vertex, f32 next_speed,
                        b32 last_vertex)
{
    start_count(&runtime_info->profiling.create_action_intervals);
    
    ActionIntervals result;
    
    f32 arrival_time_vertex = arrival_time;
    f32 departure_time_vertex;
    f32 leave_time_vertex;
    if (last_vertex)
    {
        departure_time_vertex = safe_interval_end;
        leave_time_vertex = safe_interval_end;
    }
    else
    {
        f32 edge_cost = action_time(runtime_info, vertex, speed, next_vertex, next_speed);
        departure_time_vertex = next_arrival_time - edge_cost;
        leave_time_vertex = next_arrival_time;
    }
    result.wait = {arrival_time_vertex, departure_time_vertex};
    result.move = {departure_time_vertex, leave_time_vertex};

    end_count(&runtime_info->profiling.create_action_intervals);
    return result;
}

global_variable std::vector<LowLevelNode> sipp_node_pool = std::vector<LowLevelNode>(MAX_QUEUE_SIZE);
global_variable NodeList* node_list_pool = (NodeList*)malloc(MAX_QUEUE_SIZE * sizeof(NodeList));
global_variable LowLevelNode* successors = (LowLevelNode*)malloc(MAX_SUCCESSORS * sizeof(LowLevelNode));

static Path
ds_sipp(RuntimeInfo* runtime_info,
        Graph* graph,
        u32 start_vertex, f32 start_speed, u32 goal_vertex, f32 goal_speed,
        f32 (*heuristic)(Graph*, u32, u32),
        ComputeSafeIntervalsResult* safe_intervals)
{
    start_count(&runtime_info->profiling.sipp);
    
    u32 node_count = 0;
    
    Path result;

    NodeListPool pool;
    pool.slots = node_list_pool;
    pool.count = 0;

    NodeList* queue = &pool.slots[pool.count++];
    *queue = {};
    
    f32 root_safe_interval_end = safe_intervals->vertex[start_vertex][0].end;
    if (safe_intervals->vertex[start_vertex][0].start > 0)
    {
        root_safe_interval_end = 0;
    }
    LowLevelNode root = create_ds_sipp_node(0, start_vertex,
                                        0, root_safe_interval_end,
                                        heuristic(graph, start_vertex, goal_vertex), start_speed);
    LowLevelNode* root_node = GetNode(sipp_node_pool, node_count);
    *root_node = root;
    add_low_level_node(runtime_info, &pool, queue, root_node);    

    std::vector<std::vector<f32>> visited;
    visited.resize(graph->vertices.size());
    
    u32 expansions = 0;
    while(queue->next)
    {        
        expansions++;
        LowLevelNode* current_node = queue->next->node;
        queue->next = queue->next->next;
        
        u32 current_vertex = current_node->vertex;
        f32 arrival_time = current_node->arrival_time;
        
        Assert(current_vertex < graph->vertices.size());
        
		//check if path planning is finished. If so, create result.
        if (current_vertex == goal_vertex && FloatEq(current_node->speed, goal_speed) && can_wait_forever(safe_intervals->vertex[current_vertex], arrival_time))
        {
            start_count(&runtime_info->profiling.sipp_reconstruct_path);
            
            // NOTE: Reconstruct path from best_path_vertex
            std::vector<LowLevelNode> path;
            {
                LowLevelNode* prev = current_node;
                LowLevelNode* next = current_node;
                while (next)
                {
                    next = next->parent;
                    if (next)
                    {
                        Assert(prev->arrival_time > next->arrival_time);
                    }
                    path.push_back(*prev);
                    prev = next;
                }
            }
            
            for (u32 vertex_index = 0;
                 vertex_index < path.size() - 1;
                 vertex_index++)
            {
                LowLevelNode prev = path[path.size() - vertex_index - 1];
                LowLevelNode next = path[path.size() - vertex_index - 2];
                Vertex* prev_vertex = get_vertex(graph, &prev);
                Vertex* next_vertex = get_vertex(graph, &next);
                ActionIntervals action_intervals = create_action_intervals(
                    runtime_info,
                    prev.arrival_time, prev_vertex, prev.speed, prev.safe_interval_end,
                    next.arrival_time, next_vertex, next.speed, false);
                if (interval_exists(action_intervals.wait, false) &&
                    action_intervals.wait.start + 0.001 < action_intervals.wait.end)
                {
                    result.push_back(prev);
                }
                if (interval_exists(action_intervals.move))
                {
                    LowLevelNode move_node = prev;
                    move_node.arrival_time = action_intervals.move.start;
                    result.push_back(move_node);
                }
            }
            result.push_back(path.front());

            end_count(&runtime_info->profiling.sipp_reconstruct_path);
            break;
        }


		//Get successors and add them to queue
        start_count(&runtime_info->profiling.sipp_successors);
        
        start_count(&runtime_info->profiling.sipp_iter_successor);
        u32 successor_count = get_successors_ds_sipp(runtime_info,
                                                     successors,
                                                     graph, current_node, goal_vertex, goal_speed,
                                                     heuristic, safe_intervals,
                                                     &visited);
        end_count(&runtime_info->profiling.sipp_iter_successor);
        for (u32 successor_index = 0;
             successor_index < successor_count;
             successor_index++)
        {
            LowLevelNode successor = successors[successor_index];
            Assert(node_count < MAX_QUEUE_SIZE);
            LowLevelNode* new_node = GetNode(sipp_node_pool, node_count); //&node_pool[node_count++];
            *new_node = successor;
            add_low_level_node(runtime_info, &pool, queue, new_node);

            start_count(&runtime_info->profiling.sipp_add_visited);
            visited[successor.vertex].push_back(successor.arrival_time);
            end_count(&runtime_info->profiling.sipp_add_visited);

            node_count++;
        }

        end_count(&runtime_info->profiling.sipp_successors);
    }
    
    end_count(&runtime_info->profiling.sipp);
    return result;
}

static Path
astar(RuntimeInfo* runtime_info,
      Graph* graph,
      u32 start_vertex, u32 goal_vertex,
      f32 (*heuristic)(Graph*, u32, u32), std::vector<Constraint>* constraints)
{
    start_count(&runtime_info->profiling.sipp);
    
    u32 node_count = 0;
    
    Path result;

    NodeListPool pool;
    pool.slots = node_list_pool;
    pool.count = 0;

    NodeList* queue = &pool.slots[pool.count++];
    *queue = {};

    LowLevelNode root = create_astar_node(0, start_vertex, 0,
                                          heuristic(graph, start_vertex, goal_vertex), graph->vertices[start_vertex].speed);
    LowLevelNode* root_node = GetNode(sipp_node_pool, node_count);
    *root_node = root;
    add_low_level_node(runtime_info, &pool, queue, root_node);    

    std::vector<std::vector<f32>> visited;
    visited.resize(graph->vertices.size());
    
    u32 expansions = 0;
    while(queue->next)
    {        
        expansions++;
        LowLevelNode* current_node = queue->next->node;
        queue->next = queue->next->next;
        
        u32 current_vertex = current_node->vertex;
        f32 arrival_time = current_node->arrival_time;
        
        Assert(current_vertex < graph->vertices.size());

        // TODO: add can wait forever
        if (current_vertex == goal_vertex)
        {
            start_count(&runtime_info->profiling.sipp_reconstruct_path);
            
            // NOTE: Reconstruct path from best_path_vertex
            std::vector<LowLevelNode> path;
            {
                LowLevelNode* prev = current_node;
                LowLevelNode* next = current_node;
                while (next)
                {
                    next = next->parent;
                    if (next)
                    {
                        Assert(prev->arrival_time > next->arrival_time);
                    }
                    result.push_back(*prev);
                    prev = next;
                }
            }

            std::reverse(result.begin(), result.end());
            
            end_count(&runtime_info->profiling.sipp_reconstruct_path);
            break;
        }

        start_count(&runtime_info->profiling.sipp_successors);
        
        start_count(&runtime_info->profiling.sipp_iter_successor);
        u32 successor_count = get_successors_astar(runtime_info,
                                                   successors,
                                                   graph, current_node, goal_vertex,
                                                   heuristic,
                                                   &visited, constraints);
        end_count(&runtime_info->profiling.sipp_iter_successor);
        for (u32 successor_index = 0;
             successor_index < successor_count;
             successor_index++)
        {
            LowLevelNode successor = successors[successor_index];
            Assert(node_count < MAX_QUEUE_SIZE);
            LowLevelNode* new_node = GetNode(sipp_node_pool, node_count); //&node_pool[node_count++];
            *new_node = successor;
            add_low_level_node(runtime_info, &pool, queue, new_node);

            start_count(&runtime_info->profiling.sipp_add_visited);
            visited[successor.vertex].push_back(successor.arrival_time);
            end_count(&runtime_info->profiling.sipp_add_visited);

            node_count++;
        }

        end_count(&runtime_info->profiling.sipp_successors);
    }
    
    end_count(&runtime_info->profiling.sipp);
    return result;
}


struct AgentInfo
{
    u32 id;

    u32 start_vertex;
    f32 start_speed;
    u32 goal_vertex;
    f32 goal_speed;
    
    f32 radius;
};

static u32
get_agent_index(std::vector<AgentInfo>* agents, u32 agent_id)
{
    s32 result = -1;
    for (u32 agent_index = 0;
         agent_index < agents->size();
         agent_index++)
    {
        if (agents->at(agent_index).id == agent_id)
        {
            result = agent_index;
            break;
        }
    }
    Assert(result >= 0);
    return (u32)result;
}

static u32
get_agent_index(AgentInfo* agents, u32 agent_count, u32 agent_id)
{
    s32 result = -1;
    for (u32 agent_index = 0;
         agent_index < agent_count;
         agent_index++)
    {
        if (agents[agent_index].id == agent_id)
        {
            result = agent_index;
            break;
        }
    }
    Assert(result >= 0);
    return (u32)result;
}

struct Solution
{
    RuntimeInfo runtime_info;
    std::vector<Path> paths;
};

static ComputeSafeIntervalsResult
compute_safe_intervals(Graph* graph, std::vector<Constraint> constraints)
{
    ComputeSafeIntervalsResult result;
    
    u32 safe_intervals_vertex_size = graph->vertices.size();
    result.vertex.resize(safe_intervals_vertex_size);
    for (u32 safe_interval_index = 0;
         safe_interval_index < safe_intervals_vertex_size;
         safe_interval_index++)
    {
        result.vertex[safe_interval_index].push_back({0, INF});
    }
    u32 safe_intervals_edge_size = graph->vertices.size();
    result.edge.resize(safe_intervals_edge_size);
    for (u32 vertex_index = 0;
         vertex_index < safe_intervals_edge_size;
         vertex_index++)
    {
        result.edge[vertex_index].resize(safe_intervals_edge_size);
        for (u32 edge_index = 0;
             edge_index < safe_intervals_edge_size;
             edge_index++)
        {
            result.edge[vertex_index][edge_index].push_back({0, INF});
        }
    }
    
    for (Constraint constraint : constraints)
    {
        // TODO: ta bort kollisions-intervallet från safe intervals
        if (constraint.type == ACTION_TYPE_WAIT)
        {
            u32 vertex_index = constraint.from;
            Interval interval = constraint.interval;
                
            remove_interval(result.vertex[vertex_index], interval);
        }
        else if (constraint.type == ACTION_TYPE_MOVE)
        {
            u32 from_vertex_index = constraint.from;
            u32 to_vertex_index = constraint.to;
            Interval interval = constraint.interval;
                
            remove_interval(result.edge[from_vertex_index][to_vertex_index], interval);
        }
    };

    return result;
}

std::vector<std::vector<f32>> distanceLookup;
f32 h(Graph* g, u32 vertex, u32 goal)
{
    //return distance(g->vertices[vertex].position, g->vertices[goal].position);
    return distanceLookup[vertex][goal];
}        

static f32
cost_path(Path path)
{
    return path[path.size() - 1].arrival_time;
}

static f32
cost(std::vector<Path>* paths)
{
    f32 result = 0;

    for (Path path : *paths)
    {
        result += cost_path(path);
    }
    
    return result;
}

static f32
compute_new_cost(std::vector<Path>* path_buffer, u32 agent_id, Path new_path)
{
    f32 previous_cost = cost(path_buffer);
    return previous_cost - cost_path(path_buffer->at(agent_id)) + cost_path(new_path);
}

static void
add_cbs_node(std::list<CBSNode*>& queue, CBSNode* node)
{
    auto it = queue.begin();
    while (it != queue.end())
    {
        if (node->cost < (*it)->cost)
        {
            break;
        }
        ++it;
    }
    queue.insert(it, node);
}

struct MakeConstraintResult
{
    Constraint* data;
    u32 count;
};
    
static std::vector<Constraint>
make_constraints(CBSNode* node, u32 agent_id)
{
    std::vector<Constraint> result;

    CBSNode* p = node;
    while (p->parent)
    {
        if (p->constraint.agent_id == agent_id)
        {
            result.push_back(p->constraint);
        }
        p = p->parent;
    }

    return result;
}

// Source: Guide to anticipatory collision avoidance Chapter 19
struct Collision
{
    f32 t;
    b32 ok;
};

static Collision
get_collision_time(Vec2 pos_1, f32 radius_1, Vec2 velocity_1,
                   Vec2 pos_2, f32 radius_2, Vec2 velocity_2)
{
    Collision result = {};

    f32 radius = radius_1 + radius_2;
    Vec2 w = pos_2 - pos_1;
    f32 c = dot(w, w) - radius * radius;
    if (c < 0)
    {
        result.t = 0;
        result.ok = true;
    }
    else
    {
        Vec2 v = velocity_1 - velocity_2;
        f32 a = dot(v, v);
        f32 b = dot(w, v);
        f32 discr = b * b - a * c;
        if (FloatEq(discr, 0) || discr < 0)
        {
            result.t = -1;
            result.ok = false;
        }
        else
        {
            f32 tau = (b - sqrt(discr)) / a;
            if (tau < 0)
            {
                result.t = -1;
                result.ok = false;
            }
            else
            {
                result.t = tau;
                result.ok = true;
            }
        }
    }
    
    return result;
}

static Interval
get_collision_interval(Vec2 pos_1, f32 radius_1, Vec2 velocity_1,
                       Vec2 pos_2, f32 radius_2, Vec2 velocity_2,
                       Interval action_interval)
{
    Interval result = UNDEFINED_INTERVAL;
    
    Collision c = get_collision_time(pos_1, radius_1, velocity_1,
                                     pos_2, radius_2, velocity_2);
    f32 t = c.t;
    
    if (c.ok && action_interval.start + t < action_interval.end)
    {
        Vec2 search_pos_1 = pos_1 + velocity_1 * t;
        Vec2 search_pos_2 = pos_2 + velocity_2 * t;
            
        result.start = action_interval.start + t;
        // NOTE: Collision detect until no collision = result.end
        f32 delta = 0.1f;
        while (result.start + t < action_interval.end)
        {
            t += delta;
            search_pos_1 += velocity_1 * delta;
            search_pos_2 += velocity_2 * delta;

#if 1
            c = get_collision_time(search_pos_1, radius_1, velocity_1,
                                   search_pos_2, radius_2, velocity_2);
            if (!c.ok) break;
#else
            f32 dist = distance(search_pos_1, search_pos_2);
            if (dist > radius_1 + radius_2)
            {
                break;
            }
#endif
        }

        if (result.start + t > action_interval.end)
        {
            result.end = action_interval.end;
        }
        else
        {
            result.end = result.start + t;
        }
    }

    if (result.start + 0.001 > result.end)
    {
        result = UNDEFINED_INTERVAL;
    }
    
    return result;
}

struct FindConflictResult
{
    b32 no_conflict_found;
    Conflict conflict;
};

static FindConflictResult
find_conflict(RuntimeInfo* runtime_info, std::vector<Path>& path_buffer,
              Graph* graph, std::vector<AgentInfo>& agents, CBSNode* current_node)
{
    start_count(&runtime_info->profiling.find_conflict);
    
    FindConflictResult result = {};
    result.no_conflict_found = true;

    for (u32 agent_index = 0;
         agent_index < agents.size();
         agent_index++)
    {
        AgentInfo& agent = agents[agent_index];
        LowLevelNode* path = path_buffer[agent.id].data();
        u32 path_length = path_buffer[agent.id].size();
        for (u32 node_index = 0;
#if ROAD_EXPERIMENT
             node_index < path_length - 2;
#else
             node_index < path_length - 1;
#endif
             node_index++)
        {
            LowLevelNode prev_node = path[node_index];
            LowLevelNode next_node = path[node_index + 1];
            b32 is_move_action = prev_node.vertex != next_node.vertex;
            Interval action_interval = {prev_node.arrival_time, next_node.arrival_time};
            
            for (u32 other_agent_index = agent_index + 1;
                 other_agent_index < agents.size();
                 other_agent_index++)
            {
                AgentInfo& other_agent = agents[other_agent_index];
                LowLevelNode* other_path = path_buffer[other_agent.id].data();
                u32 other_path_length = path_buffer[other_agent.id].size();
                for (u32 other_node_index = 0;
#if ROAD_EXPERIMENT
                     other_node_index < other_path_length - 2;
#else
                     other_node_index < other_path_length - 1;
#endif
                     other_node_index++)
                {
                    LowLevelNode prev_other_node = other_path[other_node_index];
                    LowLevelNode next_other_node = other_path[other_node_index + 1];
                    b32 other_is_move_action = prev_other_node.vertex != next_other_node.vertex;
                    Interval other_action_interval = {prev_other_node.arrival_time, next_other_node.arrival_time};

                    Interval intersecting_interval = intersection(action_interval, other_action_interval);
                    if (interval_exists(intersecting_interval) &&
                        intersecting_interval.start + 0.1 < intersecting_interval.end)
                    {
                        Interval collision_interval = UNDEFINED_INTERVAL;
                        
                        Vertex* prev_vertex = get_vertex(graph, &prev_node);
                        Vec2 prev_position = prev_vertex->position;
                        Vertex* prev_other_vertex = get_vertex(graph, &prev_other_node);
                        Vec2 prev_other_position = prev_other_vertex->position;
                        if (is_move_action)
                        {
                            Vertex* next_vertex = get_vertex(graph, &next_node);
                            Vec2 next_position = next_vertex->position;
                            Vec2 velocity = compute_velocity(prev_position, next_position, prev_node.speed, next_node.speed);

                            if (other_is_move_action)
                            {
                                Vertex* next_other_vertex = get_vertex(graph, &next_other_node);
                                Vec2 next_other_position = next_other_vertex->position;   
                                Vec2 other_velocity = compute_velocity(prev_other_position, next_other_position, prev_other_node.speed, next_other_node.speed);

                                collision_interval = get_collision_interval(prev_position, agent.radius, velocity,
                                                                            prev_other_position, other_agent.radius, other_velocity,
                                                                            intersecting_interval);
                            }
                            else
                            {
                                if (next_position == prev_other_position)
                                {
                                    collision_interval = get_collision_interval(prev_position, agent.radius, velocity,
                                                                                prev_other_position, other_agent.radius, {},
                                                                                intersecting_interval);
                                }
                            }
                        }
                        else if (other_is_move_action)
                        {
                            Vertex* next_other_vertex = get_vertex(graph, &next_other_node);
                            Vec2 next_other_position = next_other_vertex->position;
                            Vec2 other_velocity = compute_velocity(prev_other_position, next_other_position, prev_other_node.speed, next_other_node.speed);
                            if (prev_position == next_other_position)
                            {
                                collision_interval = get_collision_interval(prev_position, agent.radius, {},
                                                                            prev_other_position, other_agent.radius, other_velocity,
                                                                            intersecting_interval);
                            }
                        }

                        if (interval_exists(collision_interval))
                        {
                            Conflict conflict = {};
                            conflict.interval = collision_interval;

                            conflict.agent_1_id = agent.id;
                            if (is_move_action)
                            {
                                conflict.action_1 = {action_interval, ACTION_TYPE_MOVE};
                                conflict.action_1.move.from = prev_node.vertex;
                                conflict.action_1.move.to = next_node.vertex;
                            }
                            else
                            {
                                conflict.action_1 = {action_interval, ACTION_TYPE_WAIT};
                                conflict.action_1.wait_vertex = prev_node.vertex;
                            }
                            
                            conflict.agent_2_id = other_agent.id;
                            if (other_is_move_action)
                            {
                                conflict.action_2 = {other_action_interval, ACTION_TYPE_MOVE};
                                conflict.action_2.move.from = prev_other_node.vertex;
                                conflict.action_2.move.to = next_other_node.vertex;
                            }
                            else
                            {
                                conflict.action_2 = {other_action_interval, ACTION_TYPE_WAIT};
                                conflict.action_2.wait_vertex = prev_other_node.vertex;
                            }
                            
                            if (result.no_conflict_found)
                            {
                                result.conflict = conflict;
                                result.no_conflict_found = false;
                                end_count(&runtime_info->profiling.find_conflict);
                                return result;
                            }
                        }
                    }
                }
            }
        }
    }
    
    end_count(&runtime_info->profiling.find_conflict);
    return result;
}

static b32
constraint_sets_equal(RuntimeInfo* runtime_info, std::vector<Constraint>* set_1, std::vector<Constraint>* set_2)
{
    start_count(&runtime_info->profiling.constraint_sets_equal);
    
    b32 result = false;
    
    if (set_1->size() == set_2->size())
    {
        u32* checked = (u32*)calloc(set_1->size(), sizeof(u32));
        
        for (u32 constraint_index = 0;
             constraint_index < set_1->size();
             constraint_index++)
        {
            if (!checked[constraint_index])
            {
                Constraint c1 = set_1->data()[constraint_index];
                Constraint c2 = set_2->data()[constraint_index];
                if (constraints_equal(c1, c2))
                {
                    checked[constraint_index] = true;
                }
            }
        }

        result = true;
        for (u32 constraint_index = 0;
             constraint_index < set_1->size();
             constraint_index++)
        {
            if (!checked[constraint_index])
            {
                result = false;
                break;
            }
        }

        free(checked);
    }

    end_count(&runtime_info->profiling.constraint_sets_equal);
    return result;
}

static void
create_cbs_node(RuntimeInfo* runtime_info,
                CBSNode* new_node,
                CBSNode* parent, std::vector<std::vector<std::vector<Constraint>>>* visited,
                Constraint constraint, Graph* graph, std::vector<AgentInfo>* agents,
                std::vector<Path>* path_buffer,
                ModelType model_type)
{
    start_count(&runtime_info->profiling.create_cbs_node);
    
    u32 agent_id = constraint.agent_id;
#if 0
    std::vector<Constraint> test = make_constraints(parent, agent_id);
    Assert(!constraint_in_set(constraint, test));
#endif
    new_node->valid = true;
    new_node->parent = parent;
    new_node->constraint = constraint;
    std::vector<Constraint> constraints = make_constraints(new_node, agent_id);
#if 1
    start_count(&runtime_info->profiling.create_cbs_node_visited);
    std::vector<std::vector<Constraint>>* constraint_sets = &visited->data()[agent_id];
    for (u32 constraint_set_index = 0;
         constraint_set_index < constraint_sets->size();
         constraint_set_index++)
    {
        std::vector<Constraint>* constraint_set = &constraint_sets->data()[constraint_set_index];
        if (constraint_sets_equal(runtime_info, constraint_set, &constraints))
        {
            runtime_info->visited_pruned++;
            new_node->valid = false;
            return;
        }
    }

    visited->at(agent_id).push_back(constraints);
    end_count(&runtime_info->profiling.create_cbs_node_visited);
#endif
    switch (model_type)
    {
        case ModelTypeDiscreteSpeeds:
        {
            ComputeSafeIntervalsResult safe_intervals = compute_safe_intervals(graph, constraints);

            u32 agent_index = get_agent_index(agents, agent_id);
            AgentInfo agent = agents->at(agent_index);
            Path new_path = ds_sipp(runtime_info,
                                    graph,
                                    agent.start_vertex, agent.start_speed,
                                    agent.goal_vertex, agent.goal_speed,
                                    h, &safe_intervals);
            if (new_path.size() == 0)
            {
                new_node->valid = false;
            }
            else
            {
                new_node->parent = parent;
                new_node->constraint = constraint;
                new_node->cost = compute_new_cost(path_buffer, agent_id, new_path);
            }
        }
    }
    
    end_count(&runtime_info->profiling.create_cbs_node);
}

static Solution
high_level(Graph* graph, std::vector<AgentInfo>& agents,
           ModelType model_type)
{
    std::vector<CBSNode> cbs_node_pool(MAX_QUEUE_SIZE);

    Solution result;
    result.paths.resize(agents.size());

    u32 nodes_in_use = 0;

    std::list<CBSNode*> queue;

    RuntimeInfo runtime_info = {};
    
    std::vector<std::vector<std::vector<Constraint>>> visited;
    visited.resize(agents.size());

    // NOTE: each index in the path_buffer corresponds to an agent id
    std::vector<Path> path_buffer;
    path_buffer.resize(agents.size());

    CBSNode* root = GetNode(cbs_node_pool, nodes_in_use);
    root->valid = true;
    root->parent = 0;
    root->constraint = {};
    root->cost = 0;
    
    add_cbs_node(queue, root);

    u32 level = 0;
    u32 printed_nodes = 0;

    auto start_execution_time = std::chrono::high_resolution_clock::now();
    while (queue.size() > 0)
    {
        start_count(&runtime_info.profiling.cbs);
        runtime_info.nodes_explored++;
        CBSNode* current_node;

        runtime_info.nodes_explored++;
        current_node = queue.front();
        queue.pop_front();

        b32 all_paths_valid = true;
        for (AgentInfo agent : agents)
        {
            switch (model_type)
            {
                case ModelTypeDiscreteSpeeds:
                {
                    std::vector<Constraint> constraints = make_constraints(current_node, agent.id);
                    ComputeSafeIntervalsResult safe_intervals = compute_safe_intervals(graph, constraints);
            
                    path_buffer[agent.id] = ds_sipp(&runtime_info,
                                                    graph,
                                                    agent.start_vertex, agent.start_speed,
                                                    agent.goal_vertex, agent.goal_speed,
                                                    h, &safe_intervals);
                }
                break;
                
                case ModelTypeDiscreteTime:
                {
                    std::vector<Constraint> constraints = make_constraints(current_node, agent.id);
                    path_buffer[agent.id] = astar(&runtime_info,
                                                  graph,
                                                  agent.start_vertex, agent.goal_vertex,
                                                  h, &constraints);
                }
                break;
                
                default: InvalidCodePath;
            }
            
            if (path_buffer[agent.id].size() == 0)
            {
                all_paths_valid = false;
                break;
            }
        }
        
        if (all_paths_valid)
        {
            FindConflictResult find_conflict_result = find_conflict(&runtime_info, path_buffer, graph, agents, current_node);
            if (find_conflict_result.no_conflict_found)
            {
                result.paths = path_buffer;
                break;
            }
            Conflict new_conflict = find_conflict_result.conflict;

            std::vector<Constraint> constraints = create_constraints(new_conflict, model_type);
            for (Constraint constraint : constraints)
            {
                if(nodes_in_use >= MAX_QUEUE_SIZE) {throw std::runtime_error("error");}
                Assert(nodes_in_use < MAX_QUEUE_SIZE);
                CBSNode* node = GetNode(cbs_node_pool, nodes_in_use);
                create_cbs_node(
                    &runtime_info,
                    node,
                    current_node, &visited,
                    constraint, graph, &agents,
                    &path_buffer, model_type);
                if (node->valid)
                {
                    add_cbs_node(queue, node);
                }
            }
        }
        end_count(&runtime_info.profiling.cbs);
        auto end_execution_time = std::chrono::high_resolution_clock::now();
        runtime_info.execution_time = std::chrono::duration<f32>(end_execution_time - start_execution_time).count();
        if (runtime_info.execution_time > MAX_DURATION)
        {
            break;
        }
    }
    
    auto end_execution_time = std::chrono::high_resolution_clock::now();
    runtime_info.execution_time = std::chrono::duration<f32>(end_execution_time - start_execution_time).count();    

    result.runtime_info = runtime_info;
    return result;
}
