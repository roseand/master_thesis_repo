#pragma once

#include <iostream>
#include <fstream>
#include <string>

enum class MapType
{
    Highway,
    Exit,
    Roundabout,
    Intersection,
    Entrance
};

enum class exp1ClassificationType
{
    ClassA,
    ClassB
};

enum class DataType
{
    train,
    validation,
    test
};

std::string get_map_csv_file_name(MapType map_type)
{
    std::string name;

    switch (map_type)
    {
        case MapType::Highway:
            name = "highway.csv";
            break;
        case MapType::Exit:
            name = "exit.csv";
            break;
        case MapType::Roundabout:
            name = "roundabout.csv";
            break;
        case MapType::Intersection:
            name = "intersection.csv";
            break;
    }

    return name;
}

void write_map_csv(Graph graph, MapType map_type)
{
    std::string file_name = get_map_csv_file_name(map_type);
    std::string maps_path = "../data_exp1/maps/";
    std::string file_path = maps_path + file_name;

    std::ofstream file;
    file.open(file_path);

    file << "id,x,y,neighbours\n";

    for (int i = 0; i < graph.vertices.size(); i++)
    {
        Vertex n = graph.vertices[i];

        std::string neighbours = "\"(";
        for (int j = 0; j < n.edges.size(); j++)
        {
            neighbours += std::to_string(n.edges[j].to);
            if (j != n.edges.size() - 1) neighbours += ",";
        }
        neighbours += ")\"";

        file
			<< n.id << ","
			<< n.position.x << ","
			<< n.position.y << ","
			<< neighbours << "\n";
    }

    file.close();
}



void exp1_write_train_csv(std::vector<AgentInfo> agents, exp1ClassificationType class_type, DataType data_type, std::string file_name)
{
    std::string class_path;
    std::string data_folder;
    std::string class_folder;

    switch (data_type)
    {
    case DataType::train:
        data_folder = "train/";
        break;
    case DataType::validation:
        data_folder = "validation/";
        break;
    }

    switch (class_type)
    {
    case exp1ClassificationType::ClassA:
        class_folder = "classA/";
        break;
    case exp1ClassificationType::ClassB:
        class_folder = "classB/";
        break;
    }

    class_path = "../data_exp1/" + data_folder + class_folder;

    std::string file_path = class_path + file_name + ".csv";

    std::ofstream file;
    file.open(file_path);

    file << "agent_id,start,goal\n";

    for (AgentInfo agent : agents)
    {
        file
            << agent.id << ","
            << agent.start_vertex << ","
            << agent.goal_vertex << "\n";
    }

    file.close();
}

void exp1_write_test_csv(std::vector<AgentInfo> agents, std::string file_name)
{
    std::string test_path = "../data_exp1/test/";

    std::string file_path = test_path + file_name + ".csv";

    std::ofstream file;
    file.open(file_path);

    file << "agent_id,start,goal\n";

    for (AgentInfo agent : agents)
    {
        file
            << agent.id << ","
            << agent.start_vertex << ","
            << agent.goal_vertex << "\n";
    }

    file.close();
}
