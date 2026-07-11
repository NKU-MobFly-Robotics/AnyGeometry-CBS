#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <string>
#include <map>
#include <fstream>
#include <chrono>
#include <thread>
#include <atomic>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include "timer.hpp"

#include "AG_CBS.hpp"
#include "environment.hpp"
#include "common_utils.hpp"

using libMultiRobotPlanning::AG_CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::computeShapeSheet;
using libMultiRobotPlanning::computeTouchSpotsList;

int main(int argc, char ** argv)
{
    namespace po = boost::program_options;
    std::string inputFile;
    std::string outputFile;
    bool disappearAtGoal = false;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("input,i", po::value<std::string>(&inputFile)->default_value("../src/maps/warehouse_agent6_1.yaml"), "input file (YAML)")
        ("output,o", po::value<std::string>(&outputFile)->default_value("./output.yaml"), "output file (YAML)")
        ("disappear-at-goal", po::bool_switch(&disappearAtGoal), "make agents to disappear at goal rather than staying there");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cout << desc << std::endl;
        return 1;
    }

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    YAML::Node config;
    try {
        config = YAML::LoadFile(inputFile);
    } catch (std::exception& e) {
        std::cerr << "Error loading input file: " << e.what() << std::endl;
        return 1;
    }

    std::unordered_set<Location> obstacles;
    std::vector<Location> goals;
    std::vector<State> startStates;
    std::vector<std::vector<Location>> shapes;

    const auto& dim = config["map"]["dimensions"];
    int dimx = dim[0].as<int>();
    int dimy = dim[1].as<int>();

    for (const auto& node : config["map"]["obstacles"]) {
        obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
    }

    size_t agentNum = 0;
    for (const auto& node : config["agents"]) {
        const auto& start = node["start"];
        const auto& goal = node["goal"];
        
        startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>(), 1));
        goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
        
        std::vector<Location> shapeTemp;
        for (const auto& shape : node["shape"]) {
            shapeTemp.emplace_back(Location(shape[0].as<int>(), shape[1].as<int>()));
        }
        shapes.emplace_back(shapeTemp);
        agentNum++;
    }

    // Print agent shapes
    for (size_t i = 0; i < shapes.size(); ++i) {
        std::cout << "Agent " << i << " shape:";
        for (const auto& pt : shapes[i]) {
            std::cout << " [" << pt.x << "," << pt.y << "]";
        }
        std::cout << std::endl;
    }

    // Check for identical start states
    std::unordered_set<State> startStatesSet;
    for (const auto& s : startStates) {
        if (startStatesSet.find(s) != startStatesSet.end()) {
            std::cout << "Identical start states detected -> no solution!" << std::endl;
            return 0;
        }
        startStatesSet.insert(s);
    }

    // Compute shape sheets and touch spots using common utilities
    auto shapesSheet = computeShapeSheet(shapes, agentNum);
    auto h1List = computeTouchSpotsList(shapesSheet);

    Environment mapf(dimx, dimy, obstacles, goals, disappearAtGoal, shapes, shapesSheet, h1List);
    AG_CBS<State, Action, float, Conflict, Constraints, Environment> cbs(mapf);
    std::vector<PlanResult<State, Action, float>> solution;

    Timer timer;
    bool success = cbs.search(startStates, solution);
    timer.stop();

    // Ensure output directory exists
    std::string outputDir = outputFile.substr(0, outputFile.find_last_of("/"));
    if (!outputDir.empty()) {
        std::string mkdirCmd = "mkdir -p " + outputDir;
        system(mkdirCmd.c_str());
    }

    if (success) {
        std::cout << "Planning successful!" << std::endl;
        float cost = 0;
        float makespan = 0;
        for (const auto& s : solution) {
            cost += s.cost;
            makespan = std::max<float>(makespan, s.cost);
        }

        std::ofstream out(outputFile);
        out << "statistics:" << std::endl;
        out << "  cost: " << cost << std::endl;
        out << "  makespan: " << makespan << std::endl;
        out << "  runtime: " << timer.elapsedSeconds() << std::endl;
        out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
        out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
        out << "schedule:" << std::endl;
        for (size_t a = 0; a < solution.size(); ++a) {
            out << "  agent" << a << ":" << std::endl;
            for (const auto& state : solution[a].states) {
                out << "    - x: " << state.first.x << std::endl
                    << "      y: " << state.first.y << std::endl
                    << "      o: " << state.first.o << std::endl
                    << "      t: " << state.second << std::endl;
            }
        }
    } else {
        std::cout << "Planning NOT successful! or Over Time Limit!" << std::endl;
        int error_value = -1;
        std::ofstream out(outputFile);
        out << "statistics:" << std::endl;
        out << "  cost: " << error_value << std::endl;
        out << "  makespan: " << error_value << std::endl;
        out << "  runtime: " << error_value << std::endl;
        out << "  highLevelExpanded: " << error_value << std::endl;
        out << "  lowLevelExpanded: " << error_value << std::endl;
    }
    return 0;
}
