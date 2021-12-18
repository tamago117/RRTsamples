#pragma once

#include <iostream>
#include <cmath>
#include <limits>
#include <random>
#include <vector>
#include "matplotlibcpp.h"

namespace ctr{

namespace plt = matplotlibcpp;

class RRT
{
public:
    RRT();
    bool planning(std::vector<double>& fx, std::vector<double>& fy, double sx, double sy, double gx, double gy, const std::vector<double>& ox, const std::vector<double>& oy);

private:
    struct Node
    {
        int x;
        int y;
        std::vector<double> path_x;
        std::vector<double> path_y;
        Node* p_node = NULL;
    };

    std::vector<std::vector<int>> calc_obstacle_map(const std::vector<int>& ox, const std::vector<int>& oy, const int min_ox, const int max_ox, const int min_oy, const int max_oy);

    Node* get_random_node(int min_x, int min_y, int max_x, int max_y, double gx, double gy);
    Node* get_nearest_node(std::vector<Node*> node_list, Node* random_node);
    Node* expand_node(Node* from_node, Node* to_node, double extend_length = std::numeric_limits<double>::max());
    bool check_collision(Node* node, const std::vector<double>& ox, const std::vector<double>& oy);
    void calc_final_path(Node* goal, std::vector<double>& fx, std::vector<double>& fy);

    void draw_graph(Node* node);

    double resolution{1.5};
    double robotRadius{3};
    int max_iterations{1000};
    double expand_distance{4.0};
    double goal_sample_rate{0.1};
};

RRT::RRT()
{

}

bool RRT::planning(std::vector<double>& fx, std::vector<double>& fy, double sx, double sy, double gx, double gy, const std::vector<double>& ox, const std::vector<double>& oy)
{
    auto nstart = new Node{(int)std::round(sx/resolution), (int)std::round(sy/resolution)};
    auto ngoal = new Node{(int)std::round(gx/resolution), (int)std::round(gy/resolution)};

    int min_ox = std::numeric_limits<int>::max();
    int max_ox = std::numeric_limits<int>::min();
    int min_oy = std::numeric_limits<int>::max();
    int max_oy = std::numeric_limits<int>::min();

    //position(m) -> node index
    std::vector<int> ox_idx;
    std::vector<int> oy_idx;

    for(const auto& iox:ox){
        int map_x = (int)std::round(iox*1.0/resolution);
        ox_idx.push_back(map_x);
        min_ox = std::min(map_x, min_ox);
        max_ox = std::max(map_x, max_ox);
    }

    for(const auto& ioy:oy){
        int map_y = (int)std::round(ioy*1.0/resolution);
        oy_idx.push_back(map_y);
        min_oy = std::min(map_y, min_oy);
        max_oy = std::max(map_y, max_oy);
    }

    int xwidth = max_ox-min_ox;
    int ywidth = max_oy-min_oy;

    std::vector<Node*> node_list{nstart};

    int try_count = 0;
    while(true)
    {
        if(try_count > max_iterations){
            std::cout<<"path don't find!"<<std::endl;
            //break;
            return false;
        }

        auto rnd_node = get_random_node(min_ox, min_oy, max_ox, max_oy, gx, gy);
        auto nearest_node = get_nearest_node(node_list, rnd_node);

        auto new_node = expand_node(nearest_node, rnd_node, expand_distance);

        if(check_collision(new_node, ox, oy)){
            node_list.push_back(new_node);

            //animation
            if(try_count%1 == 0){
                draw_graph(new_node);
            }
        }

        

        double distance = std::sqrt(std::pow(new_node->x*resolution - gx, 2) + std::pow(new_node->y*resolution - gy, 2));
        if(distance < expand_distance){
            auto final_node = expand_node(node_list.back(), ngoal, expand_distance);
            if(check_collision(final_node, ox, oy)){
                calc_final_path(final_node, fx, fy);

                delete nearest_node;
                delete rnd_node;
                delete new_node;
                break;
            }
        }

        try_count++;
    }

    delete ngoal;
    delete nstart;

    return true;

}

RRT::Node* RRT::get_random_node(int min_x, int min_y, int max_x, int max_y, double gx, double gy)
{
    std::random_device rnd;     // 非決定的な乱数生成器
    std::mt19937 mt(rnd());
    std::uniform_int_distribution<> rand1(0, 1);
    std::uniform_int_distribution<> randx(min_x, max_x);
    std::uniform_int_distribution<> randy(min_y, max_y);

    if(rand1(mt)>goal_sample_rate){
        return new Node{(int)randx(mt), (int)randy(mt)};
    }else{//goal point sampling
        return new Node{(int)std::round(gx/resolution), (int)std::round(gy/resolution)};
    }
}

RRT::Node* RRT::get_nearest_node(std::vector<Node*> node_list, Node* random_node)
{
    //kdtree使いたいなあ・・・
    double min_distance = std::numeric_limits<double>::max();
    int index = 0, min_index = 0;
    for(const auto node : node_list){
        double distance = std::sqrt(std::pow(node->x-random_node->x, 2) + std::pow(node->y-random_node->y, 2));
        if(distance<min_distance){
            min_distance = distance;
            min_index = index;
        }
        index++;
    }

    return node_list[min_index];
}

RRT::Node* RRT::expand_node(Node* from_node, Node* to_node, double extend_length)
{
    auto new_node = new Node{from_node->x, from_node->y};
    new_node->p_node = from_node;

    new_node->path_x.push_back(new_node->x*resolution);
    new_node->path_y.push_back(new_node->y*resolution);

    auto distance = [&]() -> double
    {
        return std::sqrt(std::pow(to_node->x - from_node->x, 2) + std::pow(to_node->y - from_node->y, 2)) * resolution;
    };
    auto angle = [&]() -> double
    {
        double dx = to_node->x - from_node->x;
        double dy = to_node->y - from_node->y;
        return atan2(dy, dx);
    };

    if(extend_length > distance()){
        extend_length = distance();
    }

    int n_expand = std::round(extend_length/resolution);

    for(int i=0; i<n_expand; ++i){
        new_node->x += resolution * cos(angle());
        new_node->y += resolution * sin(angle());
        new_node->path_x.push_back(new_node->x*resolution);
        new_node->path_y.push_back(new_node->y*resolution);
    }

    if(distance() < resolution){
        new_node->path_x.push_back(new_node->x*resolution);
        new_node->path_y.push_back(new_node->y*resolution);
        new_node->x = to_node->x;
        new_node->y = to_node->y;
    }

    return new_node;

}

bool RRT::check_collision(Node* node, const std::vector<double>& ox, const std::vector<double>& oy)
{
    int obst_size = ox.size();
    for(int i = 0; i < obst_size; ++i){
        double distance = std::sqrt(std::pow(node->x*resolution - ox[i], 2) + std::pow(node->y*resolution - oy[i], 2));
        if(distance < robotRadius){
            return false;
        }
        /*int path_size = node->path_x.size();
        for(int j = 0; j < path_size; ++j){
            double distance = std::sqrt(std::pow(node->path_x[i] - ox[i], 2) + std::pow(node->path_y[i] - oy[i], 2));
            if(distance < robotRadius){
                return false;
            }
        }*/
    }

    return true;
}

void RRT::calc_final_path(Node* goal, std::vector<double>& fx, std::vector<double>& fy)
{
    std::vector<double> rx;
    std::vector<double> ry;
    Node* node = goal;

    while(node->p_node != NULL)
    {
        node = node->p_node;
        rx.push_back(node->x * resolution);
        ry.push_back(node->y * resolution);
    }

    fx = rx;
    fy = ry;
}

void RRT::draw_graph(Node* node)
{
    std::vector<double> x{node->x * resolution};
    std::vector<double> y{node->y * resolution};

    plt::plot(x, y, "^k");
    plt::plot(node->path_x, node->path_y, "-g");
    plt::pause(0.001);

}

}