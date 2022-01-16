#pragma once

#include <iostream>
#include <cmath>
#include <limits>
#include <random>
#include <vector>
#include <algorithm>
#include "matplotlibcpp.h"

namespace ctr{

namespace plt = matplotlibcpp;

class RRTstar
{
public:
    RRTstar();
    bool planning(std::vector<double>& fx, std::vector<double>& fy, double sx, double sy, double gx, double gy, const std::vector<double>& ox, const std::vector<double>& oy);

private:
    struct Node
    {
        int x;
        int y;
        std::vector<double> path_x;
        std::vector<double> path_y;
        double cost = 0.0;
        Node* p_node = NULL;
    };

    std::vector<Node*> node_list;

    Node* get_random_node(int min_x, int min_y, int max_x, int max_y, double gx, double gy);
    Node* get_nearest_node(Node* random_node);
    Node* expand_node(Node* from_node, Node* to_node);
    bool check_collision(Node* node, const std::vector<double>& ox, const std::vector<double>& oy);
    std::vector<Node*> find_near_nodes(Node* new_node);
    Node* choose_parent(Node* new_node, std::vector<Node*> near_nodes);
    double calc_new_cost(Node* from_node, Node* to_node);
    void rewire(Node* new_node, std::vector<Node*> near_nodes);
    void propagate_cost_to_leaves(Node *parent_node);
    void calc_final_path(Node* goal, std::vector<double>& fx, std::vector<double>& fy);

    void draw_graph(Node* node);

    std::vector<double> obstract_x;
    std::vector<double> obstract_y;

    double resolution{1.5};
    double robotRadius{2};
    int max_iterations{400};
    double expand_distance{3.0};
    double goal_sample_rate{0.2};

    double connect_circle_dist{50.0};
    bool search_max_iterations{true};
};

RRTstar::RRTstar()
{

}

bool RRTstar::planning(std::vector<double>& fx, std::vector<double>& fy, double sx, double sy, double gx, double gy, const std::vector<double>& ox, const std::vector<double>& oy)
{
    obstract_x = ox;
    obstract_y = oy;

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

    //start nodeを収納
    node_list.push_back(nstart);

    int try_count = 0;
    while(true)
    {
        //試行回数がmax_iteration より多くなったらやめる
        if(try_count > max_iterations){
            std::cout<<"path don't find!"<<std::endl;
            //break;
            return false;
        }
        //randomな位置のnodeを取得する
        auto rnd_node = get_random_node(min_ox + robotRadius/resolution, min_oy + robotRadius/resolution, max_ox - robotRadius/resolution, max_oy - robotRadius/resolution, gx, gy);
        //rnd_nodeの一番近い既存のnodeを調べる
        auto nearest_node = get_nearest_node(rnd_node);
        //rnd_nodeからnearest_nodeまでのpathを引く
        auto new_node = expand_node(nearest_node, rnd_node);
        //nearest_node -> new_node -> rnd_node

        //new_nodeのコストを記述する
        new_node->cost = nearest_node->cost + std::sqrt(std::pow((new_node->x - nearest_node->x)*resolution, 2)
                                                      + std::pow((new_node->y - nearest_node->y)*resolution - gy, 2));
        //衝突判定
        if(check_collision(new_node, ox, oy)){
            //new_node->周辺のノードを取得
            auto near_nodes = find_near_nodes(new_node);
            //parentを選ぶ
            auto node_with_updated_parent = choose_parent(new_node, near_nodes);

            if(node_with_updated_parent){
                //選ばれたparent nodeにrewireする
                rewire(node_with_updated_parent, near_nodes);
                node_list.push_back(node_with_updated_parent);
            }else{
                node_list.push_back(new_node);
            }

            //animation
            if(try_count%1 == 0){
                draw_graph(new_node);
            }
        }


        double distance = std::sqrt(std::pow(new_node->x*resolution - gx, 2) + std::pow(new_node->y*resolution - gy, 2));
        //distance < expand_distance
        if(try_count == max_iterations-1){
            auto final_node = expand_node(node_list.back(), ngoal);
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

RRTstar::Node* RRTstar::get_random_node(int min_x, int min_y, int max_x, int max_y, double gx, double gy)
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

RRTstar::Node* RRTstar::get_nearest_node(Node* random_node)
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

RRTstar::Node* RRTstar::expand_node(Node* from_node, Node* to_node)
{
    auto new_node = new Node{from_node->x, from_node->y};
    new_node->p_node = from_node;

    new_node->path_x.push_back(new_node->x*resolution);
    new_node->path_y.push_back(new_node->y*resolution);

    //distance function
    auto distance = [&]() -> double
    {
        return std::sqrt(std::pow(to_node->x - from_node->x, 2) + std::pow(to_node->y - from_node->y, 2)) * resolution;
    };
    //angle function
    auto angle = [&]() -> double
    {
        double dx = to_node->x - from_node->x;
        double dy = to_node->y - from_node->y;
        return atan2(dy, dx);
    };

    double extend_length = expand_distance;
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

bool RRTstar::check_collision(Node* node, const std::vector<double>& ox, const std::vector<double>& oy)
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

std::vector<RRTstar::Node*> RRTstar::find_near_nodes(Node* new_node)
{
    std::vector<Node*> near_nodes;

    int num_node = node_list.size() + 1;
    double r = connect_circle_dist * sqrt(log(num_node)/num_node);
    r = std::min(r, expand_distance);

    for(const auto node : node_list){
        double distance = std::sqrt(std::pow(node->x - new_node->x, 2) + std::pow(node->y - new_node->y, 2));
        if(distance < r){
            near_nodes.push_back(node);
        }
    }

    return near_nodes;
}

RRTstar::Node* RRTstar::choose_parent(Node* new_node, std::vector<Node*> near_nodes)
{
    if(near_nodes.empty()) return NULL;

    std::vector<double> costs(near_nodes.size());
    //costを列挙
    for(const auto near_node : near_nodes){
        auto t_node = expand_node(near_node, new_node);

        if(check_collision(t_node, obstract_x, obstract_y)){
            costs.push_back(calc_new_cost(near_node, new_node));
        }else{
            costs.push_back(std::numeric_limits<double>::max());
        }
    }

    //最小のcostを取得
    std::vector<double>::iterator min_iter = std::min_element(costs.begin(),costs.end());
    double min_cost = *min_iter;
    int min_index = std::distance(costs.begin(), min_iter);

    if(min_cost == std::numeric_limits<double>::max()){
        return NULL;
    }

    new_node = expand_node(near_nodes[min_index], new_node);
    new_node->cost = min_cost;

    return new_node;
}

double RRTstar::calc_new_cost(Node* from_node, Node* to_node)
{
    double distance = std::sqrt(std::pow(to_node->x - from_node->x, 2) + std::pow(to_node->y - from_node->y, 2)) * resolution;

    return from_node->cost + distance;
}

/*Node* RRTstar::search_best_goal_node()
{
    
}*/

void RRTstar::calc_final_path(Node* goal, std::vector<double>& fx, std::vector<double>& fy)
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

void RRTstar::rewire(Node* new_node, std::vector<Node*> near_nodes)
{
    for(const auto& near_node : near_nodes){
        auto edge_node = expand_node(new_node, near_node);
        edge_node->cost = calc_new_cost(new_node, near_node);

        bool no_collision = check_collision(edge_node, obstract_x, obstract_y);
        bool improved_cost = near_node->cost > edge_node->cost;

        if(no_collision && improved_cost){
            near_node->x = edge_node->x;
            near_node->y = edge_node->y;
            near_node->cost = edge_node->cost;
            near_node->path_x = edge_node->path_x;
            near_node->path_y = edge_node->path_y;
            near_node->p_node = edge_node->p_node;
            propagate_cost_to_leaves(new_node);
        }
    }
}

void RRTstar::propagate_cost_to_leaves(Node *parent_node)
{
    for(auto &node : node_list){
        //node_listの中からparent_nodeを探す
        if(node->p_node == parent_node){
            node->cost = calc_new_cost(parent_node, node);
            propagate_cost_to_leaves(node);
        }
    }
}


void RRTstar::draw_graph(Node* node)
{
    std::vector<double> x{node->x * resolution};
    std::vector<double> y{node->y * resolution};

    plt::plot(x, y, "^k");
    plt::plot(node->path_x, node->path_y, "-g");
    plt::pause(0.001);

}

}//namespace ctr