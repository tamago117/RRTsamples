#include<iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include<array>
#include "include/matplotlibcpp.h"
#include "include/RRT.h"
#include "include/bezier_curve.h"

namespace plt = matplotlibcpp;
const int n = 50;
const int approNode = 20;
const double approResolution = 10;

void obstract_line(std::vector<std::vector<double>>& ob, double x1, double x2, double y1, double y2)
{
    //終了処理
    const int pitch = 2;
    double dx, dy, dis, preX, preY;

    dx = x2 - x1;
    dy = y2 - y1;
    dis = sqrt(pow(dx, 2) + pow(dy, 2));
    preX = x1;
    preY = y1;

    double angle = atan2(y2-y1, x2-x1);

    std::vector<double> temp_array(2);

    while(dis > pitch)
    {
        temp_array[0] = pitch*cos(angle) + preX;
        temp_array[1] = pitch*sin(angle) + preY;
        ob.push_back(temp_array);

        preX = temp_array[0];
        preY = temp_array[1];

        dx = x2 - preX;
        dy = y2 - preY;
        dis = sqrt(pow(dx, 2) + pow(dy, 2));

    }
}

int main()
{

    std::vector<double> goal_x{45};
    std::vector<double> goal_y{40};
    std::vector<double> goal{goal_x[0], goal_y[0]};
    std::vector<double> ini_x{4};
    std::vector<double> ini_y{4};
    std::vector<std::vector<double>> ob;
    std::vector<double> ob_x, ob_y;
    std::vector<double> path_x, path_y;
    std::vector<double> r_x;
    std::vector<double> r_y;

    obstract_line(ob, 0, 0, 0, n);
    obstract_line(ob, 0, n, n, n);
    obstract_line(ob, n, n, n, 0);
    obstract_line(ob, n, 0, 0, 0);
    obstract_line(ob, 15, 15, 0, 25);
    obstract_line(ob, 35, 35, 15, 50);
    //obstract_line(ob, 10, 30, 43, 13);

    ctr::RRT rrt;

    for(int i=0; i<ob.size(); i++){
        ob_x.push_back(ob[i][0]);
        ob_y.push_back(ob[i][1]);
    }

    // Clear previous plot
    //plt::clf();
    plt::scatter(ob_x, ob_y); //obstract
    plt::plot(goal_x, goal_y, "xb"); //goal
    plt::plot(ini_x, ini_y, "xb"); //start

    // Set x-axis to interval [0,n]
    plt::xlim(-10, n+10);
    plt::ylim(-10, n+10);

    // Add graph title
    plt::title("RRT");

    rrt.planning(path_x, path_y, ini_x[0], ini_y[0], goal_x[0], goal_y[0], ob_x, ob_y);
    /*bezier_curve bezier;

    int node;
    for(int i=0; i<path_x.size()/approNode+1; i++){
        node = i*approNode;
        std::vector<double> p_x, p_y;
        if(i<path_x.size()/approNode){
            for(int j=0; j<approNode; j++){
                p_x.push_back(path_x[node+j]);
                p_y.push_back(path_y[node+j]);
            }
        }else{
            for(int j=0; j<path_x.size() - node; j++){
                p_x.push_back(path_x[node+j]);
                p_y.push_back(path_y[node+j]);
            }
        }
        

        for (double t = 0; t <= 1.0; t += (1/approResolution)){
            std::array<double, 2> point = bezier.getPos(t, p_x, p_y);
            r_x.push_back(point[0]);
            r_y.push_back(point[1]);
        }
    }*/

    plt::plot(path_x, path_y); //path
    //plt::plot(r_x, r_y); //smooth path

    
    // Display plot continuously
    //plt::pause(0.001);
    // show plots
    plt::show();

    return 0;
}