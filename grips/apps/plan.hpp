#include <iostream>
#include <vector>
#include <memory>
#include "base/gnode.h"
#include "PostSmoothing.h"
#include "base/PlannerUtils.hpp"

struct Point
{
    double x,y,z;
    point():x(0),y(0),z(0){}
};

class planning
{
    using point = Point;
public:
    planning()
    {
        smooth.reset(new PostSmoothing());
        PlannerSettings::initializeSteering();//初始化约束方式
        /*设置环境*/
        PlannerSettings::environment = Environment::createFromObstacles(obstacles, 40, 25);
    PlannerSettings::environment->setStart(Tpoint(5, 3));
    PlannerSettings::environment->setGoal(Tpoint(36, 22));
    }

    ~planning(){}

    std::vector<GNode> solutionPath(const std::vector<point> &solutionPoints) const
    {
        std::vector<GNode> points;
        for(const auto &solutionPoint:solutionPoints){
            double x = solutionPoint.x;
            double y = solutionPoint.y;
            points.emplace_back(x,y);
        }
        //PlannerUtils::updateAngles(gnodes, true);
        return points;
    }

    std::vector<point> getTrajectory(const std::vector<point> &points)
    {
        std::vector<GNode> path = solutionPath(points);
        smooth->smooth(path);
        std::vector<point> smoothPath;
        for(const auto &p : path){
            double x = p.x;
            double y = p.y;
            smoothPath.emplace_back(x,y,0.0);
        }
        return smoothPath;
    }

private:
    std::shared_ptr<PostSmoothing> smooth;
};