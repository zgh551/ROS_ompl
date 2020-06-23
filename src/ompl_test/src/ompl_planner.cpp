// ros lib
#include "ros/ros.h"
// rviz initial pose and goal pose
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
// rivz marker
#include "visualization_msgs/Marker.h"
// Pose
#include "geometry_msgs/PoseArray.h"
// rviz polygon 
#include "geometry_msgs/PolygonStamped.h"
// rviz path
#include "nav_msgs/Path.h"
// ros tf 
#include "tf/transform_datatypes.h"

// ompl lib 
// the state space
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>

#include <ompl/geometric/planners/AnytimePathShortening.h>

// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include <fstream>
#include <vector>
#include <math.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

#define BOUNDARY 10

// the ompl variable define
ob::StateSpacePtr space;
ob::ProblemDefinitionPtr pdef;
ob::SpaceInformationPtr si;
ob::PlannerPtr opt_plan;
og::SimpleSetupPtr ss;

// the temp variable define
double init_x, init_y, init_yaw;
uint8_t solve_flag = 0;

// the visulization marker define
visualization_msgs::Marker init_line_strip, goal_line_strip;
visualization_msgs::Marker simplify_point;
geometry_msgs::PoseArray sample_point;

geometry_msgs::PolygonStamped vehicle_modle;

nav_msgs::Path plan_path;

// obstacle 
typedef struct _Boundary{
    double low_x;
    double low_y;
    double high_x;
    double high_y;
}Boundary;

typedef struct _Position{
    double x;
    double y;
    double yaw;
}Position;

typedef struct _ObstacleBoundary{
    double lenght;
    double width;
    double lenght_half;
    double width_half;
    double cos_heading;
    double sin_heading;
    Position pose;
    Boundary boundary={ std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::lowest(),std::numeric_limits<double>::lowest()};
}ObstacleBoundary;

ObstacleBoundary vehicle_box;
std::vector<ObstacleBoundary>obstacle_vehicle_boundary;
std::vector<visualization_msgs::Marker> obstacle_masker_array;
Position start_position;

// vehicle information define
#define FRONT_EDGE_TO_CENTER ( 3.8860 )
#define REAR_EDGE_TO_CENTER  ( 1.1000 )
#define LR_EDGE_TO_CENTER    ( 0.9275 )
double front_axis_lenght, rear_axis_lenght;
double front_axis_angle, rear_axis_angle;


// judge the point whether in the boundary 
bool isInBoundary(double x, double y, Boundary& b)
{
    if((x >= b.low_x && x <= b.high_x) && (y >= b.low_y && y <= b.high_y))
    {
        return true;
    }
    else
    {
        return false;
    }  
}
// expand the boundary with radius r
void BoundaryExpand(const Boundary& init_b, Boundary& goal_b, double r)
{
    goal_b.low_x = init_b.low_x - r;
    goal_b.low_y = init_b.low_y - r;
    goal_b.high_x = init_b.high_x + r;
    goal_b.high_y = init_b.high_y + r;
}

// the vehicle parameter init
void VehicleInit(void)
{
    front_axis_lenght = sqrt(pow(FRONT_EDGE_TO_CENTER, 2) + pow(LR_EDGE_TO_CENTER, 2));
    rear_axis_lenght  = sqrt(pow(REAR_EDGE_TO_CENTER , 2) + pow(LR_EDGE_TO_CENTER, 2));

    front_axis_angle = atan2(LR_EDGE_TO_CENTER, FRONT_EDGE_TO_CENTER);
    rear_axis_angle  = atan2(LR_EDGE_TO_CENTER, REAR_EDGE_TO_CENTER);

    vehicle_box.lenght = FRONT_EDGE_TO_CENTER + REAR_EDGE_TO_CENTER;
    vehicle_box.width  = LR_EDGE_TO_CENTER * 2;

    vehicle_box.lenght_half = 0.5 * (FRONT_EDGE_TO_CENTER + REAR_EDGE_TO_CENTER);
    vehicle_box.width_half  = LR_EDGE_TO_CENTER;
}

// rotation the location 
void rotate(const double& init_x, const double& init_y, const double& yaw, double& goal_x, double& goal_y)
{
    double s_vl,c_vl;
    s_vl = sin(yaw);
    c_vl = cos(yaw);

    goal_x = init_x * c_vl - init_y * s_vl;
    goal_y = init_x * s_vl + init_y * c_vl;
}

// calculate the vehicle edge
void VehicleEdge(const double& x, const double& y, const double& yaw,std::vector<Position> p)
{
    p.clear();
    Position temp_p;
    rotate(front_axis_lenght, 0, front_axis_angle + yaw, temp_p.x, temp_p.y);
    temp_p.x += x;
    temp_p.y += y;
    temp_p.yaw = yaw;
    p.push_back(temp_p);

    rotate(front_axis_lenght, 0, front_axis_angle - yaw, temp_p.x, temp_p.y);
    temp_p.x += x;
    temp_p.y += y;
    temp_p.yaw = yaw;
    p.push_back(temp_p);

    rotate(-rear_axis_lenght, 0, rear_axis_angle + yaw, temp_p.x, temp_p.y);
    temp_p.x += x;
    temp_p.y += y;
    temp_p.yaw = yaw;
    p.push_back(temp_p);

    rotate(-rear_axis_lenght, 0, rear_axis_angle - yaw, temp_p.x, temp_p.y);
    temp_p.x += x;
    temp_p.y += y;
    temp_p.yaw = yaw;
    p.push_back(temp_p);
}

bool ompl_StateValidityCheckerFunction(const ob::State *state)
{
    const auto* stateE2 = state->as<ob::SE2StateSpace::StateType>();
    
    double x = stateE2->getX();
    double y = stateE2->getY();
    double yaw = stateE2->getYaw();

    // for(auto b : obstacle_vehicle_boundary)
    // {
    //     Boundary exp_b;
    //     if(isInBoundary(x, y, b))
    //     {
    //         return false;
    //     }
    //     else
    //     {
    //         BoundaryExpand(b, exp_b, 2.5);
    //         if(isInBoundary(x, y, exp_b))
    //         {
    //             std::vector<Position> temp_p;
    //             VehicleEdge(x, y, yaw, temp_p);
    //             for(auto p : temp_p)
    //             {
    //                 if(isInBoundary(p.x, p.y, b))
    //                 {
    //                     return false;
    //                 }
    //             }
    //         }
    //     }
    // }
    return true;
}

void InitCorners(ObstacleBoundary& v)
{
    const double dx1 =  v.cos_heading * v.lenght_half;
    const double dy1 =  v.sin_heading * v.lenght_half;
    const double dx2 =  v.sin_heading * v.width_half;
    const double dy2 = -v.cos_heading * v.width_half;

    std::vector<Position> corners;
    corners.clear();
    Position temp_p;
    temp_p.yaw = 0.0;
    temp_p.x = v.pose.x + dx1 + dx2;
    temp_p.y = v.pose.y + dy1 + dy2;
    corners.push_back(temp_p);
    temp_p.x = v.pose.x + dx1 - dx2;
    temp_p.y = v.pose.y + dy1 - dy2;
    corners.push_back(temp_p);
    temp_p.x = v.pose.x - dx1 - dx2;
    temp_p.y = v.pose.y - dy1 - dy2;
    corners.push_back(temp_p);
    temp_p.x = v.pose.x - dx1 + dx2;
    temp_p.y = v.pose.y - dy1 + dy2;
    corners.push_back(temp_p);

    v.boundary.low_x = std::numeric_limits<double>::max();
    v.boundary.low_y = std::numeric_limits<double>::max();

    v.boundary.high_x = std::numeric_limits<double>::lowest();
    v.boundary.high_y = std::numeric_limits<double>::lowest();

    for(auto &corner : corners)
    {
        v.boundary.low_x = fmin(corner.x, v.boundary.low_x);
        v.boundary.low_y = fmin(corner.y, v.boundary.low_y);
        v.boundary.high_x = fmax(corner.x, v.boundary.high_x);
        v.boundary.high_y = fmax(corner.y, v.boundary.high_y);
    }
}

bool HasOverlap(const ObstacleBoundary& v, const ObstacleBoundary& o)
{
    if(v.boundary.high_x < o.boundary.low_x  ||
       v.boundary.low_x  > o.boundary.high_x ||
       v.boundary.high_y < o.boundary.low_y  ||
       v.boundary.low_y  > o.boundary.high_y
    )
    {
        return false;
    }

    const double shift_x = o.pose.x - v.pose.x;
    const double shift_y = o.pose.y - v.pose.y;

    const double dx1 =  v.cos_heading * v.lenght_half;
    const double dy1 =  v.sin_heading * v.lenght_half;
    const double dx2 =  v.sin_heading * v.width_half;
    const double dy2 = -v.cos_heading * v.width_half;

    const double dx3 =  o.cos_heading * o.lenght_half;
    const double dy3 =  o.sin_heading * o.lenght_half;
    const double dx4 =  o.sin_heading * o.width_half;
    const double dy4 = -o.cos_heading * o.width_half;  

    return  abs(shift_x * v.cos_heading + shift_y * v.sin_heading) <=
            abs(dx3     * v.cos_heading + dy3     * v.sin_heading) +
            abs(dx4     * v.cos_heading + dy4     * v.sin_heading) +
            v.lenght_half                                           &&
            abs(shift_x * v.sin_heading - shift_y * v.cos_heading) <=
            abs(dx3     * v.sin_heading - dy3     * v.cos_heading) +
            abs(dx4     * v.sin_heading - dy4     * v.cos_heading) +
            v.width_half                                            &&
            abs(shift_x * o.cos_heading + shift_y * o.sin_heading) <=
            abs(dx1     * o.cos_heading + dy1     * o.sin_heading) +
            abs(dx2     * o.cos_heading + dy2     * o.sin_heading) +
            o.lenght_half                                           &&
            fabs(shift_x * o.sin_heading - shift_y * o.cos_heading) <=
            fabs(dx1     * o.sin_heading - dy1     * o.cos_heading) +
            fabs(dx2     * o.sin_heading - dy2     * o.cos_heading) +
            o.width_half                                            ;
}

double MinDistance(const ObstacleBoundary& v, const ObstacleBoundary& o)
{
    if(v.boundary.high_x < o.boundary.low_x  ||
       v.boundary.low_x  > o.boundary.high_x ||
       v.boundary.high_y < o.boundary.low_y  ||
       v.boundary.low_y  > o.boundary.high_y
    )
    {
        if(v.boundary.high_x < o.boundary.low_x)
        {

        }
    }
    else
    {
        return 0.0;
    }
}

class ValidityChecker : public ob::StateValidityChecker
{
    public:
    ValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si) {}

    bool isValid(const ob::State* state) const override
    {
        const auto* stateE2 = state->as<ob::SE2StateSpace::StateType>();

        vehicle_box.pose.x = stateE2->getX() + 0.5 * (FRONT_EDGE_TO_CENTER - REAR_EDGE_TO_CENTER);
        vehicle_box.pose.y = stateE2->getY();
        vehicle_box.pose.yaw = stateE2->getYaw();
        vehicle_box.cos_heading = cos(vehicle_box.pose.yaw);
        vehicle_box.sin_heading = sin(vehicle_box.pose.yaw);

        InitCorners(vehicle_box);
        for(auto b : obstacle_vehicle_boundary)
        {
            InitCorners(b);
            if (HasOverlap(vehicle_box, b))
            {
                return false;
            }
        }
        return true;
    }

    double clearance(const ob::State* state) const override
    {
        const auto* stateE2 = state->as<ob::SE2StateSpace::StateType>();

        vehicle_box.pose.x = stateE2->getX() + 0.5 * (FRONT_EDGE_TO_CENTER - REAR_EDGE_TO_CENTER);
        vehicle_box.pose.y = stateE2->getY();
        vehicle_box.pose.yaw = stateE2->getYaw();
        vehicle_box.cos_heading = cos(vehicle_box.pose.yaw);
        vehicle_box.sin_heading = sin(vehicle_box.pose.yaw);

        InitCorners(vehicle_box);
        for(auto b : obstacle_vehicle_boundary)
        {
            InitCorners(b);

        }   
    }
};

// the lenght optimization
ob::OptimizationObjectivePtr getPathLenghtObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

class ClearanceObjective : public ob::StateCostIntegralObjective
{
    ClearanceObjective(const ob::SpaceInformationPtr& si) : ob::StateCostIntegralObjective(si, true)
    {

    }

    ob::Cost stateCost(const ob::State* s) const override
    {
        // return ob::Cost(1.0 - )
    }
};

void SpaceInit(void)
{
    ob::RealVectorBounds obstacle_boundary = ob::RealVectorBounds(2);

    obstacle_boundary.setLow(0, -BOUNDARY);
    obstacle_boundary.setLow(1, -BOUNDARY);

    obstacle_boundary.setHigh(0, BOUNDARY);
    obstacle_boundary.setHigh(1, BOUNDARY);
    
    space = std::make_shared<ob::ReedsSheppStateSpace>(5.0);
    space->as<ob::SE2StateSpace>()->setBounds(obstacle_boundary);

    si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));
    si->setStateValidityCheckingResolution(0.03);
    si->setup();

    // pdef = std::make_shared<ob::ProblemDefinition>(si);
    // pdef->setOptimizationObjective(getPathLenghtObjective(si));

    // opt_plan = std::make_shared<og::ABITstar>(si);
    // opt_plan = std::make_shared<og::AITstar>(si);
    // std::shared_ptr<og::AnytimePathShortening> opt_plan = std::make_shared<og::AnytimePathShortening>(si);
    // opt_plan->setPlanners("BITstar,RRTstart,TRRT,STRIDE");
    // opt_plan->printSettings(std::cout);

    // std::shared_ptr<og::RRTstar> opt_plan = std::make_shared<og::RRTstar>(si);

    ss = std::make_shared<og::SimpleSetup>(si);
    ss->setOptimizationObjective(getPathLenghtObjective(si));
    // ss->setPlanner(std::make_shared<og::InformedRRTstar>(si));
    ss->setPlanner(std::make_shared<og::AITstar>(si));

    // obstacle configure
    ObstacleBoundary temp_o;
    obstacle_vehicle_boundary.clear();

    temp_o.lenght =  5.0;
    temp_o.width  =  1.8;
    temp_o.pose.x = -0.9;
    temp_o.pose.y = -2.5;
    temp_o.pose.yaw = M_PI_2;
    temp_o.lenght_half = temp_o.lenght * 0.5;
    temp_o.width_half  = temp_o.width * 0.5;
    temp_o.cos_heading = cos(temp_o.pose.yaw);
    temp_o.sin_heading = sin(temp_o.pose.yaw);
    obstacle_vehicle_boundary.push_back(temp_o);

    temp_o.lenght =  5.0;
    temp_o.width  =  1.8;
    temp_o.pose.x =  3.9;
    temp_o.pose.y = -2.5;
    temp_o.pose.yaw = M_PI_2;
    temp_o.lenght_half = temp_o.lenght * 0.5;
    temp_o.width_half  = temp_o.width * 0.5;
    temp_o.cos_heading = cos(temp_o.pose.yaw);
    temp_o.sin_heading = sin(temp_o.pose.yaw);
    obstacle_vehicle_boundary.push_back(temp_o);

    temp_o.lenght =  5.0;
    temp_o.width  =  1.8;
    temp_o.pose.x =  1.5;
    temp_o.pose.y =  7.0;
    temp_o.pose.yaw = 0.0;
    temp_o.lenght_half = temp_o.lenght * 0.5;
    temp_o.width_half  = temp_o.width * 0.5;
    temp_o.cos_heading = cos(temp_o.pose.yaw);
    temp_o.sin_heading = sin(temp_o.pose.yaw);
    obstacle_vehicle_boundary.push_back(temp_o);
}

// the marker init
void MarkerLineStripInit(void)
{
    init_line_strip.header.frame_id =
    goal_line_strip.header.frame_id = 
    vehicle_modle.header.frame_id   =
    plan_path.header.frame_id       =
    sample_point.header.frame_id    =
    simplify_point.header.frame_id  =
    "/ompl_frame";

    init_line_strip.header.stamp    = 
    goal_line_strip.header.stamp    = 
    vehicle_modle.header.stamp      =
    plan_path.header.stamp          =
    sample_point.header.stamp       =
    simplify_point.header.stamp     =
    ros::Time::now();

    init_line_strip.ns  = "init line strip"; 
    goal_line_strip.ns  = "goal line strip";
    simplify_point.ns   = "simplify point";

    init_line_strip.action = 
    goal_line_strip.action =
    simplify_point.action  =
    visualization_msgs::Marker::MODIFY;
   
    init_line_strip.id = 0;
    goal_line_strip.id = 1;
    simplify_point.id  = 2;

    init_line_strip.type = 
    goal_line_strip.type = 
    visualization_msgs::Marker::ARROW;

    simplify_point.type = visualization_msgs::Marker::POINTS;

    init_line_strip.scale.x = goal_line_strip.scale.x = 1;
    init_line_strip.scale.y = goal_line_strip.scale.y = 0.1;
    init_line_strip.scale.z = goal_line_strip.scale.z = 0.1;

    // sample_point.scale.x = 0.5;
    // sample_point.scale.y = 0.05;
    // sample_point.scale.z = 0.1;

    simplify_point.scale.x = simplify_point.scale.y = simplify_point.scale.z = 0.1;

    init_line_strip.color.g = 1.0;
    init_line_strip.color.a = 1.0;

    goal_line_strip.color.b = 1.0;
    goal_line_strip.color.a = 1.0;

    // sample_point.color.r = 1.0;
    // sample_point.color.g = 1.0;
    // sample_point.color.a = 1.0;

    simplify_point.color.r = 1.0;
    simplify_point.color.a = 1.0;

    int32_t id_index = 4;
    obstacle_masker_array.clear();
    for(auto b : obstacle_vehicle_boundary)
    {
        visualization_msgs::Marker obstacle_temp;

        obstacle_temp.header.frame_id = "ompl_frame";
        obstacle_temp.header.stamp = ros::Time::now();
        obstacle_temp.ns = "obstacle_vehicle";
        obstacle_temp.id = id_index++;
        obstacle_temp.action = visualization_msgs::Marker::MODIFY;
        obstacle_temp.type   = visualization_msgs::Marker::CUBE;

        obstacle_temp.color.g = 1.0;
        obstacle_temp.color.b = 1.0;
        obstacle_temp.color.a = 1.0;

        obstacle_temp.scale.z = 0.1;
        obstacle_temp.scale.x = b.lenght;
        obstacle_temp.scale.y = b.width;
        
        obstacle_temp.pose.position.x = b.pose.x;
        obstacle_temp.pose.position.y = b.pose.y;
        tf::Quaternion q = tf::createQuaternionFromYaw(b.pose.yaw);
        tf::quaternionTFToMsg(q, obstacle_temp.pose.orientation);
        obstacle_masker_array.push_back(obstacle_temp);
    }

    // polygon 
    geometry_msgs::Point32 p;
    p.z = 0;

    p.x = 0;
    p.y = 0;
    vehicle_modle.polygon.points.push_back(p);
    p.x = 0;
    p.y = -5;
    vehicle_modle.polygon.points.push_back(p);
    p.x = -2;
    p.y = -5;
    vehicle_modle.polygon.points.push_back(p);
    p.x = -2;
    p.y = 0;
    vehicle_modle.polygon.points.push_back(p);
}

// the init position callback
void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& p)
{
    double yaw = tf::getYaw(p.pose.pose.orientation);

    start_position.x = p.pose.pose.position.x;
    start_position.y = p.pose.pose.position.y;
    start_position.yaw = yaw;

    // update the masker position
    init_line_strip.pose = p.pose.pose;

    ROS_INFO("the pose x:%f, y:%f, yaw:%f", p.pose.pose.position.x, p.pose.pose.position.y, yaw);
}

// the goal position callback
void GoalPoseCallback(const geometry_msgs::PoseStamped& g)
{
    double yaw = tf::getYaw(g.pose.orientation);
    ROS_INFO("the position x:%f, y:%f, yaw:%f",g.pose.position.x, g.pose.position.y, yaw);
    goal_line_strip.pose = g.pose;

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space); 

    start->as<ob::SE2StateSpace::StateType>()->setX(start_position.x);
    start->as<ob::SE2StateSpace::StateType>()->setY(start_position.y);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(start_position.yaw);

    goal->as<ob::SE2StateSpace::StateType>()->setX( g.pose.position.x );
    goal->as<ob::SE2StateSpace::StateType>()->setY( g.pose.position.y );
    goal->as<ob::SE2StateSpace::StateType>()->setYaw( yaw );

    ss->setStartAndGoalStates(start, goal);
    solve_flag = 0xff;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ompl_planner");

    ros::NodeHandle n;

    ros::Rate r(20);

    // the line marker publisher
    ros::Publisher line_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    ros::Publisher polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("geometry_polygon", 100);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory", 100);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseArray>("pose_array", 100);

    // the initial position and the goal position subscriber
    ros::Subscriber init_pose_sub = n.subscribe("initialpose", 100, InitialPoseCallback);
    ros::Subscriber goal_pose_sub = n.subscribe("move_base_simple/goal", 100, GoalPoseCallback);

    // init the state space
    SpaceInit();

    VehicleInit();

    // the marker init
    MarkerLineStripInit();

    while(ros::ok())
    {
        if(0xff == solve_flag)
        {
            // setup the planner
            ss->setup();
            ss->print();
            
            ob::PlannerStatus solved = ss->solve(10);
            if(solved)
            {
                //update all sample point to the pose marker
                geometry_msgs::Pose temp_pose;
                sample_point.poses.clear();
                for(auto &state : ss->getSolutionPath().getStates())
                {
                    temp_pose.position.x = state->as<ob::SE2StateSpace::StateType>()->getX();
                    temp_pose.position.y = state->as<ob::SE2StateSpace::StateType>()->getY();
                    tf::Quaternion q = tf::createQuaternionFromYaw(state->as<ob::SE2StateSpace::StateType>()->getYaw());
                    tf::quaternionTFToMsg(q, temp_pose.orientation);
                    sample_point.poses.push_back(temp_pose);
                }
                    
                // simplify the sample point
                ss->simplifySolution();
                og::PathGeometric path = ss->getSolutionPath();
                // update all the simplify point to the marker point
                geometry_msgs::Point temp_p;
                simplify_point.points.clear();
                for(auto &state : path.getStates())
                {
                    temp_p.x = state->as<ob::SE2StateSpace::StateType>()->getX();
                    temp_p.y = state->as<ob::SE2StateSpace::StateType>()->getY();
                    simplify_point.points.push_back(temp_p);
                }

                // the interpolate process
                path.interpolate(1000);

                // update the all interpolate point to the path
                geometry_msgs::PoseStamped p;
                plan_path.poses.clear();
                for(auto &state : path.getStates())
                {
                    p.pose.position.x = state->as<ob::SE2StateSpace::StateType>()->getX();
                    p.pose.position.y = state->as<ob::SE2StateSpace::StateType>()->getY();
                    plan_path.poses.push_back(p);
                }
            }
            else
            {
                std::cout << "No solution found" << std::endl;
            }
            ss->clear();
            solve_flag = 0;
        }
        
        line_marker_pub.publish(init_line_strip);
        line_marker_pub.publish(goal_line_strip);
        line_marker_pub.publish(simplify_point);
        for( auto m : obstacle_masker_array)
        {
            line_marker_pub.publish(m);
        }
        polygon_pub.publish(vehicle_modle);
        path_pub.publish(plan_path);
        pose_pub.publish(sample_point);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
