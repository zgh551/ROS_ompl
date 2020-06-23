#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "polygon_test");

    ros::NodeHandle n;

    ros::Rate r(20);

    ros::Publisher polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("geometry_polygon", 100);

    while(ros::ok())
    {
        geometry_msgs::PolygonStamped vehicle_modle;

        vehicle_modle.header.frame_id = "/vehicle_frame";
        vehicle_modle.header.stamp = ros::Time::now();

        geometry_msgs::Point32 p;
        p.z = 0;

        p.x = 0;
        p.y = 0;
        vehicle_modle.polygon.points.push_back(p);
        p.x = 0;
        p.y = 1;
        vehicle_modle.polygon.points.push_back(p);
        p.x = 1;
        p.y = 1;
        vehicle_modle.polygon.points.push_back(p);
        p.x = 1;
        p.y = 0;
        vehicle_modle.polygon.points.push_back(p);

        polygon_pub.publish(vehicle_modle);

        r.sleep();
    }
}