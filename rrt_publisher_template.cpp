//
// Created by mab on 03.11.22.
//

#include "ros/ros.h"
#include "<visualization_msgs/Marker.h>"
#include "rrt.h"

using RosMarker = visualization_msgs::Marker;

ConfigXY kStart(2, 2, 0);
ConfigXY kGoal(18, 18, 90);
ConfigSpace<ConfigXY> kConfigSpace(0, 20, 0, 20);

RosMarker createRosMarker(uint32_t shape, ConfigXY &kStart, std::array<double, 3> color);
RosMarker createRosPoints(std::array<double, 3> color);
inline void addPoint(RosMarker &points, ConfigXY &config);
RosMarker createRosLineList(std::array<double, 3> color);
inline void addLine(RosMarker &line_list, ConfigXY &start, ConfigXY &goal);
void addTreeEdgesToMarker(RosMarker &line_list, Node<ConfigXY> &node);
void addFinalPath(RosMarker &line_list, Node<ConfigXY> &node);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt");
    ros::NodeHandle n;
    ros::Publisher rrt_pub = n.advertise<visualization_msgs::Marker>("rrt_publisher", 1);
    ros::Rate loop_rate(10);

    RRT<ConfigXY> rrt(kStart, kGoal, kConfigSpace);
    rrt.run(100);

    RosMarker start = createRosMarker(RosMarker::POINTS, kStart, {1, 0, 0});
    RosMarker goal = createRosMarker(RosMarker::POINTS, kGoal, {0, 1, 0});

    RosMarker edges = createRosLineList({0, 0, 1});
    addTreeEdgesToMarker(edges, &rrt.nodes[0]);

    RosMarker final_path = createRosLineList({0, 1, 0});
    addFinalPath(final_path, &rrt.nodes.back());


    int count = 0;
    while (ros::ok())
    {
        while (rrt_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        rrt_pub.publish(start);
        rrt_pub.publish(goal);
        rrt_pub.publish(edges);
        rrt_pub.publish(final_path);

        loop_rate.sleep();
    }


    return 0;
}


RosMarker createRosMarker(uint32_t shape, ConfigXY config, std::array<double, 3> color) {
    static int marker_id = 0;
    RosMarker marker;
    marker.header.frame_id = "/rrt";
    marker.header.stamp = ros::Time::now();

    marker.ns = "Start/Goal";
    marker.id = marker_id;

    marker.type = shape;
    marker.action  = RosMarker::ADD;

    marker.pose.position.x = config[0];
    marker.pose.position.y = config[1];
    if (config.size() == 3) marker.pose.orientation.z = config[2];
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_id++;
    return marker;
}

RosMarker createRosPoints(std::array<double, 3> color) {
    static int points_id = 0;
    RosMarker points;
    points.header.frame_id = "/rrt";
    points.header.stamp = ros::Time::now();

    points.ns = "Nodes";
    points.id = points_id;

    points.type = RosMarker::POINTS;
    points.action  = RosMarker::ADD;

    points.pose.orientation.w = 1.0;

    points.scale.x = 0.2;
    points.scale.y = 0.2;

    points.color.r = color[0];
    points.color.g = color[1];
    points.color.b = color[2];
    points.color.a = 1.0;

    points.lifetime = ros::Duration();

    points_id++;
    return points;
}

inline void addPoint(RosMarker &marker, ConfigXY &config) {
    geometry_msgs::Point point;
    point.x = config[0];
    point.y = config[1];

    marker.points.push_back(point);
}

RosMarker createRosLineList(std::array<double, 3> color) {
    static int line_list_id = 0;
    RosMarker line_list;
    line_list.header.frame_id = "/rrt";
    line_list.header.stamp = ros::Time::now();

    line_list.ns = "Edges";
    line_list.id = line_list_id;

    line_list.type = RosMarker::LINE_LIST;
    line_list.action  = RosMarker::ADD;

    line_list.pose.orientation.w = 1.0;

    line_list.scale.x = 0.1;

    line_list.color.r = color[0];
    line_list.color.g = color[1];
    line_list.color.b = color[2];
    line_list.color.a = 1.0;

    line_list.lifetime = ros::Duration();

    line_list_id++;
    return line_list;
}

inline void addLine(RosMarker &line_list, ConfigXY &start, ConfigXY &goal) {
    geometry_msgs::Point point;
    point.x = start[0];
    point.y = start[1];
    line_list.points.push_back(point);
    point.x = goal[0];
    point.y = goal[1];
    line_list.points.push_back(point);
}

RosMarker createRosLineStrip(std::array<double, 3> color) {
    static int line_strip_id = 0;
    RosMarker line_strip;
    line_strip.header.frame_id = "/rrt";
    line_strip.header.stamp = ros::Time::now();

    line_strip.ns = "FinalPath";
    line_strip.id = line_strip_id;

    line_strip.type = RosMarker::LINE_STRIP;
    line_strip.action  = RosMarker::ADD;

    line_strip.pose.orientation.w = 1.0;

    line_strip.scale.x = 0.1;

    line_strip.color.r = color[0];
    line_strip.color.g = color[1];
    line_strip.color.b = color[2];
    line_strip.color.a = 1.0;

    line_strip.lifetime = ros::Duration();

    line_strip_id++;
    return line_strip;
}

void addTreeEdgesToMarker(RosMarker &line_list, Node<ConfigXY> *node) {
    for (Node<Config> *child: node->getChildren()) {
        addLine(line_list, node->config, child->config);
        addTreeEdgesToMarker(line_list, child);
    }
}

void addFinalPath(RosMarker &line_strip, Node<ConfigXY> *node) {
    if (!node) return;
    addPoint(line_strip, node->config);
    addFinalPath(line_strip, node.getParent());
}