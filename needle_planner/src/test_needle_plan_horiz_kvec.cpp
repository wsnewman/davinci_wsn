//test program/example for stimulating needle_plan_horz_kvec
//prompts for entrance pt and kvec angle; computes an exit point
// these SHOULD be generated from HMI (faked here)
// puts points in a geometry_msgs/Polygon message
// and publishes this message to topic "/entrance_and_exit_pts"
// needle_plan_horz_kvec listens to this topic and computes and saves a needle-drive trajectory

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_planner"); // name of this node will be "minimal_publisher"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher pub = n.advertise<geometry_msgs::Polygon>("/entrance_and_exit_pts", 1);
    double kvec_yaw;
    double d_in_to_out = 0.01; 
    double x_entry,y_entry,z_entry;
    geometry_msgs::Polygon pts_array_msg;
    geometry_msgs::Point32 pt;
    cout<<"enter entrance point x (e.g. 0.0): ";
    cin>>x_entry;
    cout<<"enter entrance point y (e.g. 0.0): ";
    cin>>y_entry;    
    cout<<"enter entrance point z (e.g. 0.12): ";
    cin>>z_entry;
    pt.x = x_entry;
    pt.y = y_entry;
    pt.z = z_entry;
    pts_array_msg.points.push_back(pt);
    
    cout<<"enter kvec_yaw (0-2pi, e.g. 0.7): ";
    cin>>kvec_yaw;    
    pt.x+= d_in_to_out*cos(kvec_yaw);
    pt.y+= d_in_to_out*sin(kvec_yaw);
    pts_array_msg.points.push_back(pt);
    
    pub.publish(pts_array_msg);
    
}

