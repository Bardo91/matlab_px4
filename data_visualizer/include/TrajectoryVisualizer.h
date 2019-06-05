//
//
//
//
//
//

#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <splines.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>


class TrajectoryVisualizer{
public:
    TrajectoryVisualizer();
    
    void draw(const std::vector<Eigen::Vector3f> &_trajectory, unsigned char _r=0, unsigned char g=255, unsigned char b=0, bool _useSpline = false);

    void drawSphere(const Eigen::Vector3f &_center, float _radius);

    int rangeRandom(int _min, int _max);

    void spin();

    void spinOnce();

private:
    void trajectoryCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    void waypointsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    int itemCounter_ = 0;

    std::vector<vtkSmartPointer<vtkPolyData>> trajectories_;

    ros::Subscriber trajectorySubscriber_;

    ros::Subscriber waypointsSubscriber_;

};
