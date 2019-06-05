//
//
//
//
//
//

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <TrajectoryVisualizer.h>
#include <chrono>
#include <thread>

int main(int _argc, char** _argv){

    ros::init(_argc, _argv, "data_visualizer");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    TrajectoryVisualizer visualizer;

    std::vector<Eigen::Vector3f> trajectory = {
        {0,0,0},
        {1,1,1},
        {1,2,3},
        {3,2,1}
    };


    // visualizer.draw(trajectory);
    // visualizer.draw(trajectory, 255, 0, 0, true);

    while(ros::ok()){
        visualizer.spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

}
