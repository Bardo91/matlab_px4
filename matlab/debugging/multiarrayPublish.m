clear all
clc

rosinit
computedTrajectoryPublisher = rospublisher('/computed_trajectory','std_msgs/Float64MultiArray');





trajectory_msg = rosmessage(computedTrajectoryPublisher);
while 1
    trajectory_msg.Data = ones(1000,1);
    
    send(computedTrajectoryPublisher, trajectory_msg)
    
end
rosshutdown

