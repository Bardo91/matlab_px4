%% estrarre la prima traiettoria 
bag = rosbag('/home/emanuela/Scrivania/rosbags/rosbag_22-05-19/2019-05-22-13-45-02_filtered.bag');
TrajectorySelect = select(bag, 'topic', '/computed_trajectory');   %'std_msgs/Float64MultiArray'
Trajectorymsg = readMessages(TrajectorySelect,'DataFormat','struct');   