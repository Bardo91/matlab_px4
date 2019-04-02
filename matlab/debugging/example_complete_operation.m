%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Take off
% Init service
clientTakeoff = rossvcclient('/ual/take_off');  %rossvcclient creates a Ros service client object that uses a persistant connection to send and receive request from a ROS service;

% Get empty message of the right type
takeoffReq = rosmessage(clientTakeoff); % msg=rosmessage(message type) --> create an empy Ros message object with message type

% Set valies
takeoffReq.Blocking = 1;
takeoffReq.Height = 2.0;

% Call client
takeoffResp = call(clientTakeoff,takeoffReq);

%% Readl pose
% Subscribe to pose
poseSubscriber = rossubscriber('/ual/pose'); % create a Ros subscriber for receiving message 

for i=1:100
    msg = poseSubscriber.receive(1);           %messaggio contenente i valori della posizione per ogni i
    display(msg.Pose.Orientation);
    pause(0.03);
end

%% Move
% Send speed message
speedPublisher = rospublisher('/ual/set_velocity','geometry_msgs/TwistStamped');
msg = rosmessage(speedPublisher);

for i=1:100
    msg.Twist.Linear.Z = 1.0;
    send(speedPublisher, msg)
    pause(0.03);
end

for i=1:100
    msg.Twist.Linear.X = 1.0;
    send(speedPublisher, msg)
    pause(0.03);
end

for i=1:100
    msg.Twist.Linear.X = -1.0;
    send(speedPublisher, msg)
    pause(0.03);
end

%% Land
% Init service
clientLand= rossvcclient('/ual/land');

% Get empty message of the right type
landReq = rosmessage(clientLand);

% Set valies
landReq.Blocking = 1;

% Call client
landResp = call(clientLand, landReq);
