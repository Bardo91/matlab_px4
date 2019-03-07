%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Take off
% Init service
clientTakeoff = rossvcclient('/ual/take_off');

% Get empty message of the right type
takeoffReq = rosmessage(clientTakeoff);

% Set valies
takeoffReq.Blocking = 1;
takeoffReq.Height = 2.0;

% Call client
takeoffResp = call(clientTakeoff,takeoffReq);

%% Readl pose
% Subscribe to pose
poseSubscriber = rossubscriber('/ual/pose');

for i=1:100
    display(poseSubscriber.receive(1).Pose.Position);
    pause(0.03);
end

%% Move
% Send speed message
speedPublisher = rospublisher('/ual/set_velocity','geometry_msgs/TwistStamped');

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
