%% Take off

clientTakeoff = rossvcclient('/ual/take_off');  %rossvcclient creates a Ros service client object that uses a persistant connection to send and receive request from a ROS service;

% Get empty message of the right type
takeoffReq = rosmessage(clientTakeoff); % msg=rosmessage(message type) --> create an empy Ros message object with message type

% Set valies
takeoffReq.Blocking = 1;
takeoffReq.Height = 2.0;

% Call client
takeoffResp = call(clientTakeoff,takeoffReq);
