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

%% Read linear velocity
% Subscribe to velocity
velocitySubscriber = rossubscriber('/ual/velocity'); % create a Ros subscriber for receiving message 

for i=1:100
    msg = velocitySubscriber.receive(1);           %messaggio contenente i valori della posizione per ogni i
    display(msg.Twist.Linear);
    pause(0.03);
end
