%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Init ROS
rosinit

% Init service
clientTakeoff = rossvcclient('/ual/take_off');

% Get empty message of the right type
takeoffReq = rosmessage(clientTakeoff);

% Set valies
takeoffReq.Blocking = 1;
takeoffReq.Height = 2.0;

% Call client
takeoffResp = call(clientTakeoff,takeoffReq,'Timeout',3);