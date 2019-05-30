%% Land
% Init service
clientLand= rossvcclient('/ual/land');

% Get empty message of the right type
landReq = rosmessage(clientLand);

% Set valies
landReq.Blocking = 1;

% Call client
landResp = call(clientLand, landReq);

rosshutdown
