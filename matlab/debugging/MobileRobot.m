classdef MobileRobot < handle
  properties(Access = public)
      pose;
      poseSubscriber
  end
    methods(Access = public)
        function obj = MobileRobot()  % Constructor   
            obj.poseSubscriber = rossubscriber('/ual/pose', @obj.callbackPose);  
            figure(100);
        end
        
        function takeoffResp = takeoff(obj, altitude)
            clientTakeoff = rossvcclient('/ual/take_off');  
            takeoffReq = rosmessage(clientTakeoff); 
            takeoffReq.Blocking = 1;
            takeoffReq.Height = altitude;
            takeoffResp = call(clientTakeoff,takeoffReq);
            pause(3)
        end
        
        function landResp = land(obj)
            clientLand = rossvcclient('/ual/land');  
            landReq = rosmessage(clientLand); 
            landReq.Blocking = 1;
            landResp = call(clientLand,landReq);
            pause(3)
        end
        
        function  callbackPose(obj, src, msg) 
            obj.pose = msg.Pose.Position;
            figure(100);
            plot3(msg.Pose.Position.X, msg.Pose.Position.Y, msg.Pose.Position.Z, 'ob')
        end
    end
end