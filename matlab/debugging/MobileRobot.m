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
        function  callbackPose(obj, src, msg) 
            obj.pose = msg.Pose.Position;
            figure(100);
            plot3(msg.Pose.Position.X, msg.Pose.Position.Y, msg.Pose.Position.Z, 'ob')
        end
    end
end