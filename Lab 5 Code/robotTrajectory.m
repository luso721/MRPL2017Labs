classdef robotTrajectory < handle
    
    properties (Access = private)
        time_samples;
        distance_samples;
        velocity_samples;
        pose_samples;
    end
    
    methods
        
        function obj = robotTrajectory()
            
        end
        
        
        function pose = getPoseAtTime(obj, t)
            pose = 0;
        end
    end
    
   
end