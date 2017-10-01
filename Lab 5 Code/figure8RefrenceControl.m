classdef figure8RefrenceControl < handle
    
    properties (Access = private)
       t_f;
       k_theta;
       k_k;
       T_f;
       v;
       k_s;
       k_v;
       t_pause;
    end
    
    methods    
        
        function obj = figure8RefrenceControl(Ks, Kv, tPause)
            % Construct a figure 8 trajectory. It will not start until
            % tPause has elapsed and it will stay at zero for tPause
            % afterwards. Kv scales velocity up when > 1 and Ks scales
            % the size of the curve itself up.
            
            v = 0.2;
            s_f = 1;
            
            obj.t_f = s_f/v;
            obj.k_theta = 2*pi/s_f;
            obj.k_k = 15.1084;
            obj.T_f = (Ks/Kv)*(s_f/v);
            obj.v = 0.2;
            obj.k_s = Ks;
            obj.k_v = Kv;
            obj.t_pause = tPause;
    
        end
        
        function [V, w] = computeControl(obj, timeNow)
            % Return the linear and angular velocity that the robot 
            % should be executing at time timeNow. Any zero velocity
            % pauses specified in the constructor are implemented here
            % too.
            
            %Check some edge cases. (1) Negative time is bad, so 
            %just publish zero velocity. (2) It time is within pause, 
            %publish zero velocity. (3) If time is during final terminal  
            %pause, publish zero velocity. Otherwise, by elimniation, timeNow
            %must be during the trajectory time. 
            if (timeNow < 0 || (0 < timeNow && timeNow < obj.t_pause) ...
                || (obj.T_f < timeNow && timeNow < obj.T_f + obj.t_pause)...
                || (obj.T_f + (2*obj.t_pause) < timeNow))
                V = 0;
                w = 0;
                return;
            end
            
            %loop_pause = .05;
            first_iteration = false;
            index = 1;
            while(true)
                if (first_iteration == false)
                    start_tic = tic();
                    
                    %time at the start of the first iteration
                    start_time = toc(start_tic); 
                    first_iteration = true;
                end
            
                current_time = toc(start_tic);
                T = current_time;
                
                dt = current_time - start_time; 
                
                if(dt > timeNow)
                    break;
                end
            
                %convert to unscaled time. 
                t = (obj.k_s/obj.k_v)*(T);
            
                %compute the trajectory
                s = obj.v*t;
                kappa = (obj.k_k/obj.k_s)*sin(obj.k_theta * s);
                V = obj.k_v*obj.v;
                omega = kappa*V;
                       
                % compute total elapsed time
                
                %pause(loop_pause);
                index = index + 1;
            end 
            w = omega;
        end
        
        
        function duration  = getTrajectoryDuration(obj)
            duration = obj.T_f + (2*obj.t_pause);
        end
        
    end
end





