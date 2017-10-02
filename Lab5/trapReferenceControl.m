classdef trapReferenceControl < handle
    
    properties (Access = public)
        v_max;
        a_max;
        t_ramp; 
        t_f;        
    end
    
    methods (Static = true)
        
        function obj = trapReferenceControl(vMax, aMax)
            obj.v_max = vMax;
            obj.a_max = aMax;
            obj.t_ramp = vMax/aMax;
            obj.t_f = (1 + vMax^2/aMax)/vMax;
        end
        
        function V_ref = computeControl(obj, time_now)
            
            if (time_now <= 0)
                V_ref = 0;
                return 
            
            elseif (time_now < obj.t_ramp)
                V_ref = obj.a_max * time_now;
                return 
            
            elseif (time_now > obj.t_f)
                V_ref = 0;
                return
                
            elseif ((obj.t_f - time_now) < t_ramp)
                V_ref = obj.a_max*(obj.t_f - time_now);
                return 
                
            elseif ((time_now > obj.t_ramp) && (time_now < obj.t_f - obj.t_ramp))
                V_ref = obj.v_max;
                
            else
                V_ref = 0;
            end

        end
        
        function duration = getTrajectoryDuration(obj)
            duration = obj.t_f;
        end
    end
    
end