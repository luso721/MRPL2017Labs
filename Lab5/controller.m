classdef controller < handle
    
    properties (Constant)
    end
    
    properties (Access = public)
        tau;
        k_x;
        k_y;
        k_theta;
        x_r;
        y_r;
        th_r;
        x_ref;
        y_ref;
        th_ref;
    end
    
    methods
        function obj = controller(rob_pose, ref_pose, V)
            %control parameters
            %disp(V)
            obj.tau = 1.0;
            obj.k_x = 1/obj.tau;
            %obj.k_y = 2/(obj.tau^2*abs(V));
            if (abs(V) < .01)
                obj.k_y = 0;
            else 
                obj.k_y = 2/(obj.tau^2*abs(V));
           end
   
            obj.k_theta = 1/obj.tau;
   

            obj.x_r = rob_pose(1);
            obj.y_r = rob_pose(2);
            obj.th_r = rob_pose(3);
            
            obj.x_ref = ref_pose(1);
            obj.y_ref = ref_pose(2);
            obj.th_ref = ref_pose(3);            
           
        end
        
        function [err_r, e_theta] = computeError(obj)
            %distantce error in world coordinates
            err_w = [obj.x_ref - obj.x_r; obj.y_ref - obj.y_r];
            ksi = obj.th_ref - obj.th_r; %bearing error
            e_theta = atan2(sin(ksi), cos(ksi));   
            
            %convert the error to robot coords, using rotation matrix:
            rot_matrix = [cos(obj.th_r) -1*sin(obj.th_r); sin(obj.th_r) cos(obj.th_r)];
            rot_matrix_inv = inv(rot_matrix);
            
            %use inverted matrix to obtain x-y error of rob w.r.t traj pt.
            %this is slowish, but the inverse is needed.
            err_r = rot_matrix_inv*err_w;  %#ok<MINV> 
        end
                
        function [V, w] = computeFeedback(obj)
            %compute porportional control, according to handout.
            [err_r, e_theta] = computeError(obj);
            
            u_p = [obj.k_x 0; 0 obj.k_y]*err_r;
            %u_p(1) = obj.k_y*err_r(2);
            %u_p(2) = obj.k_x*err_r(1) + obj.k_theta*e_theta;
            u_p(2) = u_p(2) + (obj.k_theta*e_theta);
            u_v = u_p(1);
            u_w = u_p(2);
   
            %these are the linear and angular velocities that we must add to the 
            %current robot velocities as the feedback. 
            V = u_v;
            w = u_w;
        end
        
    end
end
