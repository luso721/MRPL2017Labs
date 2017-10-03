%doesn't need to be a class since the class would only include 
% 1 function. V would be the velocity of the robot currently. 
function [V, w] = controller(rob_pose, ref_pose, V)
   %control parameters
   %disp(V)
   tau = 120;
   k_x = 1/tau;
   k_y = 1/(tau^2*.8);
   %if (abs(V) <k .01)
   %    k_y = 0;
   %else 
   %    k_y = 2./(tau^2*abs(V));
   %end
   
   k_theta = 1/tau;
   

   x_r = rob_pose(1);
   y_r = rob_pose(2);
   th_r = rob_pose(3);
            
   x_ref = ref_pose(1);
   y_ref = ref_pose(2);
   th_ref = ref_pose(3);
            
   %distantce error in world coordinates
   err_w = [x_ref - x_r; y_ref - y_r];
   ksi = th_ref - th_r; %bearing error
   e_theta = atan2(sin(ksi), cos(ksi));   
            
   %convert the error to robot coords, using rotation matrix:
   rot_matrix = [cos(th_r) -1*sin(th_r); sin(th_r) cos(th_r)];
   rot_matrix_inv = inv(rot_matrix);
            
   %use inverted matrix to obtain x-y error of rob w.r.t traj pt.
   %this is slowish, but the inverse is needed.
   err_r = rot_matrix_inv*err_w;  %#ok<MINV> 
            
   %compute porportional control, according to handout.
   u_p = [k_x 0; 0 k_y]*err_r;
   u_p(2) = u_p(2) + (k_theta*e_theta);
   u_v = u_p(1);
   u_w = u_p(2);
   
   %these are the linear and angular velocities that we must add to the 
   %current robot velocities as the feedback. 
   V = u_v;
   w = u_w;
           
end
