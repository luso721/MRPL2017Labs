function uref = trapezoidalVelocityProfile(t , amax, vmax, dist, sgn)
%uref Return the velocity command of a trapezoidal profile.
% Returns the velocity command of a trapezoidal profile of maximum
% acceleration amax and maximum velocity vmax whose ramps are of
% duf_ration tf. Sgn is the sign of the desired velocities.
% Returns 0 if t is negative.  

s_f = dist;
t_ramp = vmax/amax;
t_f = (s_f + (vmax^2/amax))/vmax;

if (t < t_ramp)
    uref = amax*t;

elseif ((t_f - t) < t_ramp)
    %sgn accounts for dicrection of accelration
    uref = 1*amax*(t_f-t);

elseif ((t_ramp) < t && (t < t_f - t_ramp))
    uref = vmax;
    
else 
    uref = 0;
end

uref = sgn*uref;

    

