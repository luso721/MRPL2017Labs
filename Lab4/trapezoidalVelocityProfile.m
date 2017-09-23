function uref = trapezoidalVelocityProfile(t , amax, vmax, dist, sgn)
%uref Return the velocity command of a trapezoidal profile.
% Returns the velocity command of a trapezoidal profile of maximum
% acceleration amax and maximum velocity vmax whose ramps are of
% duration tf. Sgn is the sign of the desired velocities.
% Returns 0 if t is negative.  

s_f = dist;
t_ramp = vmax/amax;
t_f = (s_f/vmax) + t_ramp;

if (t < t_ramp)
    uref = amax*t;

if ((t_f - t) < t_ramp)
    uref = -1*amax*(t_f -t);
end

if ((t_ramp) < t && (t < t_f - t_ramp))
    uref = vmax;
end

if (t < 0)
    uref = 0;
end

if (s_f < 0)
    uref = uref * -1;
end

uref = sgn*uref;

    

