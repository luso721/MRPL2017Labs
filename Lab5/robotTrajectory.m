classdef robotTrajectory < handle
    
    properties (Constant)
       dt = 0.001;
    end
    
    properties (Access = public)
        x;
        y;
        th;
        p;
        s;
        t;
        V;
        w;
        numSamples;
        
        time_samples;
        distance_samples;
        velocity_samples;
        pose_samples;
    end
    
    methods
        
        function obj = robotTrajectory(p0, s0, referenceControl)
            x(1) = p0(1);
            y(1) = p0(2);
            th(1) = p0(3);
            p(1, 1:3) = p0;  % pose
            s(1) = s0;  % distance
            
            duration = getTrajectoryDuration(referenceControl);
            obj.numSamples = round(duration / obj.dt);
            
            for (i=1:obj.numSamples-1)
                t(i) = (i-1) * obj.dt;
                
                [V_ref, w_ref] = computeControl(referenceControl, t(i));
                
                V(i) = V_ref;
                w(i) = w_ref;
                s(i+1) = s(i) + V(i)*obj.dt;
                
                x(i+1) = x(i) + V(i)*cos(th(i))*obj.dt;
                y(i+1) = y(i) + V(i)*sin(th(i))*obj.dt;
                th(i+1) = th(i) + w(i)*obj.dt;
                p(i+1, 1:3) = [x(i+1), y(i+1), th(i+1)];
            end
            t(obj.numSamples) = (obj.numSamples-1)*obj.dt;
            [V_ref, w_ref] = computeControl(referenceControl, t(obj.numSamples));
            V(obj.numSamples) = V_ref;
            w(obj.numSamples) = w_ref;
            
            obj.t = t;
            obj.V = V;
            obj.w = w;
            obj.s = s;
            obj.x = x;
            obj.y = y;
            obj.th = th;
            obj.p = p;
        end
        
        
        function pose = getPoseAtTime(obj, time)
            index = time / obj.dt;
            if (index > size(obj.t, 2))
                pose = obj.p(end, 1:3);
            elseif (rem(index, 1) == 0)
                pose = obj.p(index, 1:3);
            else
                a = floor(index);
                b = ceil(index);
                
                tq = [obj.t(a), time, obj.t(b)];
                xq = interp1(t, x, tq);
                yq = interp1(t, y, tq);
                thq = interp1(t, th, tq);
                
                pose = [xq(2), yq(2), thq(2)];
            end
        end
    end
    
   
end