function [x,y,th] = irToXy( i, r )
  
  if (r < .06)
      r = 0;
  end
  
  offset = 4; 
  th = i + offset;

  th = th*pi/180;
  
  if (th > pi)
      th = th - 2*pi;
  end
   
  x = r*cos(th);
  y = r*sin(th);
  
end