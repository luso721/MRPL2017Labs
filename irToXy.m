function [ x, y,th] = irToXy( i, r )
  
  if (r < .06)
      r = 0;
  end
  
  th = i + 4;
  th = th*pi/180;
  
  if (th > pi)
      th = th - 2*pi;
  end
   
  x = r*cos(th);
  y = r*sin(th);
  
end