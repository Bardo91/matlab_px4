function d = point_to_line(pt, v1, v2)
      a = v1 - v2;
      b = pt - v2;
      d = norm(cross(a,b)) / norm(a);
      
% v1 = [0,0,0];
% v2 = [3,0,0];
% pt = [0,5,0];
% distance = point_to_line(pt,v1,v2)