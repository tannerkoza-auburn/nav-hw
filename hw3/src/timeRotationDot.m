function CbnDot = timeRotationDot(t)

CbnDot = [-sin(t), sin(t^2)*cos(t) + 2*t*cos(t^2)*sin(t),   cos(t^2)*cos(t) - 2*t*sin(t^2)*sin(t);
       0,                         -2*t*sin(t^2),                           -2*t*cos(t^2);
-cos(t), 2*t*cos(t^2)*cos(t) - sin(t^2)*sin(t), - cos(t^2)*sin(t) - 2*t*sin(t^2)*cos(t)];

end