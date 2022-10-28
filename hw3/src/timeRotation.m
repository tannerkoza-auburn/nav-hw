function Cbn = timeRotation(t)

Cbn = [cos(t), sin(t) * sin(t^2), sin(t) * cos(t^2); ...
        0, cos(t^2), -sin(t^2); ...
        -sin(t), cos(t) * sin(t^2), cos(t) * cos(t^2)];

end