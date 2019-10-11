function T = T(Roll, Pitch, Yaw, x, y, z)
%Calculates the T matrix for the position and orientation of the tool
q = [Roll; Pitch; Yaw];
q = rad2deg(q);
R = rotz(q(1))*roty(q(2))*rotx(q(3));
d = [x; y; z];
T = [R(1,1) R(1,2) R(1,3) d(1)
     R(2,1) R(2,2) R(2,3) d(2)
     R(3,1) R(3,2) R(3,3) d(3)
        0     0       0     1];
end

