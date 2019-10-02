function Rot = HRot(axis, angle)
    %Create HT matrix
    if axis == 'x'
        Rot = transl(0,0,0)*trotx(angle);
    elseif axis == 'y'  
        Rot = transl(0,0,0)*troty(angle);
    elseif axis == 'z'  
        Rot = transl(0,0,0)*trotz(angle);
    end