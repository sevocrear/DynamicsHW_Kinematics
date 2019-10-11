    function A = HomoGenTrans(thetai, di, ai, alphai)
    %syms theta short;
    Rotz = [cos(thetai) -sin(thetai) 0.0 0.0
                   sin(thetai) cos(thetai) 0.0 0.0
                   0.0 0.0 1.0 0.0
                   0.0 0.0 0.0 1.0];
    Transz = [1.0 0.0 0.0 0.0
                   0.0 1.0 0.0 0.0
                   0.0 0.0 1.0 di
                   0.0 0.0 0.0 1.0];
    Transx = [1.0 0.0 0.0 ai
                   0.0 1.0 0.0 0.0
                   0.0 0.0 1.0 0.0
                   0.0 0.0 0.0 1.0];
    Rotx = [1.0 0.0 0.0 0.0
                   0.0 cos(alphai) -sin(alphai) 0.0
                   0.0 sin(alphai) cos(alphai) 0.0
                   0.0 0.0 0.0 1.0];


    B = vpa(Rotz*Transz*Transx*Rotx);
    A = simplify(B);
    