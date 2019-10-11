function A46 = IK(DH4, DH5, DH6)
    %Used for calculating in symbolic view the Homog. matrix for last 3
    %angles
    
    %syms q1;
    %syms q2;
    %syms q3;
    syms q4;
    syms q5;
    syms q6;
    
     A4 = (HomoGenTrans(q4,DH4(2),DH4(3),DH4(4)));
     A5 = (HomoGenTrans(q5,DH5(2),DH5(3),DH5(4)));
     A6 = (HomoGenTrans(q6,DH6(2),DH6(3),DH6(4)));
     A46 = A4*A5*A6;
     A46 = simplify(A46);



end

    