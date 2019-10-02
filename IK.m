function A57 = IK(DH5, DH6, DH7)
    %Used for calculating in symbolic view the Homog. matrix for last 3
    %angles
    
    %syms q1;
    %syms q2;
    %syms q3;
    syms q5;
    syms q6;
    syms q7;
    
     A5 = (HomoGenTrans(q5,DH5(2),DH5(3),DH5(4)));
     A6 = (HomoGenTrans(q6,DH6(2),DH6(3),DH6(4)));
     A7 = (HomoGenTrans(q7,DH7(2),DH7(3),DH7(4)));
     A57 = A5*A6*A7;
     A57 = simplify(A57);



end

    