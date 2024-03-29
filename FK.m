function T = FK(q, alpha, DH1, DH2, DH3, DH4, DH5, DH6)
    digits(2);
%Function calculates the T0_6 matrix

    T = HRot('z',q(1))*transl(0,0,DH1(2))* transl(DH1(3),0,0)*HRot('x',alpha(1))* ...
        HRot('z',q(2))*transl(0,0,DH2(2))* transl(DH2(3),0,0)*HRot('x',alpha(2))* ...
        HRot('z',q(3))*transl(0,0,DH3(2))* transl(DH3(3),0,0)*HRot('x',alpha(3))* ...
        HRot('z',q(4))*transl(0,0,DH4(2))* transl(DH4(3),0,0)*HRot('x',alpha(4))* ...
        HRot('z',q(5))*transl(0,0,DH5(2))* transl(DH5(3),0,0)*HRot('x',alpha(5))* ...
        HRot('z',q(6))*transl(0,0,DH6(2))* transl(DH6(3),0,0)*HRot('x',alpha(6))
   
end
    