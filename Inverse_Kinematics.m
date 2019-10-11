function q = Inverse_Kinematics(T)
    digits(5);
    %syms q1;
    %syms q2;
    %syms q3;
    %syms q4;
    syms q5;
    syms q6;
    syms q7;
   %Configuration and DH-parameters
   L0 = 0; L11 = 0.324; L12 = 0.312; L2 = 1.075; L31 = 0.225;
   L32 = 1.076; L4 = 0.204; L5 = 0.215;
   alpha = [-pi/2 0 -pi/2 -pi/2 pi/2 pi/2];
   DH1 = [0   L11 L12	alpha(1)];
   DH2 = [0	0	L2	alpha(2)];
   DH3 = [0	0	L31	alpha(3)];
   DH4 = [0 	L4+L32	0	alpha(4)];
   DH5 = [0	0	0	alpha(5)];
   DH6 = [0	L5	0	alpha(6)];
   
   %Calculating the position of the tool
    coord_tool = [T(1,4)
        T(2,4)
        T(3,4)];
    %Limits of workspace
    if (coord_tool(3)<=-0.716)
       fprintf('position is out of robots motion range, please, change it');
       return;
   end   
   R = [T(1,1) T(1,2) T(1,3)
        T(2,1) T(2,2) T(2,3)
        T(3,1) T(3,2) T(3,3)];
   coord_c = coord_tool-L5*R*[0;1;0];% Coord of point C
   coord_c = coord_c';
   coord_tool_c = coord_tool-coord_c'
   
   %Calculating thetas
   theta1 = atan2(coord_c(2), coord_c(1));
   theta1 = round(theta1,4);
   
    %Some needed parameters (see pictures)
    L34 = L32+L4;
    p = sqrt(L31^2+L34^2);
    q = sqrt(coord_c(1)^2+coord_c(2)^2)-L12;
    s = sqrt((L11-coord_c(3))^2+q^2);
    
    ca = (L2^2+s^2-p^2)/(2*L2*s);
    a = atan2(sqrt(1-ca^2), ca);
    b = atan2(L11-coord_c(3), q);
    theta2 = b-a;
    theta2 = round(theta2,4);
    
    fi = atan2(L34, L31);
    cpsi = (L2^2+p^2-s^2)/(2*L2*p);
    psi = atan2(sqrt(1-cpsi^2), cpsi);
    
    theta3 = pi-fi-psi;
    theta3 = round(theta3, 4);
    
    q = [theta1 theta2 theta3];
   %Limits
   if (q(1)<=-pi) || (q(1)>=pi)
       fprintf('theta1 is out of robots motion range, please, change it');
       return;
   end
  if (q(2)<=-(136/180/2)*pi-pi/2) || (q(2)>=(136/180/2)*pi+pi/2)
   fprintf('theta2 is out of robots motion range, please, change it');
   return;
  end
  if (q(3)<=-(312/180/2)*pi) || (q(3)>=(312/180/2))
   fprintf('theta3 is out of robots motion range, please, change it');
   return;
  end   
   digits(5)
   
%Calculating the Homogeneous matrix from 0 to 4. 
   alpha = rad2deg(alpha);
   q = rad2deg(q);
A03 = HRot('z',q(1))*transl(0,0,DH1(2))* transl(DH1(3),0,0)*HRot('x',alpha(1))* ...
        HRot('z',q(2))*transl(0,0,DH2(2))* transl(DH2(3),0,0)*HRot('x',alpha(2))* ...
        HRot('z',q(3))*transl(0,0,DH3(2))* transl(DH3(3),0,0)*HRot('x',alpha(3));
%Calculating the Homogeneous matrix from 4 to 6. 
    T46 = inv(A03)*T;
 
    theta4 = pi-atan2(T46(2,2), T46(1,2));
    theta5 = atan2(sqrt(1-T46(3,2)^2),T46(3,2));
    theta6 = atan2(T46(3,3), (T46(3,1)));
    
    theta4 = zero_el(theta4);
    theta5 = zero_el(theta5);
    theta6 = zero_el(theta6);
    %Some numerical method for calculating three last angles (only
    %positive, and up-handed). Were set during experiments.
    if (coord_tool_c(3)<0) || (coord_tool_c(2)<0)
        theta4 = pi-theta4;
        theta6 = pi+theta6;
    end
    if (coord_tool_c(1) <0)
        theta4 = -theta4;
        theta6 = pi+theta6;
    end
    if (coord_tool_c(3)  == 0)
        theta6 = pi+theta6;
    end
    if (coord_tool_c(1)  > 0)&& (coord_tool_c(2)  > 0)&& (coord_tool_c(3)  > 0)
        theta6 = pi+theta6;
        theta4 = pi-theta4;%
    end
    if (coord_tool_c(1)  < 0)&& (coord_tool_c(2)  < 0)&& (coord_tool_c(3)  < 0)
        theta6 = -pi+theta6;
        theta4 = -theta4;
    end
    if (coord_tool_c(1) <0)&&(coord_tool_c(2) <0)&&(coord_tool_c(3) >0)
        theta4 = -theta4;
        theta6 = theta6-pi;
    end
    if (coord_tool_c(1) ==0)&&(coord_tool_c(2) <0)&&(coord_tool_c(3) ==0)
        theta4 = theta4;
        theta6 = theta6-pi;
    end
    if (coord_tool_c(1) ==0)&&(coord_tool_c(2) <0)&&(coord_tool_c(3) ==0)
        theta4 = theta4;
        theta6 = theta6-pi;
    end
    if (coord_tool_c(1) >0)&&(coord_tool_c(2) <0)&&(coord_tool_c(3) ==0)
        theta4 = theta4;
        theta6 = theta6-pi;
    end
    if (coord_tool_c(1) <0)&&(coord_tool_c(2) >0)&&(coord_tool_c(3)>0)
        theta4 = theta4+pi;
        theta6 = theta6;
    end
    if (coord_tool_c(1) >0)&&(coord_tool_c(2) >0)&&(coord_tool_c(3) ==0)
        theta4 = theta4;
        theta6 = theta6-pi;
    end
    if (coord_tool_c(1) <0)&&(coord_tool_c(2) >0)&&(coord_tool_c(3) <0)
        theta4 = -theta4;
        theta6 = theta6-pi;
    end
    
    %The final axis' angles
    q0 = [zero_el(theta1) zero_el(theta2+pi/2) zero_el(theta3) theta4 theta5 theta6];
    q = vpa(q0);
   q0 = [zero_el(theta1) zero_el(theta2+pi/2)-pi/2 zero_el(theta3) theta4 theta5 theta6];
   %Plotting
   alpha = [-pi/2 0 -pi/2 -pi/2 pi/2 pi/2];
   DH1 = [q0(1)   L11 L12	alpha(1)];
   DH2 = [q0(2)	0	L2	alpha(2)];
   DH3 = [q0(3)	0	L31	alpha(3)];
   DH4 = [q0(4) 	L4+L32	0	alpha(4)];
   DH5 = [q0(5)	0	0	alpha(5)];
   DH6 = [q0(6)	L5	0	alpha(6)];
   f2 = figure('name','Inverse Kinematics');
   figure(f2);
   R_base = rotz(0);
   trplot(R_base,'color','black')
   L(1) = Link('revolute',DH1);
   L(2) = Link('revolute',DH2);
   L(3) = Link('revolute',DH3);
   L(4) = Link('revolute',DH4);
   L(5) = Link('revolute',DH5);
   L(6) = Link('revolute',DH6);
   R = SerialLink(L, 'name', 'FANUC R-2000iC/165F');
   R.plot(q0)
   
end
   
    
    