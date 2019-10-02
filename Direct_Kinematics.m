function T_solved = Direct_Kinematics(q0)
   %Using DH parameters
   %L = Link ([Th d a alpha])
   %Th - the angle about the prev z to aligh its x to the new x
   %d - the depth along the previous joint's z axis to the common normal
   %a - is the distance along the rotated x axis (radius of rotation about
   %previous z axis)
   %alpha - angle of rotation about the new x axis to put previous z in its
   %desired orientation
   %if z and z next are parallel then a = 0. d is various
   %Configuration, DH-parameters
   L11 = 0.324; L12 = 0.312; L2 = 1.075; L31 = 0.225;
   L32 = 1.076; L4 = 0.204; L5 = 0.215;
   q = [q0(1) q0(2)-pi/2 q0(3) q0(4) q0(5) q0(6) q0(7)]; %Set rotations;
   alpha = [-pi/2 0 -pi/2 0 -pi/2 pi/2 pi/2];
   DH1 = [q(1)   L11 L12	alpha(1)];
   DH2 = [q(2)	0	L2	alpha(2)];
   DH3 = [q(3)	0	L31	alpha(3)];
   DH4 = [q(4) 	L4+L32	0	alpha(4)];
   DH5 = [q(5)	0	0	alpha(5)];
   DH6 = [q(6)	0	0	alpha(6)];
   DH7 = [q(7)	L5	0	alpha(7)];
   clear L;
    %Limits
   if (q(1)<=-pi) || (q(1)>=pi)
       fprintf('q1 is out of robots motion range, please, change it');
       return;
   end
  if (q(2)<=-(136/180/2)*pi-pi/2) || (q(2)>=(136/180/2)*pi+pi/2)
   fprintf('q2 is out of robots motion range, please, change it');
   return;
  end
  if (q(3)<=-(312/180/2)*pi) || (q(3)>=(312/180/2))
   fprintf('q3 is out of robots motion range, please, change it');
   return;
  end
  if (q(5)<=-(360/180/2)*pi) || (q(5)>=(360/180/2)*pi)
   fprintf('q4 is out of robots motion range, please, change it');
   return;
  end
  if (q(6)<=-(250/180/2)*pi) || (q(6)>=(250/180/2)*pi)
   fprintf('q5 is out of robots motion range, please, change it');
   return;
  end
  if (q(7)<=-(360/180/2)*pi) || (q(7)>=(360/180/2)*pi)
   fprintf('q6 is out of robots motion range, please, change it');
   return;
  end
  %Plotting
   f1 = figure('name','Direct Kinematics');
   figure(f1);
   R_base = rotz(0);
   trplot(R_base,'color','black')
   L(1) = Link('revolute',DH1);
   L(2) = Link('revolute',DH2);
   L(3) = Link('revolute',DH3);
   L(4) = Link(DH4);
   L(5) = Link('revolute',DH5);
   L(6) = Link('revolute',DH6);
   L(7) = Link('revolute',DH7);
   R = SerialLink(L, 'name', 'FANUC R-2000iC/165F');
   R.plot(q)
   hold off
   %Exam the Forward Kinematics solution
   T_validate = R.fkine(q)
   %To use my own solver for FK at first we should convert rad to deg
   alpha = rad2deg(alpha);
   q = rad2deg(q);
   T_solved = FK(q, alpha, DH1, DH2, DH3, DH4, DH5, DH6, DH7);
