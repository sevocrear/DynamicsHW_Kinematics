   clc;clear;
   format long
   %% Direct Kinematics
  % Please, enter the angles:
   theta1 = 3*pi/4;
   theta2 = pi/4;
   theta3 = pi/4;
   theta4 = pi/8;
   theta5 = pi/4;
   theta6 = pi/4;
   
   %This line (down) is not allowed to change.
   q0 = [theta1; theta2; theta3; 0; theta4; theta5; theta6]; %Set rotations;

   
%    T_DK = Direct_Kinematics(q0)

   %% Inverse Kinematics
%There is possibility to set your own T matrix or go through the solution
%of Direct Kinematics (Unbracking needed)
%Ctrl-R --> Add brackets
%Ctrl-T--> Remove brackets

%Your own

   T =[     1          0         0       1.5
            0          -1        0        1
            0          0         1        1
            0          0         0        1];

%DK_Solution
   
    %T = T_DK;

   
%The function of IK_Solver that gives us the needed joint's angles
   q_Solved = Inverse_Kinematics(T)
   
 %For DK_Solution  
 
%    q_Solved=q_Solved.';
%    q_Direct =q0;
%    Martix = [vpa(q_Direct), q_Solved]   
   
  %Done
   