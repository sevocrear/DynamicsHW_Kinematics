   clc;clear;
   format long
   %% Direct Kinematics
  % Please, enter the angles:
  
   theta1 = pi/2;
   theta2 = pi/3;
   theta3 = pi/4;
   theta4 = pi/8;
   theta5 = pi/4;
   theta6 = 3*pi/4;
   
   %This line (down) is not allowed to change.
   q0 = [theta1; theta2; theta3; theta4; theta5; theta6]; %Set rotations;

   
     T_DK = Direct_Kinematics(q0)

   %% Inverse Kinematics
%There is possibility to set your own T matrix or go through the solution
%of Direct Kinematics (Unbracking needed)
%Ctrl-R --> Add brackets
%Ctrl-T--> Remove brackets

%Your own

%    T =[     1          0         0       1.5
%             0          -1        0        1
%             0          0         1        1
%             0          0         0        1];
        

%Through the Roll, Pitch, Yaw Angles


%         Roll = pi/4;
%         Pitch = pi/3;
%         Yaw = pi/8;
%         x = -1;
%         y = 0.5;
%         z = 1.5;
%         
%         T = T(Roll, Pitch, Yaw, x, y, z)


%DK_Solution
   
     T = T_DK;

   
%The function of IK_Solver that gives us the needed joint's angles

   q_Solved = Inverse_Kinematics(T)
   
 %For DK_Solution  
 
   q_Solved=q_Solved.';
   q_Direct =q0;
   Martix = [vpa(q_Direct), q_Solved]   
   
  %Done
   