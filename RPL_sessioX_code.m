%////////////////////
%    sessio2.m
%////////////////////



%================%
% MAIN FUNCTION  %
%================%


% ====================================== %
% INICI CODI SESSIÓ 1                    %
% ====================================== %

%////////////////////
%    sessio1.m
%////////////////////



%================%
% MAIN FUNCTION  %
%================%



function sessio1
    
    % We first close all the figures.
    close all;
 
%==========================================================================
% 1./ We plot the 3d canvas:
%--------------------------------------------------------------------------
    
    % Draw the axes, choose the intervals and set the view.  
    plot_canvas;
    
 
 
%==========================================================================
% 2./ We calculate the homogeneous transforms from the DH parameters 
%     for the initial configuration theta_init:
%--------------------------------------------------------------------------
 
    % The initial configuration (configuration shown A)
    theta_init = [90, -90, -90, 0, 0, 0]';
    
    theta_init = pas_radians(theta_init);
 
    % The DH parameters of PUMA 560
    DH = [  0     0     0   theta_init(1)   % i=1
          -90     0     0   theta_init(2)   % i=2
            0   650   190   theta_init(3)   % i=3
          -90     0   600   theta_init(4)   % i=4
           90     0     0   theta_init(5)   % i=5
          -90     0     0   theta_init(6)   % i=6
          ];
        
    % The homogeneous transforms TT from the DH parameters.
    TT = get_homogeneous_transforms(DH);
   
 

%==========================================================================
% 3./ We plot the joints positions for the initial configuration:
%--------------------------------------------------------------------------
    
    % Joints are visualized on the canvas.
    %plot_joints(TT);
    

 
%==========================================================================
% 4./ Simulate links as straight lines connecting joints:
%--------------------------------------------------------------------------
 
    % We connect the joints with straight lines to simulate the links.
    simulate_links(TT);    
    %pause;  % a bit, to watch the plot of the configuration shown.
 
    
    
%==========================================================================
% 5./ Repeate for a new configuration:
%--------------------------------------------------------------------------
 
    % The new configuration (configuration shown B).

    thetaB = [45, -60, 30, 0, 0, 0]';
    
    thetaB = pas_radians(thetaB);
            
    %------------------------------------------------------------------%
    % TO DO: Write new call to the functions to implement the new conf.% 
    %------------------------------------------------------------------%
    
    DH2 = [  0     0     0   thetaB(1)   % i=1
           -90     0     0   thetaB(2)   % i=2
             0   650   190   thetaB(3)   % i=3
           -90     0   600   thetaB(4)   % i=4
            90     0     0   thetaB(5)   % i=5
           -90     0     0   thetaB(6)   % i=6
          ];
         
    % The homogeneous transforms TT from the DH parameters.
    TT2 = get_homogeneous_transforms(DH2);
    
%==========================================================================
% 5.3./ We plot the joints positions for the initial configuration:
%--------------------------------------------------------------------------
    
    % Joints are visualized on the canvas.
    %plot_joints(TT2);
    
%==========================================================================
% 5.4./ Simulate links as straight lines connecting joints:
%--------------------------------------------------------------------------
 
    % We connect the joints with straight lines to simulate the links.
    simulate_links(TT2);    
    %pause;  % a bit, to watch the plot of the configuration shown.
 
%==========================================================================
% 6./ Modify the Puma robot by transforming Joint 5 into a prismatic joint
%     and provide the position for the given configurations:
%--------------------------------------------------------------------------
    
    %------------------------------------------------------------------%
    % TO DO: Write new confs. and implement the calls. The new confs.  % 
    %        imply changing JOINT 5 by a prismatic joint and solve the %
    %        forward kinematics for d1=100, d1=300 and d1=-300.        %
    %------------------------------------------------------------------%
    
    
      DH3 = [      0     0     0   thetaB(1)   % i=1
                 -90     0     0   thetaB(2)   % i=2
                   0   650   190   thetaB(3)   % i=3
                 -90     0   600   thetaB(4)   % i=4
                   0     0     0   thetaB(5)   % i=5
                 -90     0   100   thetaB(6)   % i=6
          ];
           
      
      
      TT3 = get_homogeneous_transforms(DH3);
      %plot_joints(TT3);
      simulate_links(TT3);
      %pause;
       
    
      DH4 = [      0     0     0   thetaB(1)   % i=1
                 -90     0     0   thetaB(2)   % i=2
                   0   650   190   thetaB(3)   % i=3
                 -90     0   600   thetaB(4)   % i=4
                   0     0     0   thetaB(5)   % i=5
                 -90     0   300   thetaB(6)   % i=6
          ];
           
      TT4 = get_homogeneous_transforms(DH4);
      %plot_joints(TT4);
      simulate_links(TT4); 
      %pause;
          
          
      DH5 = [     0     0     0   thetaB(1)   % i=1
                -90     0     0   thetaB(2)   % i=2
                  0   650   190   thetaB(3)   % i=3
                -90     0   600   thetaB(4)   % i=4
                  0     0     0   thetaB(5)   % i=5
                -90     0  -300   thetaB(6)   % i=6
             ];
        
    
  
      TT5 = get_homogeneous_transforms(DH5);
      %plot_joints(TT5);
      simulate_links(TT5);
      %pause;
      
      
      sessio2;
      
end
 

%======================%
% AUXILIARY FUNCTIONS  %
%======================%

function POS = get_3Dpositions(TT)

    POS.H01 = TT.T01;
    POS.H02 = POS.H01*TT.T12;
    POS.H03 = POS.H02*TT.T23;
    POS.H04 = POS.H03*TT.T34;
    POS.H05 = POS.H04*TT.T45;
    POS.H06 = POS.H05*TT.T56;
    
end

function nouTheta = pas_radians(theta)
    
    for n = 1:6
        nouTheta(n) = theta(n)*2*pi/360;
    end


end


 
%% here we build up a 3D canvas for plotting stuff
function plot_canvas
 
    %------------------------------------------------------------------%
    % TO DO: Initialize the axes, their limits and set the view.       %
    %------------------------------------------------------------------%
    
    xlim([-1000 1000]);
    ylim([-1000 1000]);
    zlim([-1000 1000]);
    
    view([1,1,1]);
    grid on;
    
    hold on;
 
end
 
 
 
%% here the forward kinematics
function TT = get_homogeneous_transforms(DH)
 
    %------------------------------------------------------------------%
    % TO DO: Obtain T(0,6) as composition of the 6 different matrices  %
    %        T(i-1,i). You are expected to obtain each T as a call to  %
    %        function 'tmat', implementing the equation 3.6 in Craig's %
    %        "Introduction to Robotics".                               %                 
    %  HINT: TT may be a structure containing all the T's.             %
    %------------------------------------------------------------------%
   
    TT.T01 = tmat(DH(1, 1), DH(1, 2), DH(1, 3), DH(1, 4));
    TT.T12 = tmat(DH(2, 1), DH(2, 2), DH(2, 3), DH(2, 4));
    TT.T23 = tmat(DH(3, 1), DH(3, 2), DH(3, 3), DH(3, 4));
    TT.T34 = tmat(DH(4, 1), DH(4, 2), DH(4, 3), DH(4, 4));
    TT.T45 = tmat(DH(5, 1), DH(5, 2), DH(5, 3), DH(5, 4));
    TT.T56 = tmat(DH(6, 1), DH(6, 2), DH(6 ,3), DH(6, 4));
end
    
 
 
%% here the tmat function
function T = tmat(alpha, a, d, theta)
 
    %------------------------------------------------------------------%
    % TO DO: Implement here the equation 3.6 in Craig's "Introduction  %
    %         to Robotics".                                            %
    %------------------------------------------------------------------%
    
    T = [cos(theta) -sin(theta) 0 a; 
        sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d ; 
        sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha);
        0 0 0 1];
    
end
 
 

%% here we plot the joints
function plot_joints(TT)
 
    %------------------------------------------------------------------%
    % TO DO: Plot the joints in the canvas.                            %   
    % HINT 1: You can call 'plot_canvas' to rewrite the axes.         %
    % HINT 2: Use the T's to obtain the 3D positions.                  %
    %------------------------------------------------------------------%
    POS = get_3Dpositions(TT);
    plot_canvas;
    
    points = [POS.H01(1, 4) POS.H01(2, 4) POS.H01(3, 4)
              POS.H02(1, 4) POS.H02(2, 4) POS.H02(3, 4)
              POS.H03(1, 4) POS.H03(2, 4) POS.H03(3, 4)
              POS.H04(1, 4) POS.H04(2, 4) POS.H04(3, 4)
              POS.H05(1, 4) POS.H05(2, 4) POS.H05(3, 4)
              POS.H06(1, 4) POS.H06(2, 4) POS.H06(3, 4)];

    plot3(points(:,1),points(:,2),points(:,3), '*');
    disp(points(1, :));
    disp(points(2, :));
    disp(points(3, :));
    disp(points(4, :));
    disp(points(5, :));
    disp(points(6, :));
    
    
    

    grid on;
    hold on;

end
 
 
 
%% here we plot the links
function simulate_links(TT)
 
    %------------------------------------------------------------------%
    % TO DO: Plot the links in the canvas.                             %   
    % HINT 1: You can call 'plot_canvas' to rewrite the axes.          %
    % HINT 2: You can simulate the links by connecting the each pair   % 
    %         of consecutive joints with a straight line.              %                           %
    %------------------------------------------------------------------%
    POS = get_3Dpositions(TT);
    
    points = [POS.H01(1, 4) POS.H01(2, 4) POS.H01(3, 4)
              POS.H02(1, 4) POS.H02(2, 4) POS.H02(3, 4)
              POS.H03(1, 4) POS.H03(2, 4) POS.H03(3, 4)
              POS.H04(1, 4) POS.H04(2, 4) POS.H04(3, 4)
              POS.H05(1, 4) POS.H05(2, 4) POS.H05(3, 4)
              POS.H06(1, 4) POS.H06(2, 4) POS.H06(3, 4)];

    
    plot3(points(:,1),points(:,2),points(:,3));
    
    grid on;
    hold on;
    
end

%% 
% ======================================
% FI CODI SESSIÓ 1
% ======================================

function sessio2

%==========================================================================
% 7./ Call a function named  inverse_kinematics. 
% This function will have the prototype shown in the code: 
% INPUT: The DH parameters and the target 3D position, 
% OUTPUT: A vector of generalised coordinates.
%
%--------------------------------------------------------------------------
    
    %------------------------------------------------------------------%
    % TO DO: 7.1.	Obtain the generalized coordinates for the target 
    % position P_target = [-400 300 -100]'. Verify that the obtained angles    
    % correspond with the real target.         
    %------------------------------------------------------------------%

    % (?)
    
P_target_sessio2 = [-400 300 -100];
P_target = [-400 300 -200];
theta_init = [90, -90, -90, 0, 0, 0];
    
theta_init = pas_radians(theta_init);

DH = [      0     0     0   theta_init(1)   % i=1
          -90     0     0   theta_init(2)   % i=2
            0   650   190   theta_init(3)   % i=3
          -90     0   600   theta_init(4)   % i=4
           90     0     0   theta_init(5)   % i=5
          -90     0     0   theta_init(6)   % i=6
          ];
      
      %theta_target = array de angles theta resultat del calcul de invers
      %kinematics

[theta_target] = inverse_kinematics(DH, P_target );

DH_compr = [      0     0     0   theta_target(1)   % i=1
                -90     0     0   theta_target(2)   % i=2
                  0   650   190   theta_target(3)   % i=3
                -90     0   600   theta_target(4)   % i=4
                 90     0     0   theta_target(5)   % i=5
                -90     0     0   theta_target(6)   % i=6
          ];

TT_comp = get_homogeneous_transforms(DH_compr);

POS_comp = get_3Dpositions(TT_comp);

P_comp = [POS_comp.H06(1, 4) POS_comp.H06(2, 4) POS_comp.H06(3, 4)];

disp(P_target);
disp(P_comp);
    
    % (?)

    
    
    

    

%==========================================================================
% 8./ TODO: Plot the values of Px, Py and Pz in the trajectory from 
% P_init = [500 500 500]? to P_target = [-400 300 -100]' 
% by finding a path in the configuration space using using:
% 	2.1.	Linear interpolation. 
% 	2.2.	Spline interpolation.
% HINT: You can use the function ?linespace?
%--------------------------------------------------------------------------

P_init = [500 500 500];
P_target = [-400 300 -100];

particions = 20;

%Interpolació lineal

pos_x = linspace(P_init(1),P_target(1),particions);
pos_y = linspace(P_init(2),P_target(2),particions);
pos_z = linspace(P_init(3),P_target(3),particions);

matriu_posicions = [pos_x
                    pos_y
                    pos_z];

    hold off;
    
for iteracio = 1:(particions-1)
    plot3(matriu_posicions(1, iteracio), matriu_posicions(2, iteracio), matriu_posicions(3, iteracio), '*');
    grid on;
    hold on;
end
pause;

%Interpolació spline

%Spline interpolation.
    curva = [0,36,64,81,64,36,0]; %X^2 funcion cuadratica a mano
    for i=1:length(curva)
        Ly(i) = Ly(i)+curva(i);
    end

for i=1:length(Lx)
        new_P_target = [Lx(i),Ly(i),Lz(i)];
        [theta_target] = inverse_kinematics(DH, new_P_target)     
        DH = [  0     0     0   theta_target(1);   % i=1
              -90     0     0   theta_target(2);   % i=2
                0   650   190   theta_target(3);   % i=3
              -90     0   600   theta_target(4);   % i=4
               90     0     0   theta_target(5);   % i=5
              -90     0     0   theta_target(6);   % i=6
              ];






%TODO

particions = 20;

pos_x = linspace(P_init(1),P_target(1),particions);
pos_y = linspace(P_init(2),P_target(2),particions);
pos_z = linspace(P_init(3),P_target(3),particions);

cs = spline(pos_x, pos_y, pos_z);

sx = spline(cs, pos_x);
sy = spline(cs, pos_y);
sz = spline(cs, pos_z);

xx = linspace(0, particions, particions+1);


disp xx;

matriu_posicions = [pos_x
                    pos_y
                    pos_z];

    hold off;
xp = ppval(sx,xx);
yp = ppval(sy,xx);
zp = ppval(sz,xx);

matriu_posicions = [xp
                    yp
                    zp];
    
for iteracio = 1:(particions-1)
    plot3(matriu_posicions(1, iteracio), matriu_posicions(2, iteracio), matriu_posicions(3, iteracio), '-*');
    grid on;
    hold on;
end
pause;







    
%==========================================================================
% 9./ TODO: Now Plot the values of theta1, theta2,
% theta3 , theta4, theta5 and theta6 for 2 types of trayectories from 
% P_init = [500 500 500]? to P_target = [-400 300 -100]' in the 3D space
% thorugh:
% 	2.1.	Linear interpolation. 
% 	2.2.	Spline interpolation.
% HINT: You can use the function ?linespace?
%--------------------------------------------------------------------------
 
P_init = [500 500 500];
P_target = [-400 300 -100];

particions = 20;

%Interpolació lineal

pos_x = linspace(P_init(1),P_target(1),particions);
pos_y = linspace(P_init(2),P_target(2),particions);
pos_z = linspace(P_init(3),P_target(3),particions);

matriu_posicions = [pos_x
                    pos_y
                    pos_z];

    hold off;
    colors = ['g o' 'c o' 'r o' 'y o' 'b o' 'o']
    
for iteracio = 1:(particions-1)
    
    P_target = [matriu_posicions(1, iteracio), matriu_posicions(2, iteracio), matriu_posicions(3, iteracio)];
    
    [theta_target] = inverse_kinematics(DH, P_target );
    
    
    
    for iteracio2 = 1:6
        plot(iteracio,theta_target(iteracio2) ,colors(iteracio2));
        hold on;
        
    end
    

end 
pause;


%Interpolació spline
%TODO

particions = 20;

pos_x = linspace(P_init(1),P_target(1),particions);
pos_y = linspace(P_init(2),P_target(2),particions);
pos_z = linspace(P_init(3),P_target(3),particions);

cs = spline(pos_x, pos_y, pos_z);

sx = spline(cs, pos_x);
sy = spline(cs, pos_y);
sz = spline(cs, pos_z);

xx = linspace(0, particions, particions+1);


disp xx;

matriu_posicions = [pos_x
                    pos_y
                    pos_z];

    hold off;
xp = ppval(sx,xx);
yp = ppval(sy,xx);
zp = ppval(sz,xx);

matriu_posicions = [xp
                    yp
                    zp];
    
for iteracio = 1:(particions-1)
    plot3(matriu_posicions(1, iteracio), matriu_posicions(2, iteracio), matriu_posicions(3, iteracio), '- *');
    grid on;
    hold on;
end
pause;




%==========================================================================
% 11./ TODO: Complete the TODO 11: Implement the trajectory from from 
% P_init = [500 500 500]? to P_target = [-400 300 -100]' by using the 
% incremental motion through the Jacobian.
%--------------------------------------------------------------------------

J = get_jacobian(TT)  

% (?)


%======================%
% AUXILIARY FUNCTIONS  %
%======================%

function J = get_jacobian(TT)

%==========================================================================
% 10./ TODO: Implement the function get_jacobian, which returns
% the Jacobian of the system using the homogeneous transforms as input.
%--------------------------------------------------------------------------

    % (?) Matriu jacobiana 6x6. Jv velocitats lineals, JV velocitats
    % angulars. Ambdos de tamany 3x1. Jvi = zi x (E-Pi)
    %                                             ^ 4 columna DH (6,0)
    
    
    

    
    
    


end

%% 
%
% ACKNOWLEDGEMENTS
% ----------------
% Parts of this code were obtained from: 
%   3D Puma Robot Demo
% http://www.mathworks.com/matlabcentral/fileexchange/14932-3d-puma-robot-demo
%   Thanks  Don Riley for his wonderful contribution
%
%============================================================
%%
% This function computes the Inverse Kinematics for the Puma 762 robot
% given X,Y,Z coordinates for a point in the workspace. Note: The IK are
% computed for the origin of Coordinate systems 4,5 & 6.

%%
% This function computes the Inverse Kinematics for the Puma 762 robot
% given X,Y,Z coordinates for a point in the workspace. Note: The IK are
% computed for the origin of Coordinate systems 4,5 & 6.
    %
    % The original call to the function in the Toolbox
    % function [theta1,theta2,theta3,theta4,theta5,theta6] = PumaIK(Px,Py,Pz)
    %
    % The new call to the functiona, adapted to the assignement
    function theta_target = inverse_kinematics(DH, P_target)
    
        % first we get the target point from the DH parameters
        Px = P_target(1);
        Py = P_target(2);
        Pz = P_target(3);
        
%         % now we get the rest of relevant parameters of the puma
%         a2 = 650;
%         a3 = 0;
%         d3 = 190;
%         d4 = 600;
        
        a2 = DH(3,2);
        a3 = DH(4,2);
        d3 = DH(3,3);
        d4 = DH(4,3);
        
        
%         DH = [      0     0     0   theta_init(1)   % i=1
%                   -90     0     0   theta_init(2)   % i=2
%                     0   650   190   theta_init(3)   % i=3
%                   -90     0   600   theta_init(4)   % i=4
%                    90     0     0   theta_init(5)   % i=5
%                   -90     0     0   theta_init(6)   % i=6
%                   ];            
 
 
        % now we start solving the invers kinematics following the
        % formulae in Craig's Chapter 4.7
        theta4 = 0;
        theta5 = 0;
        theta6 = 0;
        sign1 = 1;
        sign3 = 1;
        nogo = 0;
        noplot = 0;
        % Because the sqrt term in theta1 & theta3 can be + or - we run through
        % all possible combinations (i = 4) and take the first combination that
        % satisfies the joint angle constraints.
        while nogo == 0;
            for i = 1:1:4
                if i == 1
                    sign1 = 1;
                    sign3 = 1;
                elseif i == 2
                    sign1 = 1;
                    sign3 = -1;
                elseif i == 3
                    sign1 = -1;
                    sign3 = 1;
                else
                    sign1 = -1;
                    sign3 = -1;
                end
                
                          
 
                rho = sqrt(Px^2+Py^2);
                phi = atan2(Py,Px);
                K = (Px^2+Py^2+Pz^2-a2^2-a3^2-d3^2-d4^2)/(2*a2);
                c4 = cos(theta4);
                s4 = sin(theta4);
                c5 = cos(theta5);
                s5 = sin(theta5);
                c6 = cos(theta6);
                s6 = sin(theta6);
                theta1 = (atan2(Py,Px)-atan2(d3,sign1*sqrt(Px^2+Py^2-d3^2)));
 
                c1 = cos(theta1);
                s1 = sin(theta1);
                theta3 = (atan2(a3,d4)-atan2(K,sign3*sqrt(a3^2+d4^2-K^2)));
 
                c3 = cos(theta3);
                s3 = sin(theta3);
                t23 = atan2((-a3-a2*c3)*Pz-(c1*Px+s1*Py)*(d4-a2*s3),(a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py));
                theta2 = (t23 - theta3);
 
                c2 = cos(theta2);
                s2 = sin(theta2);
                s23 = ((-a3-a2*c3)*Pz+(c1*Px+s1*Py)*(a2*s3-d4))/(Pz^2+(c1*Px+s1*Py)^2);
                c23 = ((a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py))/(Pz^2+(c1*Px+s1*Py)^2);
                r13 = -c1*(c23*c4*s5+s23*c5)-s1*s4*s5;
                r23 = -s1*(c23*c4*s5+s23*c5)+c1*s4*s5;
                r33 = s23*c4*s5 - c23*c5;
                theta4 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);
 
                r11 = c1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)+s1*(s4*c5*c6+c4*s6);
                r21 = s1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)-c1*(s4*c5*c6+c4*s6);
                r31 = -s23*(c4*c5*c6-s4*s6)-c23*s5*c6;
                s5 = -(r13*(c1*c23*c4+s1*s4)+r23*(s1*c23*c4-c1*s4)-r33*(s23*c4));
                c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
                theta5 = atan2(s5,c5);
 
                s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*(s23*s4);
                c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
                theta6 = atan2(s6,c6);
 
                theta1 = theta1*180/pi;
                theta2 = theta2*180/pi;
                theta3 = theta3*180/pi;
                theta4 = theta4*180/pi;
                theta5 = theta5*180/pi;
                theta6 = theta6*180/pi;
                if theta2>=160 && theta2<=180
                    theta2 = -theta2;
                end
 
                if theta1<=160 && theta1>=-160 && (theta2<=20 && theta2>=-200) && theta3<=45 && theta3>=-225 && theta4<=266 && theta4>=-266 && theta5<=100 && theta5>=-100 && theta6<=266 && theta6>=-266
                    nogo = 1;
                    theta3 = theta3+180;
                    break
                end
                if i == 4 && nogo == 0
                    h = errordlg('Point unreachable due to joint angle constraints.','JOINT ERROR');
                    waitfor(h);
                    nogo = 1;
                    noplot = 1;
                    break
                end
            end
        end
        
        
        theta_target = [theta1,theta2,theta3+180,theta4,theta5,theta6]';
    end
end


