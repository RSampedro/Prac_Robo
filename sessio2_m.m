%////////////////////
%    sessio2.m
%////////////////////



%================%
% MAIN FUNCTION  %
%================%

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
    theta_init = [90, -90, -90, 0, 0, 0];
    P_target = [-400 300 -200];
    mostra_Robot(P_target);
    title('7 : Cinematica Inversa');
    pause;
    % (?)

    %==========================================================================
    % 8./ TODO: Plot the values of Px, Py and Pz in the trajectory from 
    % P_init = [500 500 500]? to P_target = [-400 300 -100]' 
    % by finding a path in the configuration space using using:
    % 	2.1.	Linear interpolation. 
    % 	2.2.	Spline interpolation.
    % HINT: You can use the function ?linespace?
    %--------------------------------------------------------------------------
    
    close all;
    P_init = [500 500 500];
    P_target = [-400 300 -200];

    frames = 20;

    %Interpolació lineal

    pos_x = linspace(P_init(1),P_target(1),frames);
    pos_y = linspace(P_init(2),P_target(2),frames);
    pos_z = linspace(P_init(3),P_target(3),frames);

    
    for i=1:frames
        clf;
        mostra_Robot([pos_x(i) pos_y(i) pos_z(i)]);
        pause(0.3);
    end
    title('8_1: Interpolació lineal');
    
    hold off;
    pause;
    
    %==========================================================================
    
    %Interpolació spline 
    pos_y = interp1([P_init(1) pos_x(round(frames/2)) P_target(1)],[P_init(2) pos_y(round(frames/2))+norm(pos_y(round(frames/2))-P_init(2)) P_target(2)],pos_x,'spline');

    close all;
    
    %Visualitzem
    for i=1:frames
        clf;
        mostra_Robot([pos_x(i) pos_y(i) pos_z(i)]);
        pause(0.3);
        title('8_1: Interpolació spline');
    end

    hold off;
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

    frames = 20;

    %Interpolació lineal

    pos_x = linspace(P_init(1),P_target(1),frames);
    pos_y = linspace(P_init(2),P_target(2),frames);
    pos_z = linspace(P_init(3),P_target(3),frames);
    pos = [pos_x; pos_y; pos_z];
    
    DH = [      0     0     0   theta_init(1)   % i=1
              -90     0     0   theta_init(2)   % i=2
                0   650   190   theta_init(3)   % i=3
              -90     0   600   theta_init(4)   % i=4
               90     0     0   theta_init(5)   % i=5
              -90     0     0   theta_init(6)   % i=6
              ];

    hold off;
    colors = ['r' 'g' 'b' 'c' 'm' 'y'];

    hold on;
    thetas = zeros(6,frames);
    clf;
    for i=1:frames
        [theta_target] = inverse_kinematics(DH, pos(:,i));
        thetas(:,i) = theta_target;
    end

    hold on;
    grid on;
    t(1) = plot(thetas(1, :), colors(1));
    t(2) = plot(thetas(2, :), colors(2));
    t(3) = plot(thetas(3, :), colors(3));
    t(4) = plot(thetas(4, :), colors(4));
    t(5) = plot(thetas(5, :), colors(5));
    t(6) = plot(thetas(6, :), colors(6));
    title('9_1: Interpolació Lineal');
    hold off;
    pause;


    %Interpolació spline


    pos_x = linspace(P_init(1),P_target(1),frames);
    pos_y = interp1([P_init(1) pos_x(uint8(frames/2)) P_target(1)],[P_init(2) pos_y(uint8(frames/2))+norm(pos_y(uint8(frames/2))-P_init(2)) P_target(2)],pos_x,'spline');
    pos_z = linspace(P_init(3),P_target(3),frames);
    pos2 = [pos_x; pos_y; pos_z];

    for i=1:frames
            [theta_target] = inverse_kinematics(DH, pos2(:,i));
            thetas(:,i) = theta_target;
    end


    hold off;

    % Plot de las variaciones de theta
    t = zeros(6,1);
    clf;
    hold on;
    grid on;
    t(1) = plot(thetas(1, :), colors(1));
    t(2) = plot(thetas(2, :), colors(2));
    t(3) = plot(thetas(3, :), colors(3));
    t(4) = plot(thetas(4, :), colors(4));
    t(5) = plot(thetas(5, :), colors(5));
    t(6) = plot(thetas(6, :), colors(6));
    title('9_1: Interpolació Spline');
    hold off;
    pause;


    %==========================================================================
    % 11./ TODO: Complete the TODO 11: Implement the trajectory from from 
    % P_init = [500 500 500]? to P_target = [-400 300 -100]' by using the 
    % incremental motion through the Jacobian.
    %--------------------------------------------------------------------------
    
    [theta_init]= inverse_kinematics(DH,P_init);
    DH = [DH(:,1:3) theta_init];
    TT = get_homogeneous_transforms(DH);
    TT = get_3Dpositions(TT);
    
    J = get_jacobian(TT)  
    
    P_target= [-400 300 -100]';
    P_init=[500 500 500]';
    
    error = 20;
    aux = 10;
    difference = inf;

    increments = (pos(:,2) - pos(:,1))*aux;
    increments = [increments;0;0;0];

    
    


    hold on;

    theta = theta_init;
    while difference > error
        
        TT = get_homogeneous_transforms(DH);
        TT = get_3Dpositions(TT);
        clf;
        plot_canvas;
        plot_joints(TT);
        simulate_links(TT)
        
        %difference = sqrt((P_target(1)-TT.T06(1,4))^2+(P_target(2)-TT.T06(2,4))^2+(P_target(3)-TT.T06(3,4))^2)
        difference = (norm(P_target - TT.T06(1:3,4)))
        
        J = get_jacobian(TT);
        
        theta_aux = J\increments;
        theta = theta + theta_aux;
        
        DH = [DH(:,1:3) theta];
        
        pause(0.3);
        
    end

    hold off;
    
end
% % (?)  

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
    % angulars. Ambdos de tamany 3x6. Jvi = zi x (E-Pi)
    %                                             ^ 4 columna DH (6,0)
    
    JV=zeros(3,6);
    JW=zeros(3,6);
    

    %6a columna (T06)
    JV(:,6) = cross(TT.T06(1:3,3),TT.T06(1:3,4)-TT.T06(1:3,4));
    JW(:,6) = TT.T06(1:3,3);
    %5a columna (T05)
    JV(:,5) = cross(TT.T05(1:3,3),TT.T06(1:3,4)-TT.T05(1:3,4));
    JW(:,5) = TT.T05(1:3,3);
    %4a columna (T04)
    JV(:,4) = cross(TT.T04(1:3,3),TT.T06(1:3,4)-TT.T04(1:3,4));
    JW(:,4) = TT.T04(1:3,3);
    %3era columna (T03)
    JV(:,3) = cross(TT.T03(1:3,3),TT.T06(1:3,4)-TT.T03(1:3,4));
    JW(:,3) = TT.T03(1:3,3);
    %2ona coluna (T02)
    JV(:,2) = cross(TT.T02(1:3,3),TT.T06(1:3,4)-TT.T02(1:3,4));
    JW(:,2) = TT.T02(1:3,3);
    %1era columna (T01)
    JV(:,1) = cross(TT.T01(1:3,3),TT.T06(1:3,4)-TT.T01(1:3,4));
    JW(:,1) = TT.T01(1:3,3);  
    
    J = [
         JV
         JW
        ];
    
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
                nogo = 1; %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         
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

%% Funcions meves
%======================%
% AUXILIARY FUNCTIONS  %
%======================%

function mostra_Robot(position)

    P_target = position;
    theta_init = [90, -90, -90, 0, 0, 0];

    %theta_init = pas_radians(theta_init);

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

    P_comp = [POS_comp.T06(1, 4) POS_comp.T06(2, 4) POS_comp.T06(3, 4)];

    % We connect the joints with straight lines to simulate the links.
    plot_canvas;
    plot_joints(TT_comp);
    simulate_links(TT_comp);
    disp(P_target);
    disp(P_comp);
end



function TT = get_3Dpositions(TT)

    TT.T01 = TT.T01;
    TT.T02 = TT.T01*TT.T12;
    TT.T03 = TT.T02*TT.T23;
    TT.T04 = TT.T03*TT.T34;
    TT.T05 = TT.T04*TT.T45;
    TT.T06 = TT.T05*TT.T56;
    
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
   
    TT.T01 = tmat(DH(1,1), DH(1,2), DH(1,3), DH(1,4));
    TT.T12 = tmat(DH(2,1), DH(2,2), DH(2,3), DH(2,4));
    TT.T23 = tmat(DH(3,1), DH(3,2), DH(3,3), DH(3,4));
    TT.T34 = tmat(DH(4,1), DH(4,2), DH(4,3), DH(4,4));
    TT.T45 = tmat(DH(5,1), DH(5,2), DH(5,3), DH(5,4));
    TT.T56 = tmat(DH(6,1), DH(6,2), DH(6,3), DH(6,4));
    
end
    
 
 
%% here the tmat function
function T = tmat(alpha, a, d, theta)
 
    %------------------------------------------------------------------%
    % TO DO: Implement here the equation 3.6 in Craig's "Introduction  %
    %         to Robotics".                                            %
    %------------------------------------------------------------------%
    
    T = [cosd(theta) -sind(theta) 0 a; 
        sind(theta)*cosd(alpha) cosd(theta)*cosd(alpha) -sind(alpha) -sind(alpha)*d ; 
        sind(theta)*sind(alpha) cosd(theta)*sind(alpha) cosd(alpha) cosd(alpha)*d;
        0 0 0 1];
    
end
 
 

%% here we plot the joints
function plot_joints(TT)
 
    %------------------------------------------------------------------%
    % TO DO: Plot the joints in the canvas.                            %   
    % HINT 1: You can call 'plot_canvas' to rewrite the axes.         %
    % HINT 2: Use the T's to obtain the 3D positions.                  %
    %------------------------------------------------------------------%
    TT = get_3Dpositions(TT);
    TT.T06
    plot_canvas;
    
    hold on;
    grid on;
    plot3(TT.T01(1,4),TT.T01(2,4),TT.T01(3,4),'o')
    plot3(TT.T02(1,4),TT.T02(2,4),TT.T02(3,4),'+')
    plot3(TT.T03(1,4),TT.T03(2,4),TT.T03(3,4),'+')
    plot3(TT.T04(1,4),TT.T04(2,4),TT.T04(3,4),'o')
    plot3(TT.T05(1,4),TT.T05(2,4),TT.T05(3,4),'+')
    plot3(TT.T06(1,4),TT.T06(2,4),TT.T06(3,4),'+')
    hold off

end
 
 
 
%% here we plot the links
function simulate_links(TT)
 
    %------------------------------------------------------------------%
    % TO DO: Plot the links in the canvas.                             %   
    % HINT 1: You can call 'plot_canvas' to rewrite the axes.          %
    % HINT 2: You can simulate the links by connecting the each pair   % 
    %         of consecutive joints with a straight line.              %                           %
    %------------------------------------------------------------------%
    TT = get_3Dpositions(TT);
    
    hold on
    grid on
    plot3([TT.T01(1,4), TT.T02(1,4)],[TT.T01(2,4), TT.T02(2,4)],[TT.T01(3,4), TT.T02(3,4)])
    plot3([TT.T02(1,4), TT.T03(1,4)],[TT.T02(2,4), TT.T03(2,4)],[TT.T02(3,4), TT.T03(3,4)])
    plot3([TT.T03(1,4), TT.T04(1,4)],[TT.T03(2,4), TT.T04(2,4)],[TT.T03(3,4), TT.T04(3,4)])
    plot3([TT.T04(1,4), TT.T05(1,4)],[TT.T04(2,4), TT.T05(2,4)],[TT.T04(3,4), TT.T05(3,4)])
    plot3([TT.T05(1,4), TT.T06(1,4)],[TT.T05(2,4), TT.T06(2,4)],[TT.T05(3,4), TT.T06(3,4)])
    
    hold off

    
end

