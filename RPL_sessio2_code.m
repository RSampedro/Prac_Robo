%================%
% MAIN FUNCTION  %
%================%

function sessio1
    
   
%////////////////////
%    sessio1.m
%////////////////////


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
 
    
    %todos son revolition joint --> en todos tenemos un angulo asociado
    
    %formato: [a , alpha, d, theta] para cada joint
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
    plot_joints(TT);
    

 
%==========================================================================
% 4./ Simulate links as straight lines connecting joints:
%--------------------------------------------------------------------------
 
    % We connect the joints with straight lines to simulate the links.
    simulate_links(TT);    
    title('4 : Show Robot (Conf 1)');
    pause;  % a bit, to watch the plot of the configuration shown.
 
%==========================================================================
% 5./ Repeate for a new configuration:
%--------------------------------------------------------------------------
 
    % The new configuration (configuration shown B).
    thetaB = [45, -60, 30, 0, 0, 0]';
    
    DH = [  0     0     0   thetaB(1)   % i=1
          -90     0     0   thetaB(2)   % i=2
            0   650   190   thetaB(3)   % i=3
          -90     0   600   thetaB(4)   % i=4
           90     0     0   thetaB(5)   % i=5
          -90     0     0   thetaB(6)   % i=6
          ];
        
    TT = get_homogeneous_transforms(DH);
    
    figure,         
    
    plot_joints(TT);
 
    % We connect the joints with straight lines to simulate the links.
    simulate_links(TT);    
    
    title('5 : Show Robot (Conf 2)');
    
    pause;  % a bit, to watch the plot of the configuration shown.
 
 
%==========================================================================
% 6./ Modify the Puma robot by transforming Joint 5 into a prismatic joint
%     and provide the position for the given configurations:
%--------------------------------------------------------------------------
    
    %------------------------------------------------------------------%
    % TO DO: Write new confs. and implement the calls. The new confs.  % 
    %        imply changing JOINT 5 by a prismatic joint and solve the %
    %        forward kinematics for d1=100, d1=300 and d1=-300.        %
    %------------------------------------------------------------------%
    
    for i = [100, 300, -300]
        DH = [  0       0       0       thetaB(1)   % i=1
                -90     0       0       thetaB(2)   % i=2
                0       650     190     thetaB(3)   % i=3
                -90     0       600     thetaB(4)   % i=4
                90      0       i       0           % i=5
                -90     0       0       thetaB(6)   % i=6
            ];

        TT = get_homogeneous_transforms(DH);

        figure,

        plot_joints(TT);

        % We connect the joints with straight lines to simulate the links.
        simulate_links(TT);    

        title('6 : Show Robot Prismatic');
        pause;  % a bit, to watch the plot of the configuration shown.
    end
    

 
    
%end



%================================%
% AUXILIARY FUNCTIONS SESSION 1  %
%================================%


 
%% here we build up a 3D canvas for plotting stuff
function plot_canvas
 
    xlim([-1500 1500]);
    ylim([-1500 1500]);
    zlim([-1500 1500]);
    grid on
    view([400,400,400])
    hold on 
    %------------------------------------------------------------------%
    % TO DO: Initialize the axes, their limits and set the view.       %
    %------------------------------------------------------------------%
 
end
 
 
 
%% here the forward kinematics
function TT = get_homogeneous_transforms(DH)

    %------------------------------------------------------------------%
    % TO DONE: Obtain T(0,6) as composition of the 6 different matrices  %
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
 
    TT.T02 = TT.T01 * TT.T12; %TT01 * TT12
    TT.T03 = TT.T02 * TT.T23; %TT01 * TT12 * TT23
    TT.T04 = TT.T03 * TT.T34; %TT01 * TT12 * TT23 * TT34
    TT.T05 = TT.T04 * TT.T45; %TT01 * TT12 * TT23 * TT34 * TT45
    TT.T06 = TT.T05 * TT.T56; %TT01 * TT12 * TT23 * TT34 * TT45 * TT56
    
end
    
 
 
%% here the tmat function
function T = tmat(alpha, a, d, theta)
 
    % Crea una matriz de transformacion a partir de las configuraciones de
    % un joint
    %------------------------------------------------------------------%
    % TO DO: Implement here the equation 3.6 in Craig's "Introduction  %
    %         to Robotics".                                            %
    %------------------------------------------------------------------%
    %theta = deg2rad(theta);
    %alpha = deg2rad(alpha);
    
    T=[ cosd(theta)  -sind(theta) 0 a
        sind(theta)*cosd(alpha)   cosd(theta)*cosd(alpha)   -sind(alpha) -sind(alpha)*d
        sind(theta)*sind(alpha)   cosd(theta)*sind(alpha)   cosd(alpha)  cosd(alpha)*d
        0   0   0   1];  

end


%% here we plot the joints
function plot_joints(TT)
 
    %------------------------------------------------------------------%
    % TO DO: Plot the joints in the canvas.                            %   
    % HINT 1: You can call 'plot_canvas' to rewrite the axes.          %
    % HINT 2: Use the T's to obtain the 3D positions.                  %
    %------------------------------------------------------------------%
 
    plot_canvas;
    T = TT.T01;
    %[T(1,4) T(2,4) T(3,4)]
    plot3(T(1,4), T(2,4), T(3,4), 'r*');
    
    hold on;
    
    T = T*TT.T12;
    %[T(1,4) T(2,4) T(3,4)]
    plot3(T(1,4),T(2,4), T(3,4), 'bo');
    
    T = T*TT.T23;
    %[T(1,4) T(2,4) T(3,4)]
    plot3(T(1,4), T(2,4), T(3,4), 'g+');
    
    T = T*TT.T34;
    %[T(1,4) T(2,4) T(3,4)]
    plot3(T(1,4), T(2,4), T(3,4), 'kh');
    
    T = T*TT.T45;
    %[T(1,4) T(2,4) T(3,4)]
    plot3(T(1,4), T(2,4), T(3,4), 'mx');
    
    T = T*TT.T56;
    %[T(1,4) T(2,4) T(3,4)]
    plot3(T(1,4), T(2,4), T(3,4), 'cv');
        
end
 
%% here we plot the links
function simulate_links(TT)
 
    %------------------------------------------------------------------%
    % TO DO: Plot the links in the canvas.                             %   
    % HINT 1: You can call 'plot_canvas' to rewrite the axes.          %
    % HINT 2: You can simulate the links by connecting the each pair   % 
    %         of consecutive joints with a straight line.              %                           
    %------------------------------------------------------------------%

    plot_canvas;
    Ti = TT.T01;
    Tf = Ti * TT.T12;
    plot3([Ti(1,4) Tf(1,4)], [Ti(2,4) Tf(2,4)], [Ti(3,4) Tf(3,4)], 'r');
    hold on;
    Ti = Tf;
    Tf = Ti * TT.T23;
    plot3([Ti(1,4) Tf(1,4)], [Ti(2,4) Tf(2,4)], [Ti(3,4) Tf(3,4)], 'b');
    
    Ti = Tf;
    Tf = Ti * TT.T34;
    plot3([Ti(1,4) Tf(1,4)], [Ti(2,4) Tf(2,4)], [Ti(3,4) Tf(3,4)], 'g');
    
    Ti = Tf;
    Tf = Ti * TT.T45;
    plot3([Ti(1,4) Tf(1,4)], [Ti(2,4) Tf(2,4)], [Ti(3,4) Tf(3,4)], 'k');
    
    Ti = Tf;
    Tf = Ti * TT.T56;
    plot3([Ti(1,4) Tf(1,4)], [Ti(2,4) Tf(2,4)], [Ti(3,4) Tf(3,4)], 'm');

    hold off;
 
end

%/////////////
%    END SESSION 1
%/////////////

 
 
%////////////////////
%    sessio2.m
%////////////////////


%==========================================================================
% 7./ Call a function named  inverse_kinematics. 
% This function will have the prototype shown in the code: 
% INPUT: The DH parameters and the target 3D position, 
% OUTPUT: A vector of generalised coordinates.
%
%--------------------------------------------------------------------------
    
    % inverse kinematics: dado un punto final donde queremos llegar, se
    % calcula los movimientos (configuraciones) necesarios para llegar en
    % cada uno de los joints.
    
    %------------------------------------------------------------------%
    % TO DO: 7.1.	Obtain the generalized coordinates for the target 
    % position P_target = [-400 300 -100]'. Verify that the obtained angles    
    % correspond with the real target.         
    %------------------------------------------------------------------%
    close all;

     % (?) 
    % The initial configuration (configuration shown A)
    theta_init = [90, -90, -90, 0, 0, 0]';
 
    % The DH parameters of PUMA 560
    DH = [  0     0     0   theta_init(1)   % i=1
          -90     0     0   theta_init(2)   % i=2
            0   650   190   theta_init(3)   % i=3
          -90     0   600   theta_init(4)   % i=4
           90     0     0   theta_init(5)   % i=5
          -90     0     0   theta_init(6)   % i=6
          ];

    P_target = [-400 300 -100]'; %Para evitar entrar en la singularidad
    %P_target = [-400 300 -200]';
    [theta_target] = inverse_kinematics(DH, P_target)
 
    % The DH parameters of PUMA 560
    DH = [  0     0     0   theta_target(1)   % i=1
          -90     0     0   theta_target(2)   % i=2
            0   650   190   theta_target(3)   % i=3
          -90     0   600   theta_target(4)   % i=4
           90     0     0   theta_target(5)   % i=5
          -90     0     0   theta_target(6)   % i=6
          ];
      
    TT = get_homogeneous_transforms(DH);
    
    plot_canvas;
    
    plot_joints(TT);

        % We connect the joints with straight lines to simulate the links.
    simulate_links(TT);
    
    title('7 : Inverse Kinematics');
    
    pause;
    

     % (?)

     
%%
%==========================================================================
% 8./ TODO: Plot the values of Px, Py and Pz in the trajectory from P_init = 
% [500 500 500]? to P_target = [-400 300 -100]' by finding a path in the 
% configuration space using using:
% 	2.1.	Linear interpolation. 
% 	2.2.	Spline interpolation.
% HINT: You can use the function ?linespace?
%--------------------------------------------------------------------------

    function show_robot(P_tar)
        %close all;

     % (?) 
    % The initial configuration (configuration shown A)
        theta_init = [90, -90, -90, 0, 0, 0]';
 
    % The DH parameters of PUMA 560
        DH = [  0     0     0   theta_init(1)   % i=1
          -90     0     0   theta_init(2)   % i=2
            0   650   190   theta_init(3)   % i=3
          -90     0   600   theta_init(4)   % i=4
           90     0     0   theta_init(5)   % i=5
          -90     0     0   theta_init(6)   % i=6
          ];

    %P_target = [-400 300 -100]'; Para evitar entrar en la singularidad
        %P_target = [-400 300 -200]';
        P_target = P_tar';
        [theta_target] = inverse_kinematics(DH, P_target);
 
    % The DH parameters of PUMA 560
        DH = [  0     0     0   theta_target(1)   % i=1
                -90     0     0   theta_target(2)   % i=2
                0   650   190   theta_target(3)   % i=3
          -90     0   600   theta_target(4)   % i=4
           90     0     0   theta_target(5)   % i=5
          -90     0     0   theta_target(6)   % i=6
          ];
      
        TT = get_homogeneous_transforms(DH);
    
        %plot_canvas;
    
        plot_joints(TT);
            
        % We connect the joints with straight lines to simulate the links.
        simulate_links(TT);
    
    end

%%

    P_init = [500 500 500];
    P_target = [-400 300 -200];
    
    nelem = 30;
    
    %%
    %LINEAR INTERPOLATION
    
    P_vectorX = linspace(P_init(1), P_target(1), nelem);
    P_vectorY = linspace(P_init(2), P_target(2), nelem);
    P_vectorZ = linspace(P_init(3), P_target(3), nelem);

    close all;%?????????
    plot_canvas;%?????????

    for i=1:nelem
        %figure,
        %hold on;
        clf;
        show_robot([P_vectorX(i) P_vectorY(i) P_vectorZ(i)]);
        
        pause (0.05);
    end
    title('8 : Lineal');
    pause;
    
    %%
    %SPLINE INTERPOLATION
    
    P_vectorY = interp1([P_init(1) P_vectorX(round(nelem/2)) P_target(1)],[P_init(2) P_vectorY(round(nelem/2))+norm(P_vectorY(round(nelem/2))-P_init(2)) P_target(2)],P_vectorX,'spline');
    
    close all;%?????????
    plot_canvas;%?????????

    for i=1:nelem
        %figure,
        %hold on;
        clf;
        show_robot([P_vectorX(i) P_vectorY(i) P_vectorZ(i)]);
        
        pause (0.05);
    end
    title('8 : Spline');
    pause;
    
    
%%
%==========================================================================
% 9./ TODO: Now Plot the values of theta1, theta2,
% theta3 , theta4, theta5 and theta6 for 2 types of trayectories from 
% P_init = [500 500 500]? to P_target = [-400 300 -100]' in the 3D space
% thorugh:
% 	2.1.	Linear interpolation. 
% 	2.2.	Spline interpolation.
% HINT: You can use the function ?linespace?
%--------------------------------------------------------------------------
 
%%

    P_init = [500 500 500];
    P_target = [-400 300 -100];
    
    nelem = 30;
    
    %%
    %LINEAR INTERPOLATION
    
    P_vectorX = linspace(P_init(1), P_target(1), nelem);
    P_vectorY = linspace(P_init(2), P_target(2), nelem);
    P_vectorZ = linspace(P_init(3), P_target(3), nelem);

    close all;%?????????
    plot_canvas;%?????????

    for i=1:nelem
        %figure,
        %hold on;
        clf;
        show_robot([P_vectorX(i) P_vectorY(i) P_vectorZ(i)]);
        
        pause (0.05);
    end
    title('9 : Lineal');
    pause;
    
    %%
    %SPLINE INTERPOLATION
    
    P_vectorX = linspace(P_init(1), P_target(1), nelem);
    P_vectorY = linspace(P_init(2), P_target(2), nelem);
    P_vectorZ = linspace(P_init(3), P_target(3), nelem);
    
    P_vectorY = interp1([P_init(1) P_vectorX(round(nelem/2)) P_target(1)],[P_init(2) P_vectorY(round(nelem/2))+norm(P_vectorY(round(nelem/2))-P_init(2)) P_target(2)],P_vectorX,'spline');
    
    close all;%?????????
    plot_canvas;%?????????

    for i=1:nelem
        %figure,
        %hold on;
        clf;
        show_robot([P_vectorX(i) P_vectorY(i) P_vectorZ(i)]);
        
        pause (0.05);
    end
    title('9 : Spline');
    pause;

    
%%
%==========================================================================
% 11./ TODO: Complete the TODO 11: Implement the trajectory from from 
% P_init = [500 500 500]? to P_target = [-400 300 -100]' by using the 
% incremental motion through the Jacobian.
%--------------------------------------------------------------------------


%%

    %Target and starting points
    P_init = [500 500 500];
    P_target = [-400 300 -100];
    
    %Amount of steps
    nelem = 30;
    
    %Aux
    P_vectorX = linspace(P_init(1), P_target(1), nelem);
    P_vectorY = linspace(P_init(2), P_target(2), nelem);
    P_vectorZ = linspace(P_init(3), P_target(3), nelem);
    P = [P_vectorX ; P_vectorY ; P_vectorZ];
    
    %Aux values
    error = 20;
    step = 10;
    dif = inf;
    
    %Get DeltaX
    delta_x = (P(:,2) - P(:,1)) * step;
    delta_x = [delta_x; 0; 0; 0];

    [theta] = inverse_kinematics(DH, P_init);
    DH = [DH(:,1:3) theta];
    
    close all;
    plot_canvas;
    while dif > error
        clf;
        
        %Show current robot
        TT = get_homogeneous_transforms(DH);
        plot_joints(TT);
        simulate_links(TT);
        
        %Calculate next robot
        dif = (norm(P_target - TT.T06(1:3,4)'));
        
        J = get_jacobian(TT);
        
        %J \ delxa_x = inv(J) * delta_x
        theta_aux = J \ delta_x;
        theta = theta + theta_aux;
        
        DH = [DH(:,1:3) theta];
        
        pause(0.01);
        
    end
    title('11 : Jacobian');
%{
-----------
Com calculem la trajectòria a partir de la matriu jacoviana? De la següent
manera:

A partir de la jacoviana calculem la jacoviana inversa i la multipliquem
per delta de x

delta de x serà un vector en la direcció ..

Delta(theta) = J^{-1} · Delta(x)

Delta(theta)_1 = Delta(theta)_0 + Delta(theta)

%}





%%
%===============================%
% AUXILIARY FUNCTIONS SESSION 2 %
%===============================%


function J = get_jacobian(TT)

%==========================================================================
% 10./ TODO: Implement the function get_jacobian, which returns
% the Jacobian of the system using the homogeneous transforms as input.
%--------------------------------------------------------------------------
%{    
   JACOVIANA
    
    tots els vectors respecte de la base
    matriu jacoviana 6*6
    S_v son vectors 3*1
    S_w son vectors 3*1
    
    Jv(i) = Zi x (E-Pi)   x=cross
    
    Jv1 cross(
    
    E = 4a columna de la matriu T06
    Pi = posició del frame i = 4a col de la matriu TT.Ti0
    punt origen referència 1 : 000


%}
    Jv = zeros(3,6);
    Jw = zeros(3,6);
    
    end_eff = TT.T06(1:3,4); %coordenadas del end effector (tool)
    
    z = [TT.T01(1:3,3), TT.T02( 1:3,3), TT.T03(1:3,3), TT.T04(1:3,3), TT.T05(1:3,3), TT.T06(1:3,3)];
    
    org_coord = [TT.T01(1:3,4), TT.T02(1:3,4), TT.T03(1:3,4), TT.T04(1:3,4), TT.T05(1:3,4), TT.T06(1:3,4)];
    
    for k = 1:6 
        Jv(:, k) = cross(z(:, k), end_eff - org_coord(:, k))'; 
        Jw(:, k) = z(:, k);
    end
 
    J = vertcat(Jv, Jw);
    
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

%/////////////
%    END SESSION 2
%/////////////

    end
    
    


    
    


