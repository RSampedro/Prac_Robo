%////////////////////
%    sessio1.m
%////////////////////



%================%
% MAIN FUNCTION  %
%================%


% ====================================== %
% INICI CODI SESSI� 1                    %
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
    plot_joints(TT);
    

 
%==========================================================================
% 4./ Simulate links as straight lines connecting joints:
%--------------------------------------------------------------------------
 
    % We connect the joints with straight lines to simulate the links.
    simulate_links(TT);    
    pause;  % a bit, to watch the plot of the configuration shown.
 
    
    
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
    clf;
    plot_canvas;
    % Joints are visualized on the canvas.
    plot_joints(TT2);
    
%==========================================================================
% 5.4./ Simulate links as straight lines connecting joints:
%--------------------------------------------------------------------------
 
    % We connect the joints with straight lines to simulate the links.
    simulate_links(TT2);    
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
    
    
      DH3 = [      0     0     0   thetaB(1)   % i=1
                 -90     0     0   thetaB(2)   % i=2
                   0   650   190   thetaB(3)   % i=3
                 -90     0   600   thetaB(4)   % i=4
                   0     0     0   thetaB(5)   % i=5
                 -90     0   100   thetaB(6)   % i=6
          ];
           
      
      
      TT3 = get_homogeneous_transforms(DH3);
      clf;
      plot_canvas;
      plot_joints(TT3);
      simulate_links(TT3);
      pause;
       
    
      DH4 = [      0     0     0   thetaB(1)   % i=1
                 -90     0     0   thetaB(2)   % i=2
                   0   650   190   thetaB(3)   % i=3
                 -90     0   600   thetaB(4)   % i=4
                   0     0     0   thetaB(5)   % i=5
                 -90     0   300   thetaB(6)   % i=6
          ];
           
      TT4 = get_homogeneous_transforms(DH4);
      clf;
      plot_canvas;
      plot_joints(TT4);
      simulate_links(TT4); 
      pause;
          
          
      DH5 = [     0     0     0   thetaB(1)   % i=1
                -90     0     0   thetaB(2)   % i=2
                  0   650   190   thetaB(3)   % i=3
                -90     0   600   thetaB(4)   % i=4
                  0     0     0   thetaB(5)   % i=5
                -90     0  -300   thetaB(6)   % i=6
             ];
        
    
  
      TT5 = get_homogeneous_transforms(DH5);
      clf;
      plot_canvas;
      plot_joints(TT5);
      simulate_links(TT5);
      pause;
      
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
% FI CODI SESSI� 1
% ======================================