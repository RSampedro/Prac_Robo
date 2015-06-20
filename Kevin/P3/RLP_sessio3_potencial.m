    %%
%Example of control function that moves the robot in a square path
% Acknowledgements to K.-Y. Daisy Fan
% http://www.cs.cornell.edu/courses/cs1112/2010fa/Simulator/simulatorTool.html
%%

function driveRandom(serPort)
    init = getState(serPort);
    plot(init(1),init(2),'r*');
    global goal;
    global goal_reached;
	global grid;
	global grid_goal;
    goal_reached = 0;
    goal = ginput(1);
    
    plot(goal(1),goal(2),'b*');
    crearGrid(serPort,init,goal) 
    while goal_reached == 0
        [i_pos,j_pos]=buscarPosicion(serPort);
        [x,y]=buscarSiguientePaso(serPort,i_pos,j_pos)
        moveToGoal(serPort,[x,y])
		
		if euclidean([x y], grid_goal) <= 1
			goal_reached=1;
		end
       
    end
  
function crearGrid(serPort,init,goal)
    [start walls lines beacs vwalls] = getMap(serPort);
    global wx;
    global wy;
    global grid;
    global pass;
    global record;
    global grid_goal;
    global grid_init;
    record = {};
    wx = abs(walls(1,1)-walls(1,2))
    wy = abs(walls(2,3)-walls(2,4))
   
    
    % Tamaño robot = tamaño casilla grid
    pass = 0.32;
    % Crear grid 25 x 25 
    grid = zeros((wx/pass)+1,(wy/pass)+1,5); 
    for i=1:(wy/pass)+1
       for j=1:(wx/pass)+1
           % Asignar coordenadas de cada casilla
           grid(i,j,:)=[walls(1,1)+(j-1)*pass...
                        walls(1,1)+(j)*pass...
                        walls(2,3)-(i-1)*pass...
                        walls(2,3)-(i)*pass 0];
            if (grid(i,j,1)<=goal(1) && grid(i,j,2)>goal(1) && grid(i,j,3)>=goal(2) && grid(i,j,4)<goal(2)) 
                
               grid(j,i,5)=0.0;
               grid_goal = [i j];

           end
          if (grid(i,j,1)<=init(1) && grid(i,j,2)>init(1)  && grid(i,j,3)>=init(2) && grid(i,j,4)<init(2)) 

               grid_init = [i j];

           end
           
       end
    end

    %Asignar distancias a cada punto.
    for i=1:(wy/pass)+1
       for j=1:(wx/pass)+1
          n = norm([i j]-grid_goal);
          grid(j,i,5) = n;
          %ponemos maximo coste a los obstaculos cuadrados.
       end
    end
    global max_cost;
    max_cost = max(max(grid(:,:,5)));

    %preparar obstaculos
    if (size(walls,1)>4)
        
        for w = 5:4:size(walls,1)
            xx_walls = [min(walls(w:w+3,1)) max(walls(w:w+3,1))];
            yy_walls = [min(walls(w:w+3,2)) max(walls(w:w+3,2))];
            x1 = 0;
            x2 = 0;
            y1 = 0;
            y2 = 0;
            
            for i=1:(wx/pass)+1
                for j=1:(wy/pass)+1
                    % Si el robot esta dentro de una casilla, asignar la casilla
                    % como aspirada grid(i,j,2)>xx_walls(2) && grid(i,j,3)>=yy_walls(1) && grid(i,j,4)<yy_walls(2)

                    if (grid(i,j,1)<=xx_walls(2) && grid(i,j,2)>xx_walls(1) && x1==0)
                        x1 = j;
                        
                    elseif (grid(i,j,2)>xx_walls(1) && x2==0 && x1~=0)
                        x2 = j;
                        
                    end
                    if (grid(i,j,3)<=yy_walls(2) && grid(i,j,4)>yy_walls(1) && y1==0)
                        y1 = i;
                    elseif (grid(i,j,4)<yy_walls(1) && y2==0 && y1~=0)
                        y2 = i;
                    end
                end
            end

            
            x11 = x1-2;
            x22 = x2+2;
            y11 = y1-2;
            y22 = y2+2;
            rx = x11:x22;
            ry = y11:y22;
            for x = 1:size(rx,2)
                for y = 1:size(ry,2)
                    grid(rx(x),ry(y),5)= grid(rx(x),ry(y),5)+max_cost/4;
                end
            end
            x11 = x1-1;
            x22 = x2+1;
            y11 = y1-1;
            y22 = y2+1;
            rx = x11:x22;
            ry = y11:y22;
            for x = 1:size(rx,2)
                for y = 1:size(ry,2)
                    grid(rx(x),ry(y),5)= grid(rx(x),ry(y),5)+max_cost/3;
                end
            end
            
            rx = x1:x2;
            ry = y1:y2;
            for x = 1:size(rx,2)
                for y = 1:size(ry,2)
                    grid(rx(x),ry(y),5)= max_cost+max_cost/2;
                end
            end
           
        end
        
    end
    figure(2);
    surf(grid(:,:,5));
    hold on;
    plot3(grid_init(2),grid_init(1),grid(grid_init(1),grid_init(2),5),'r*');

   
 
    
function [i_pos,j_pos]=buscarPosicion(serPort)
    global wx;    global wy;    global grid;    global pass;
    
    state =getState(serPort);
    x = state(1);
    y = state(2);
    for i=1:(wy/pass)
       for j=1:(wx/pass)
           % Si el robot esta dentro de una casilla de la malla, coger 
           % la posicion
           if grid(i,j,1)<=x && grid(i,j,2)>x && grid(i,j,3)>=y && grid(i,j,4)<y
               % Asignar 0  la casilla recorrida 
               i_pos = i;
               j_pos = j;
           end
       end
    end

function [x,y]=buscarSiguientePaso(serPort,i_pos,j_pos)
    global grid;    global pass;    global goal_reached; global max_cost;
    
    if grid(i_pos,j_pos,5)==0
        goal_reached = 1;
    end

    i_bordemin = i_pos-1;
    i_bordemax = i_pos+1;
    if (i_bordemin == 0)
        i_bordemin = i_pos;
    elseif (i_bordemax ==size(grid,1)+1)
         i_bordemax = i_pos;
    end
    j_bordemin = j_pos-1;
    j_bordemax = j_pos+1;
    if (j_bordemin == 0)
        j_bordemin = j_pos;
    elseif (j_bordemin ==size(grid,1)+1)
         j_bordemax = j_pos;
    end
    range_i = i_bordemax:-1:i_bordemin;
    range_j = j_bordemax:-1:j_bordemin;
    submatrix = grid(range_i,range_j,5);
    submatrix(2,2) = max_cost;
    [val x11] = min(submatrix);
    [val2 x22] = min(val);
    
    x1 = range_i(x22);
    
    y1 = range_j(x11(x22));
    plot3(x1,y1,grid(y1,x1,5),'r*');figure(2);
    x = grid(y1,x1,1)+pass/2;
    y = grid(y1,x1,3)+pass/2;
    
    
    
 
function angle = getAngle(serPort,goal)
    % Variables posicion actual Robot
    state=getState(serPort);
    x=state(1); y=state(2); angulo_robot=state(3);

    % Variables posicion final
    xGoal=goal(1); yGoal=goal(2);

    % Variables posicion goal respecto el robot
    difX = xGoal-x; difY = yGoal-y;

    % Calculo distancia angular
    angle = atan2(difY,difX)*180/pi - angulo_robot*180/pi;
    
function moveToGoal(serPort,goal)
    vAngular = 0.2
    % Obtener angulo de giro
    angulo_giro = getAngle(serPort,goal);
    angulo_giroR = abs(angulo_giro*pi/180);

    % Girar robot sobre si mismo
    if angulo_giroR > 0.025
        if angulo_giroR < vAngular
            turnAngle(serPort,0.025,angulo_giro);
        else
            turnAngle(serPort,vAngular,angulo_giro);
        end
    end

    % Mover el robot hacia el goal
    SetDriveWheelsCreate(serPort, .5, .5);
	
function distance = euclidean(hpoint,apoint)
    hx = hpoint(1); hy = hpoint(2);
    x = apoint(1); y = apoint(2);
    distance = sqrt((hx-x)^2 + (hy-y)^2);

%%%%%%%%%%%%%%%%%%%%%%%%%
function driveForwardUntilWall(serPort)
% Robot drives forward until it bumps a wall.
% serPort is the serial port number (for controlling the actual robot).



[BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...
    BumpsWheelDropsSensorsRoomba(serPort);
while ~BumpRight && ~BumpLeft && ~BumpFront 
    SetDriveWheelsCreate(serPort, .2,.2)
   
    
    
    pause(.1)
    [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...
        BumpsWheelDropsSensorsRoomba(serPort);
end
%beep
%StopCreate(serPort)
%Signal()


    
    
    
    


%%%%%%%%%%%%%%%%%%%%%%%%
function StopCreate(serPort)
% Stop the robot
% serPort is the serial port number (for controlling the actual robot).
SetDriveWheelsCreate(serPort, 0,0)

%%%%%%%%%%%%%%%%%%%%%%%%
function Signal()
% Make signal sound (4 beeps)
n= 4;
for k=1:n
    beep
    pause(1)
end


function squarePath(serPort)
% Robot moves along a path that forms a square
% serPort is the serial port number (for controlling the actual robot).

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k= 1:4
  % for each "leg" of the tour, move forward 2m and then turn 90degrees  

  travelDist(serPort, .9, 2)  % Move .3m/s for +2m
                              %   Speed in [.025,.5] meters/s
                              %   Positive distance means move forward
                              %   Negative distance means move backward
                          
  turnAngle(serPort, .9, 90)  % Turn at .2rad/s for +90 degrees
                              %   Speed in [.025,.2] radians/second
                              %   Positive angle means counter-clockwise
                              %   Negative angle means clockwise

end



%%
%Example of control function that moves the robot in a square path
% Acknowledgements to K.-Y. Daisy Fan
% http://www.cs.cornell.edu/courses/cs1112/2010fa/Simulator/simulatorTool.html
%%
