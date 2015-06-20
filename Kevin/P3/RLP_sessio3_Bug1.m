%%
%Example of control function that moves the robot in a square path
% Acknowledgements to K.-Y. Daisy Fan
% http://www.cs.cornell.edu/courses/cs1112/2010fa/Simulator/simulatorTool.html
%%

function bug1(serPort)
    goal = ginput(1);
    goal_reached = 0;
    vAngular = 0.2;
    obj = 0;
    state = getState(serPort);
    x = state(1); y = state(2);
    plot(state(1),state(2), 'r*');
    plot(goal(1),goal(2), 'b*');
    while 1
        % Go to goal until arrive or hit with object
        while goal_reached==0 && obj==0
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
            ax = x; ay = y;
            % Mover el robot hacia el goal
            SetDriveWheelsCreate(serPort, .2, .2);
            
            state = getState(serPort);
            x=state(1); y=state(2);
            plot([ax x],[ay y],'g-');
            % If q_goal is reached
            if euclidean(goal, [x y]) < 0.1
                goal_reached=1;
                beep
            % If hit with object
            else
                [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...
                    BumpsWheelDropsSensorsRoomba(serPort);
                if BumpRight || BumpLeft || BumpFront
                    obj = 1;
                    beep
					plot(x,y,'mx');
                end
            end
        end
        obj = 0;
        % If goal is reached then exit
        if goal_reached ==1
            break
        end
        
        % Boundary object
        hPoint = [x y];
        lPoint = [100 100];
        hitfound = 0;
        vuelta = 0;
        rotate = randi(2);
        if rotate == 1
            rotate = 50;
            girR = 0.02;
            girL = 0.2;
        else
            rotate = -50;
            girR = 0.2;
            girL = 0.02;
        end
		
        turnAngle(serPort,.2,rotate);
        while goal_reached==0 && hitfound==0
            ax = x; ay = y;
            SetDriveWheelsCreate(serPort, girR, girL);
            
            % Save L point
            state = getState(serPort);
            x=state(1); y=state(2);
            plot([ax x],[ay y],'m-');
            if euclidean(goal, [x y]) < euclidean(goal, lPoint)
                lPoint = [x y];
            end
            % If q_goal is reached
            if euclidean(goal, [x y]) < 0.1
                goal_reached=1;
                beep
            else
                % If arrive to hit point
                if vuelta
                    if euclidean(hPoint, [x y]) < 0.1
                        hitfound = 1;
                        vuelta = 0;
                        plot(x,y,'cx');
						plot(lPoint(1),lPoint(2),'ko');
                    end
                elseif euclidean(lPoint, [x y]) > 0.15
                    vuelta = 1;
                end
                
                % If hit with object rotate
                if hitfound==0
                    [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...
                    BumpsWheelDropsSensorsRoomba(serPort);
                    if BumpRight || BumpLeft || BumpFront
                        turnAngle(serPort,.2,rotate);
                        beep
                    end
                end
            end
        end
        
        % Si el punto mas cercano al goal es el hit point
        % No se puede llegar al goal
        if euclidean(hPoint, lPoint) < 0.1
            fprintf('Goal no alcanzable');
            break
        end
        
        % Go to L point
        lfound = 0;
        vuelta = 0;
        while lfound==0
            ax = x; ay = y;
            SetDriveWheelsCreate(serPort, girR, girL);
            
            state = getState(serPort);
            x=state(1); y=state(2);
            plot([ax x],[ay y],'g-');
            % If arrive to L point
            if euclidean(lPoint, [x y]) < 0.1
                lfound = 1;
                plot(x,y,'go');
            end

            % If hit with object rotate
            if lfound==0
                [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...
                BumpsWheelDropsSensorsRoomba(serPort);
                if BumpRight || BumpLeft || BumpFront
                    turnAngle(serPort,.2,rotate);
                    beep
                end
            end
        end
        
    end

% Funcion que calcula la distancia euclidiana entre dos puntos
function distance = euclidean(hpoint,apoint)
    hx = hpoint(1); hy = hpoint(2);
    x = apoint(1); y = apoint(2);
    distance = sqrt((hx-x)^2 + (hy-y)^2);
  
% Funcion que calcula el angulo de giro desde el punto actual hasta un goal  
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

function driveRandom(serPort)
while 1
    r = randi(2)
    if r==1
        % Izquierda
        driveForwardUntilWall(serPort);
        turnAngle(serPort,.2,90);
        SetDriveWheelsCreate(serPort, .2, .2);
    else
        % Derecha
        driveForwardUntilWall(serPort);
        turnAngle(serPort,.2,-90);
        pause(.1)
        SetDriveWheelsCreate(serPort, .2, .2);
    end
end 
    
%%%%%%%%%%%%%%%%%%%%%%%%%
function driveForwardUntilWall(serPort)
% Robot drives forward until it bumps a wall.
% serPort is the serial port number (for controlling the actual robot).

[BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...
    BumpsWheelDropsSensorsRoomba(serPort);
while ~BumpRight && ~BumpLeft && ~BumpFront 
    SetDriveWheelsCreate(serPort, .5,.5)
    pause(.1)
    [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...
        BumpsWheelDropsSensorsRoomba(serPort);
end
beep
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
