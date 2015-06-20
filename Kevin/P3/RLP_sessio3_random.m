%Example of control function that moves the robot in a square path
% Acknowledgements to K.-Y. Daisy Fan
% http://www.cs.cornell.edu/courses/cs1112/2010fa/Simulator/simulatorTool.html
%%

function driveRandom(serPort)
    crearGrid(serPort) 
    while 1
        r = randi(2);
       
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
  
function crearGrid(serPort)
    [start walls lines beacs vwalls] = getMap(serPort);
    global wx;
    global wy;
    global grid;
    global pass;
    global record;
    
    record = {};
    wx = abs(walls(1,1)-walls(1,2));
    wy = abs(walls(2,3)-walls(2,4));
    total_area = wx*wy
    
    % Tamaño robot = tamaño casilla grid
    pass = 0.32;
    grid_Obstaculos(serPort)    
    % Crear grid 25 x 25 
    grid = zeros(wx/pass,wy/pass,4); 
    for i=1:(wy/pass)
       for j=1:(wx/pass)
           % Asignar coordenadas de cada casilla
           grid(i,j,:)=[walls(1,1)+(j-1)*pass...
                        walls(1,1)+(j)*pass...
                        walls(2,3)-(i-1)*pass...
                        walls(2,3)-(i)*pass];
       end
    end
    
function grid_Obstaculos(serPort)
    global pass;
    [start walls lines beacs vwalls] = getMap(serPort);
    tam = size(walls);
    
    global gridObstaculos;
    gridObstaculos = 0;
    for i=5:4:tam(1)
            y = abs(walls(i,2)-walls(i,4))
            x = abs(walls(i+1,1)-walls(i+1,3))
            x = x/pass;
            y = y/pass;
            gridObstaculos = gridObstaculos + x*y;
    end

function calcularPorcentaje(serPort)
    global wx;    global wy;    global grid;    global pass;
    global record; global gridObstaculos; 
    state =getState(serPort);
    x = state(1);
    y = state(2);
    record{end+1} = [x y];
    % Mostrar recorrido 
    if (numel(record)>1)
        p1 = [record{end}(1);record{end-1}(1)];
        p2 = [record{end}(2),record{end-1}(2)];
        l = plot(p1,p2,'g','LineWidth',15);
        uistack(l,'bottom');
    end
    for i=1:(wy/pass)
       for j=1:(wx/pass)
           % Si el robot esta dentro de una casilla, asignar la casilla
           % como aspirada
           if grid(i,j,1)<=x && grid(i,j,2)>x && grid(i,j,3)>=y && grid(i,j,4)<y
               % Asignar 0  la casilla recorrida 
               grid(i,j,:)=[0 0 0 0];
           end
       end
    end
    
    % Buscar la cantidad de casillas recorridas
    aspirado = find(grid(:,:,1)==0 & grid(:,:,2)==0 & grid(:,:,3)==0 & grid(:,:,4)==0);
    n = numel(aspirado);
    fprintf('Aspirado:\n');
    tam=size(grid);
    % Tamaño mapa
    tam=tam(1)*tam(2);
    % Tamaño mapa quitando tamaño obstaculos
    tam = tam - gridObstaculos;
    % Calcular porcentaje del mapa aspirado 
    pct = (n/tam)*100
    if pct>10
         fprintf('Criterio de parado alcanzado\n');
         SetDriveWheelsCreate(serPort, 0,0)
         pause;
    end
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%
function driveForwardUntilWall(serPort)
% Robot drives forward until it bumps a wall.
% serPort is the serial port number (for controlling the actual robot).
    [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...
        BumpsWheelDropsSensorsRoomba(serPort);
    while ~BumpRight && ~BumpLeft && ~BumpFront 
        SetDriveWheelsCreate(serPort, .2,.2)
        % Calcular porcentaje despues de "dar un paso"
        calcularPorcentaje(serPort)
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
