function randomwalk
    posMinBound = [0 0];
    posMaxBound = [15 15];
    
    starting_pos = [ 7.5, 7.5];
    starting_angle = 45;
    
    
    speed = 0.1;
    turning_speed = 1;
    
    pos =starting_pos;
    angle = starting_angle;
    
    num_iterations = 100;
    
    for i=1:num_iterations
        
        tspeed = speed;
        turn = 0;
        aux = mod(i, 1);
        if aux == 0
            turn = rand()*turning_speed;
            if rand() > 0.5
                turn = -turn;
            end
        end
        angle = angle + turn;

        pos(1) = pos(1)+tspeed*sin(angle);
        pos(2) = pos(2)+tspeed*cos(angle);
        
        clf;
        hold on;
        
        draw_car(0.5, pos(1), pos(2), 360-angle);
        plot_scenario(posMinBound, posMaxBound);
        
        hold off;
        
        
        pause(0.1);
        
    end

end


function plot_scenario(posMinBound, posMaxBound)

    axis([posMinBound(1) posMaxBound(1) posMinBound(2) posMaxBound(2)]);

end

function draw_car(scale, x, y, h)

    % Make a circle for the body
    t=0:0.01:2*pi;
    len = length(t);
    bx = sin(t);
    by = cos(t);

    % Wheel locations on body
    wh1 = round(len/4);
    wh2 = round(3*len/4);

    % Draw the wheels
    wwidth= 0.2;
    wheight = 0.4;
    w = [0 -wheight;wwidth -wheight; wwidth wheight; 0 wheight; 0 0];

    % Body top
    top = round(len/2);
    % Top pointer
    pwidth = 0.1;
    pheight = 0.2;
    tp = [pwidth/2 0; 0 -pheight; -pwidth/2 0; pwidth/2 0];

    % Car outline
    car = [bx(1:wh1)' by(1:wh1)';
        bx(wh1)+w(:,1) by(wh1)+w(:,2);
        bx(wh1:wh2)' by(wh1:wh2)';
        bx(wh2)-w(:,1) by(wh2)-w(:,2);
        bx(wh2:end)' by(wh2:end)'];

    point = [bx(top)+tp(:,1) by(top)+tp(:,2)];

    %Size scaling
    car = scale*car;
    point = scale*point;

    % Rotation matrix
    R = [cos(h+pi/2) -sin(h+pi/2); sin(h+pi/2) cos(h+pi/2)];
    car = (R*car')';
    point = (R*point')';

    % Centre
    car(:,1) = car(:,1)+x;
    car(:,2) = car(:,2)+y;
    point(:,1) = point(:,1)+x;
    point(:,2) = point(:,2)+y;

    % Plot
    plot(car(:,1), car(:,2), 'b');
    plot(point(:,1), point(:,2), 'r');
    axis equal
end