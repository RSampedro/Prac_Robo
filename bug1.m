function bug1

global sensor_range;
sensor_range = 1;

global world_rectangles;

startPos= [2 3];
endPos = [12 14];


minLen.a = 1;
maxLen.a = 6;
minLen.b = 1;
maxLen.b = 5;

obstBuffer = 1;
maxCount = 10000;

global numObsts;    
numObsts = 3;
global posMinBound;
posMinBound = [0 0];
global posMaxBound;
posMaxBound = [15 15];

[aObsts, bObsts, world_rectangles] = polygonal_world(posMinBound, posMaxBound, minLen, maxLen, numObsts, startPos, endPos, obstBuffer, maxCount);

world_rectangles = [    1 6     3 12     9 6          7 2     13 2        ;
                        3 6    3 14     9 14          7 4     13 10       ;
                        3 14    9 14     11 14       13 4    15 10       ;
                        1 14     9 12     11 6        13 2    15 2        ;];



bug2_exec(startPos, endPos);


end


function [x_points, y_points] = bug2_exec(qstart, qgoal)
    global sensor_range;
    global world_rectangles;
    global posMinBound;
    global posMaxBound;
    
    robot_step = (sensor_range * 0.075);
    x_points = [];
    y_points = [];
    x_points = [x_points; qstart(1)];
    y_points = [y_points; qstart(2)];
    
    position = qstart;
    
    robot_size = robot_step*2;
    
    turn_distance = pi()/20.0;
    
    line_ang = atan2((qgoal(2) - qstart(2)), (qgoal(1) - qstart(1)));
    line_ang = normalize_ang(line_ang);
    
    mode = 0;
    starting_pos = [999,999];
    best_pos = [999, 999];
    best_dist = 999;
    flag = 0;
    flag_entrada = 0;
    flag_sortida = 0;
    
    while(  position(1) > qgoal(1) + robot_size*0.5||...
            position(1) < qgoal(1) - robot_size*0.5||...
            position(2) > qgoal(2) + robot_size*0.5||...
            position(2) < qgoal(2) - robot_size*0.5    )
        if (mode == 0)
            turn_distance = pi()/20.0 ;
            angle = atan2((qgoal(2) - position(2)), (qgoal(1) - position(1)));
            position(1) = position(1) + cos(angle)*robot_step;
            position(2) = position(2) + sin(angle)*robot_step;
            x_points = [x_points position(1)];
            y_points = [y_points position(2)];

            min_dist = read_sensor(angle, [position(1) position(2)]);

            if (min_dist < robot_size)
                dist_goal = sqrt((qgoal(2) - position(2))^2 + (qgoal(1) - position(1))^2);
                mode = 1;
                flag = 1;
                flag_entrada = 0;
                flag_sortida = 0;

            end

        elseif (mode == 1)
            if flag == 1
                starting_pos = [position(1), position(2)];
                best_pos = [0,0];
                flag = 0;
            end
            sens_dist = [];
            num_compr = 90;
            chk_angles = linspace(0, 2*pi(), num_compr);
            
            for i = 1:num_compr
                sens_dist = [sens_dist ; read_sensor(chk_angles(i), position)];
            end
            
            [min_d, ang_idx] = min(sens_dist);
            min_ang = chk_angles(ang_idx);
            min_d = read_sensor(min_ang, position);
            
            angle = min_ang - pi()/2;
            angle = normalize_ang(angle);
            
            if (min_d < robot_size)
               angle = angle - turn_distance;
            elseif (min_d > 2*robot_size)
                angle = angle + turn_distance;
            end
            
            position(1) = position(1) + cos(angle) * robot_step;
            position(2) = position(2) + sin(angle) * robot_step;
            
            x_points = [x_points position(1)];
            y_points = [y_points position(2)];
            
            angle = atan2((qgoal(2) - position(2)), (qgoal(1) - position(1)));
            angle = normalize_ang(angle);
            
            temp = sqrt((qgoal(2) - position(2))^2 + (qgoal(1) - position(1))^2);
            if (temp < best_dist)
               best_pos = position;
               best_dist = temp;
            end
            
            dist_to_start = sqrt((starting_pos(2) - position(2))^2 + (starting_pos(1) - position(1))^2);
            
            if ((flag_entrada == 1) && (dist_to_start < sensor_range/2) && (flag_sortida == 0))
                flag_sortida = 1;
            end
            
            if(abs(angle - line_ang) < pi()/180 || ...
                    abs(line_ang - pi()) < pi()/180 || ...
                    abs(angle - line_ang + pi()) < pi()/180)
                next_obs = read_sensor(angle, position);
                if (next_obs > robot_size)
                    flag_entrada = 1;
                end
            end
            
            
            if (flag_sortida == 1)
                temp = sqrt((best_pos(2) - position(2))^2 + (best_pos(1) - position(1))^2);
                if (temp < sensor_range/2)
                    mode = 0;
                    flag_entrada = 0;
                    flag_sortida = 0;
                end
            end
            flag = 0;
        end
    %clf;
    hold on;
    plot(position(1),position(2),'*')
    plotEnvironment(world_rectangles,posMinBound, posMaxBound, qstart, qgoal)
    %draw_car(0.5, position(1), position(2), angle)
        
    pause(0.01);
        
    hold off;
        %Mostramos la distancia restante al goal por pantalla
        %newdist_to_goal = sqrt((goal(2) - posicion(2))^2 + (goal(1) - posicion(1))^2)
    end
end


        

function newPoint = pointFromAlpha(currentPosition, alpha, step)
    yMove = sin(alpha);
    xMove = cos(alpha);
    newX = currentPosition(1) + xMove * step;
    newY = currentPosition(2) + yMove * step;
    newPoint = [newX, newY];
end

function dis = read_sensor(angle, current_point)
    
    global world_rectangles;
    global sensor_range;
    global numObsts;
    
    precision = 20;
    
    dis = inf;
    k = 1;
    
    next_point = pointFromAlpha(current_point, angle, sensor_range);
    point_vector(1,:) = linspace(current_point(1), next_point(1), precision);
    point_vector(2,:) = linspace(current_point(2), next_point(2), precision);
    
    while (k < numObsts*2)
        
        for i = 1:size(point_vector,2)
            in = inpolygon(point_vector(1, i), point_vector(2, i), world_rectangles(1:4, k), world_rectangles(1:4, k+1));
            
            if (in == 1)
                new_dis = pdist([[point_vector(1, i), point_vector(2,i)]; current_point]);
                if (new_dis < dis)
                    dis = new_dis;
                end
            end
        end
        k = k + 2;
    end
    
end

function [ang] = normalize_ang(angle)
    ang = angle;
    while (ang<0 ) ang>2*pi()
        if (ang<0)
            ang = ang+2*pi();
        elseif (ang>2*pi())
            ang = ang-2*pi();
        end
    end
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


function [a,b,ptsStore] = polygonal_world(posMinBound, posMaxBound, minLen, maxLen, numObsts, startPos, endPos, obst_buffer, max_count)

a = zeros(numObsts,4,2);
b = zeros(numObsts,4);

for j = 1:1:numObsts
    a(j,1,:) = [0 -1];
    a(j,2,:) = [1 0];
    a(j,3,:) = [0 1];
    a(j,4,:) = [-1 0];
end

%create the number of obsts
count = 0;
for i = 1:1:numObsts
    %loop while there are collisions with obstacles
    while(1)
        %generate random positions and lengths
        pos(i,:) = posMinBound + [rand(1)*(posMaxBound(1)-posMinBound(1)),rand(1)*(posMaxBound(2)-posMinBound(2))];
        len(i,:) = [rand(1)*(maxLen.a-minLen.a)+minLen.a,rand(1)*(maxLen.b-minLen.b)+minLen.b];
        fake_len(i,:) = len(i,:) + obst_buffer;
        theta(i) = rand(1)*pi;


        rotationMatrix = [cos(theta(i)) sin(theta(i)); -sin(theta(i)) cos(theta(i))];
        %find the points
        pts = [-len(i,1)/2, -len(i,2)/2; len(i,1)/2, -len(i,2)/2; ...
               len(i,1)/2, len(i,2)/2; -len(i,1)/2, len(i,2)/2;];
        fake_pts = [-fake_len(i,1)/2, -fake_len(i,2)/2; fake_len(i,1)/2, -fake_len(i,2)/2; ...
               fake_len(i,1)/2, fake_len(i,2)/2; -fake_len(i,1)/2, fake_len(i,2)/2;];
        for j = 1:1:4
            pts(j,:) = (rotationMatrix*(pts(j,:)'))' + [pos(i,1) pos(i,2)];
            fake_pts(j,:) = (rotationMatrix*(fake_pts(j,:)'))' + [pos(i,1) pos(i,2)];
        end
        
        

        %%need to redo these checks
    
        %check to see if it is outside the region
        if( min(fake_pts(:,1)) <= posMinBound(1) || max(fake_pts(:,1)) >= posMaxBound(1) || min(fake_pts(:,2)) <= posMinBound(2) || max(fake_pts(:,2)) >= posMaxBound(2))
            continue;
        end
        
        %check to see if it is on top of the start pos
%         if( pos(i,1) - len(i,1)/2 < startPos(1) && pos(i,2) - len(i,2)/2 < startPos(2) && pos(i,1) + len(i,1)/2 > startPos(1) && pos(i,2) + len(i,2)/2 > startPos(2))
%             continue;
%         end
        
        if( min(pts(:,1)) < startPos(1) && max(pts(:,1)) > startPos(1) && min(pts(:,2)) < startPos(2) && max(pts(:,2)) > startPos(2))
            continue;
        end

        %check to see if it is on top of the end pos
        if( min(pts(:,1)) < endPos(1) && max(pts(:,1)) > endPos(1) && min(pts(:,2)) < endPos(2) && max(pts(:,2)) > endPos(2))
            continue;
        end
        
%         if( pos(i,1) - len(i,1)/2 < endPos(1) && pos(i,2) - len(i,2)/2 < endPos(2) && pos(i,1) + len(i,1)/2 > endPos(1) && pos(i,2) + len(i,2)/2 > endPos(2))
%             continue;
%         end

        %check to see if it collided with any of the other obstacles
        collided = 0;
        for j = 1:1:(i-1)
            %check for collision
            %             if(abs(pos(j,1) - pos(i,1)) <= (len(i,1) + len(j,1))/2 && abs(pos(j,2) - pos(i,2)) <= (len(i,2) + len(j,2))/2)
            %                 collided = 1;
            %                 break;
            %             end

            if(polygonsOverlap(fake_pts, ptsStore(:,(j-1)*2+1:(j-1)*2+2)))
                collided = 1;
                break;
            end
            
%             if(~isempty(polyxpoly(pts(:,1),pts(:,2),ptsStore(:,(j-1)*2+1),ptsStore(:,(j-1)*2+2))))
%                 collided = 1;
%                 break;
%             end
        end

        if(~collided)
            break;
        end
        count = count + 1;
        if (count >= max_count)
            a = [];
            b = [];
            ptsStore = [];
            return;
        end
    end

    ptsStore(:,(i-1)*2+1:(i-1)*2+2) = pts;

    for j = 1:1:4
%         temp = rotationMatrix*[a(i,j,1);a(i,j,2);];

        next = j+1;

        if(j == 4)
            next = 1;
        end
        temp = [-(pts(j,2) - pts(next,2)); (pts(j,1) - pts(next,1))];
        temp = temp/norm(temp);
        a(i,j,1) = temp(1);
        a(i,j,2) = temp(2);
    end

    %calculate the b matrix
    for k = 1:1:4
        for j = 1:1:2
            b(i,k) = b(i,k) + pts(k,j)*a(i,k,j);
        end
    end
end


%calculate the points of each rectangle
for i = 1:1:numObsts
    pts = zeros(4,2);
    for j = 1:1:4
        if(j ~= 4)
            a1_1 = a(i,j,1);
            a1_2 = a(i,j,2);
            a2_1 = a(i,j+1,1);
            a2_2 = a(i,j+1,2);

            b1 = b(i,j);
            b2 = b(i,j+1);
        else
            a1_1 = a(i,j,1);
            a1_2 = a(i,j,2);
            a2_1 = a(i,1,1);
            a2_2 = a(i,1,2);

            b1 = b(i,j);
            b2 = b(i,1);
        end
        pts(j,:) = [-(a1_2*b2 - b1*a2_2)/(a1_1*a2_2 - a2_1*a1_2),   (a1_1*b2 - a2_1*b1)/(a1_1*a2_2 - a2_1*a1_2)];
    end
    ptsStore(:,(i-1)*2+1:(i-1)*2+2) = pts;
end
end

function val = polygonsOverlap(poly1, poly2)
%POLYGONSOVERLAP Checks if two polygons intersect
%   poly1 - N1x2 matrix of x,y coordinates of polygon vertices
%   poly2 - N2x2 matrix of x,y coordinates of polygon vertices
% ASSUMES:
%   - Polygon vertices are ordered counter clockwise (can be enforced with
%     our scripts)

val = false;

% Simple test to check if 1 is fully or partially enclosed in polygon 2
if sum(inpolygon(poly1(:,1),poly1(:,2),poly2(:,1),poly2(:,2)))
    val = true;
    return
end

% Simple test to check if 2 is fully or partially enclosed in polygon 1
if sum(inpolygon(poly2(:,1),poly2(:,2),poly1(:,1),poly1(:,2)))
    val = true;
    return
end

% Close the polygons
poly1 = [poly1;poly1(1,:)];
obstEdges = [poly2,[poly2(2:end,:);poly2(1,:)]];
% Loop over all possible intersections
for vertex = 1:(length(poly1)-1)
    if (CheckCollision(poly1(vertex,:), poly1(vertex+1,:), obstEdges))
        val = true;
        return
    end
end
end

function [ inCollision, edge ] = CheckCollision( ptA, ptB, obstEdges )
%CHECKCOLLISION Checks if a line interesects with a set of edges
%   Detailed explanation goes here

% Check for each edge
edge = [];
for k = 1:size(obstEdges,1)
    % If both vertices aren't to the left, right, above, or below the
    % edge's vertices in question, then test for collision
    if ~((max([ptA(1),ptB(1)])<min([obstEdges(k,1),obstEdges(k,3)])) || ...
         (min([ptA(1),ptB(1)])>max([obstEdges(k,1),obstEdges(k,3)])) || ...
         (max([ptA(2),ptB(2)])<min([obstEdges(k,2),obstEdges(k,4)])) || ...
         (min([ptA(2),ptB(2)])>max([obstEdges(k,2),obstEdges(k,4)])))
        if (EdgeCollision([ptA, ptB], obstEdges(k,:)))
            % Eliminate end-to-end contacts from collisions list
            if (sum(abs(ptA-obstEdges(k,1:2)))>0 && ...
                sum(abs(ptB-obstEdges(k,1:2)))>0 && ...
                sum(abs(ptA-obstEdges(k,3:4)))>0 && ...
                sum(abs(ptB-obstEdges(k,3:4)))>0)
            
                edge = k;
                inCollision = 1 ; % In Collision
                return
            end
        end
    end
end
inCollision = 0 ; % Not in collision
end

function [ colliding, pt ] = EdgeCollision( Edge1, Edge2, tol )
%EDGECOLLISION Checks if two edges are colliding
%   Edge1 -> First edge being checked
%   Edge2 -> Second edge being checked
%   return-> 1: if colliding
%            0: if not colliding
% Edges are represented by endpoints [x1 y1 x2 y2]

if nargin < 3
    tol = 0;
end


p1 = Edge1(1:2);
p2 = Edge1(3:4);
p3 = Edge2(1:2);
p4 = Edge2(3:4);

% If lines are the same
if (abs((p1(1)-p3(1)) <= tol) && (abs(p1(2)-p3(2))<=tol) && (abs(p2(1)-p4(1))<=tol) && (abs(p2(2)-p4(2))<=tol))
  colliding = true;
  pt = p1;
  return;
end

% From Introduction to Algorithms, Cormen et. al.
d = cross([p1-p3,0; p2-p3,0; p3-p1,0; p4-p1,0]',...
          [p4-p3,0; p4-p3,0; p2-p1,0; p2-p1,0]');
d = d(end,:);

colliding = false;
if ((d(1)>0 && d(2)<0) || (d(1)<0 && d(2)>0)) && ((d(3)>0 && d(4)<0) || (d(3)<0 && d(4)>0))
    colliding = true;
elseif (abs(d(1))<100*eps && OnSegment(p3,p4,p1))
    colliding = true;
elseif (abs(d(2))<100*eps && OnSegment(p3,p4,p2))
    colliding = true;
elseif (abs(d(3))<100*eps && OnSegment(p1,p2,p3))
    colliding = true;
elseif (abs(d(4))<100*eps && OnSegment(p1,p2,p4))
    colliding = true;
end
   
% Find intersection point
if (colliding)
    [ e1.A, e1.b ] = EdgePtsToVec( Edge1 );
    [ e2.A, e2.b ] = EdgePtsToVec( Edge2 );
    [pt, colliding] = AffineIntersect( e1, e2 );
else
    pt = [0 0];
end

return
end

function val = OnSegment(pi,pj,pk)
% OnSegment determines if a point known to be collinear with a segment
% lies on that segment.  Collinearity is established by a zero cross
% product.
% NOTE: this differs from the textbook algorithm in an important way.  The
% inequalities are <, not <=, so that coincident endpoints will not be
% considered collisions, but an endpoint lying in the middle of another
% line segment will be, as happens at corners.
if (min([pi(1),pj(1)]) <= pk(1) && pk(1) <= max([pi(1),pj(1)]))...
   && (min([pi(2),pj(2)]) <= pk(2) && pk(2) <= max([pi(2),pj(2)]))
    val = true;
else
    val = false;
end
end

function plotEnvironment(ptsStore,posMinBound, posMaxBound, startPos, endPos)

hold on
grid on
for i = 1:2:size(ptsStore,2)
    patch(ptsStore(:,i),ptsStore(:,i+1),[1 0 0],'linestyle','-','FaceColor',[1 0 0],'EdgeColor',[1 0 0], 'LineWidth',2)
end
plot(startPos(1),startPos(2),'b*')
plot(endPos(1),endPos(2),'g*')
patch([posMinBound(1) posMaxBound(1) posMaxBound(1) posMinBound(1)],[posMinBound(2) posMinBound(2) posMaxBound(2) posMaxBound(2)],...
    [1 0 0],'FaceColor','none','LineStyle','-','EdgeColor',[0 1 1])
hold off
axis equal
axis([posMinBound(1) posMaxBound(1) posMinBound(2) posMaxBound(2)]);
end

function [ A, b ] = EdgePtsToVec( edge )
%EDGEPTSTOVEC Converts an edge from point notation to vector notation
%   edge -> Edge represented by endpoints [x1 y1 x2 y2]
%   returns: Line in vector form Ax = b

f = [edge(1); edge(2); 0 ] ;
g = [edge(3); edge(4); 0 ] ;

A = cross(f-g, [0;0;1]) ;
A = A' / norm(A) ;
b = A*f ;
A = A(1:2) ;

end