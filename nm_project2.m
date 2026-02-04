%% NEPTUNE-R true trajectory
%  This simulation contains realistic rover trajectory
%  calculation on a lake using:
%  - Euler's method for solving equations
%  - Shore avoidance via safeLake buffer
%  - Island avoidance via local 90deg turns
%  - Conditional oxygenation based on % saturation

clearvars;
close all;
clc;

%% 1 Define Lake Boundaries
lake_xy = [...
   0   0;
   1.2 0.1;
   1.1 0.9;
   0.2 0.95;
  -0.1 0.5
];
lake        = polyshape(lake_xy);
safe_margin = 0.05;            % “safe” buffer from shore
safeLake    = polybuffer(lake, -safe_margin);

island_xy = [ ...
    0.15 0.25;
    0.25 0.4;
    0.35 0.3;
    0.4 0.2
];
island = polyshape(island_xy);

%% 2 Plot Lake, Safe Boundary & Island
figure; hold on; axis equal
plot(lake,     'FaceColor',[.8 .9 1],'EdgeColor','b');
plot(safeLake, 'FaceColor','none','EdgeColor','r','LineStyle','--');
plot(island,   'FaceColor',[.7 .85 .7],'EdgeColor','g');
title('NEPTUNE-R safe boundary & Island');
legend('Shore','Safe boundary','Island','Location','best');

%% 3 Boustrophedon coverage path S
% 3.1 carve island out of safeLake
freeSpace = subtract(safeLake, island);

% 3.2 get x/y limits correctly
[xlim, ylim] = boundingbox(freeSpace);
xmin = xlim(1);  xmax = xlim(2);
ymin = ylim(1);  ymax = ylim(2);
dy   = 0.03;       % vertical spacing between passes
nx   = 300;     % resolution along each scan‐line

% 3.3 build the back‐and‐forth runs
path = [];
dir  = 1;     % 1 = left->right, -1 = right->left
xgrid = linspace(xmin, xmax, nx);
for y = ymin : dy : ymax
    xi = xgrid;
    yi = y * ones(size(xi));

    IN   = isinterior(freeSpace, xi, yi);
    runs = bwlabel(IN);     % contiguous in‐water segments

    for r = 1:max(runs)
        idx = runs==r;
        pts = [xi(idx)' , yi(idx)'];
        if dir<0
            pts = flipud(pts);
        end
        path = [path; pts];    
    end

    dir = -dir;  % reverse sweep on next row
end

% 3.4 quick plot to verify
figure; hold on; axis equal
plot(lake,     'FaceColor',[.8 .9 1],'EdgeColor','b');
plot(safeLake, 'FaceColor','none','EdgeColor','r','LineStyle','--');
plot(island,   'FaceColor',[.7 .85 .7],'EdgeColor','g');
plot(path(:,1), path(:,2), '.','MarkerSize',3,'Color',[.5 .5 .5]);
legend('Shore','Safe boundary','Island','Labyrinth waypoints','Location','best');

%% 4 Simulation Parameters & Initialization
% scale = 1:100
dt     = 0.1;     % s
Vmax   = 0.3;     % m/s

% Oxygenation parameters
% raw DO in mg/L
do_raw_field = @(x,y) 7 + 1.5*sin(2*pi*x).*cos(2*pi*y);
% convert to % sat (assume 9 mg/L = 100%)
DO_sat_ref   = 9.0;    % mg/L at reference
DO_field_pct = @(x,y) 100 * do_raw_field(x,y) / DO_sat_ref;
DO_thr_pct   = 80;      % trigger oxygenation below 80%

% initial state
pos     = path(1,:);
vel     = [0,0];
history = pos;

%% 5 Main Loop + island avoidance
for k = 2:size(path,1)
    tgt = path(k,:);
    d   = tgt - pos;
    if norm(d) < 0.02
        continue;  % next waypoint
    end
    v_des = (d/norm(d)) * Vmax;

    % island avoidance: if next step lands inside island, rotate 90degS
    cand = pos + v_des*dt;
    if isinterior(island, cand(1), cand(2))
        v1 = [-v_des(2), v_des(1)];
        if ~isinterior(island, pos+v1*dt)
            v_des = (v1/norm(v1))*Vmax;
        else
            v2 = [v_des(2), -v_des(1)];
            if ~isinterior(island, pos+v2*dt)
                v_des = (v2/norm(v2))*Vmax;
            end
        end
    end

    % Euler integration
a    = (v_des - vel)/dt; % a  = (targetVelocity - currentVelocity) / dt
    vel  = vel + a*dt; % v_{t+dt} = v_t + a_t * dt     -- velocity update
    pos  = pos + vel*dt; % p_{t+dt} = p_t + v_t * dt     -- position update
    history(end+1,:) = pos;

    % conditional oxygenation using % sat
    pct = DO_field_pct(pos(1), pos(2));
    if pct < DO_thr_pct % if position o2 lvl < thr 
        fprintf('[%5.1f s] DO=%.1f%% < %d%% -> oxygenate at (%.2f,%.2f)\n', ...
                k*dt, pct, DO_thr_pct, pos(1), pos(2));
    end

    % shore collision check
    if ~isinterior(lake, pos(1), pos(2))
        warning('Boat ran aground at step %d!', k);
        break;
    end
end

%% 6 Plot the Boat’s Route
figure; hold on; axis equal
plot(lake,     'FaceColor',[.8 .9 1],'EdgeColor','b');
plot(island,   'FaceColor',[.7 .85 .7],'EdgeColor','g');
plot(history(:,1), history(:,2), '-r','LineWidth',1.5);
scatter(history(1,1), history(1,2), 80,'ko','filled');% start
xlabel('X (map units)'); ylabel('Y (map units)');
title('Lake w NEPTUNE-R');
legend('Shore','Island','Boat path','Start','Location','best');
