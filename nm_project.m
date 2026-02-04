%% Numerical methods extra path calculator
%  - Archimedean spiral coverage on a lake with obstacle
%  - Shore avoidance
%  - Island avoidance via local 90deg turns
%  - Oxygenation based on % saturation

clearvars;
close all;
clc;

%% 1 Lake & island boundary
lake_xy = [...
   0   0;
   1.2 0.1;
   1.1 0.9;
   0.2 0.95;
  -0.1 0.5
];
lake        = polyshape(lake_xy);
safe_margin = 0.05;          % “safe” buffer from shore
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

%% 3 Generate archimedean spiral waypoints
a     = 0.02;   % spiral params
b     = 0.02;
theta = linspace(0,20*pi,5000);
r     = a + b.*theta;
cx    = mean(lake_xy(:,1));
cy    = mean(lake_xy(:,2));
xx    = cx + r.*cos(theta);
yy    = cy + r.*sin(theta);
IN    = isinterior(safeLake, xx, yy);
path_spiral = [xx(IN)', yy(IN)'];

plot(path_spiral(:,1), path_spiral(:,2), ...
    '.', 'MarkerSize',4, 'Color',[.5 .5 .5]);
legend('Shore','Safe boundary', ...
    'Island','Spiral waypoints','Location','best');

%% 4 Simulation parameters & initialization
% scale = 1:100
dt       = 0.1;    % time step (s) 
Vmax     = 0.3;    % max speed (m/s)

% mock DO field in mg/L (base + spatial variation)
DO_field_raw = @(x,y) 7 + 1.5*sin(2*pi*x).*cos(2*pi*y);

% convert to % saturation (assume saturation ~9 mg/L)
DO_sat      = 9.0; % mg/L saturation
DO_field    = @(x,y) 100 * DO_field_raw(x,y) / DO_sat;
DO_thr_pct  = 80;        % threshold 80%

%init state
pos     = path_spiral(1,:);
vel     = [0,0];
history = pos;

%% 5 Main loop - waypoint with island avoidance
for k = 2:size(path_spiral,1)
    tgt = path_spiral(k,:);
    d   = tgt - pos;
    if norm(d) < 0.02
        continue; % close enough to waypoint
    end
    v_des = (d/norm(d)) * Vmax;

    % Island avoidance: if next step enters island, rotate 90 deg
    cand = pos + v_des*dt;
    if isinterior(island, cand(1), cand(2))
        v1 = [-v_des(2), v_des(1)];
        if ~isinterior(island, pos+v1*dt)
            v_des = (v1/norm(v1)) * Vmax; %+90deg
        else
            v2 = [v_des(2), -v_des(1)];
            if ~isinterior(island, pos+v2*dt)
                v_des = (v2/norm(v2)) * Vmax; %-90deg
            end
        end
    end

    % Euler integration
a    = (v_des - vel)/dt; % a  = (targetVelocity - currentVelocity) / dt
    vel  = vel + a*dt; % v_{t+dt} = v_t + a_t * dt     -- velocity update
    pos  = pos + vel*dt; % p_{t+dt} = p_t + v_t * dt     -- position update
    history(end+1,:) = pos;

    % Conditional oxygenation based on % sat
    pct = DO_field(pos(1), pos(2));
    if pct < DO_thr_pct %will oxygenate if < thr
        fprintf('[%5.1f s] DO=%.1f%% < %d%%  --> oxygenate at (%.2f,%.2f)\n', ...
                k*dt, pct, DO_thr_pct, pos(1), pos(2));
    end

    % check 4 shore collision
    if ~isinterior(lake, pos(1), pos(2))
        warning('Boat ran aground at step %d!', k);
        break;
    end
end

%% 6 Plot the boat s actual route
figure; hold on; axis equal
plot(lake,   'FaceColor',[.8 .9 1],'EdgeColor','b');
plot(island, 'FaceColor',[.7 .85 .7],'EdgeColor','g');
plot(history(:,1), history(:,2), '-r', 'LineWidth',1.5);
scatter(history(1,1), history(1,2), 80, 'ko', 'filled');
xlabel('X (map units)'); ylabel('Y (map units)');
title('NEPTUNE-R Trajectory');
legend('Shore','Island','Boat path','Start','Location','best');








