%% Parameters:
m = 630; Izz = 1000; lf = 1.63886; lr = 1.248156;
Caf = 140000; Car = 140000; vx = 20;
T = 0.01; la_ratio = 0.6;

%% Generate reference path:
poly_path = [0.01, 0.001, 3];
start_point = 0;
end_point = 1000;
X_path = linspace(start_point, end_point, 1000);
Y_path = polyval(poly_path, X_path);


%% ADRC-FL CONTROLLER (OPTION B)
K1 = 24; K2 = 80; % Poles in -4,-20

A = [0, 1, 0; 0, 0, 1; 0, 0,  0];
B = [0; 2*lf*Caf/Izz; 0];

eso_poles = [-38, -39, -40];
C = [1, 0, 0; 0, 1, 0]; D = zeros(2,1);

plant = ss(A, B, C, D);
plant_discrete = c2d(plant, 0.01, 'zoh');

L = place(A', C', eso_poles)';

Aobs2 = A-L*C;
Bobs2 = [B L];
Cobs2 = eye(3);
Dobs2 = zeros(3);

observer = ss(Aobs2, Bobs2, Cobs2, Dobs2);
observer_discrete = c2d(observer, T);




%% Evaluate trajectory following:
% WARNING: RUN THIS AFTER RUNNING THE SIMULINK SIMULATION TO PLOT X,Y
% TRACKING RESULTS
plot(out.X.Data, out.Y.Data, 'bo')
hold on
plot(X_path, Y_path, 'ro')

%% Test lookahead finder:

% Calibration values:
X_car = 68; Y_car = 55; heading = 0; lookahead_distance = 10;
% Xp = 45; Yp = 20; 
% 
% la_point = get_la_point(Xp, Yp, X_car, Y_car, heading, lookahead_distance);

[la_point, transform] = get_la_point(X_path, Y_path, X_car, Y_car, heading, lookahead_distance);

euclidean_distance = sqrt(la_point(1)^2 + la_point(2)^2);

plot(X_path, Y_path); hold on;
xlim([X_car - euclidean_distance, X_car + euclidean_distance]);
ylim([Y_car - euclidean_distance, Y_car + euclidean_distance]);
plot(X_car, Y_car, 'ro');

% Transform point back to global to calibrate algorithm:
la_point_global = transform*la_point;
plot(la_point_global(1), la_point_global(2), 'bo');

%% Get la_point function:

function [desired_path_point, Toc] = get_la_point(X_path, Y_path, X_car, Y_car, heading, lookahead_distance)

    la = lookahead_distance;
    
    % 1. Get global -> cog transform:
    
    Toc = [cos(heading), -sin(heading), 0, X_car; 
        sin(heading), cos(heading), 0, Y_car; 
        0, 0, 1, 0;
        0, 0, 0, 1];
    
    % P_tar_o = [X_path, Y_path, 0, 1]';
    % desired_path_point = inv(Toc)*P_tar_o;

    % 2. Iteratively find the lookahead distance point transforming global path
    % points to cog frame:
    la_point_found = false; 

    for i = 1:length(X_path)
        P_tar_o = [X_path(i), Y_path(i), 0, 1]'; % o subscript stands for global frame, c stands for car frame (cog)
        P_tar_c = inv(Toc)*P_tar_o; % Change for \ operator for faster inversion

        if (P_tar_c(1) > 0)
            euclidean_distance = sqrt(P_tar_c(1)^2 + P_tar_c(2)^2);
            if (euclidean_distance > 0.9*la && euclidean_distance < 1.1*la)
                la_point_found = true;
                break;
            end
        else
            continue;
        end
    end

    if ~la_point_found
        disp('ERROR, NOT FOUND')
    end
    disp(i)
    desired_path_point = P_tar_c;

end


