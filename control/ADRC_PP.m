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


%% ADRC-FL CONTROLLER (Step internal model for disturbance)
K1 = 48; K2 = 320; % Poles in -4,-20

A = [0, 1, 0; 0, 0, 1; 0, 0, 0];
B = [0; 2*lf*Caf/Izz; 0];

eso_poles = [-38, -39, -40];
C = [1, 0, 0; 0, 1, 0];

L = place(A', C', eso_poles)';

Aobs2 = A-L*C;
Bobs2 = [B L];
Cobs2 = eye(3);
Dobs2 = zeros(3);

observer = ss(Aobs2, Bobs2, Cobs2, Dobs2);
observer_discrete = c2d(observer, T);

%% ADRC-FL CONTROLLER (Ramp internal model for disturbance)
K1 = 48; K2 = 320; % Poles in -4,-20

A = [0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1; 0, 0, 0, 0];
B = [0; 2*lf*Caf/Izz; 0; 0];

eso_poles = [-37, -38, -39, -40];
C = [1, 0, 0, 0; 0, 1, 0, 0];

L = place(A', C', eso_poles)';

Aobs2 = A-L*C;
Bobs2 = [B L];
Cobs2 = eye(4);
Dobs2 = zeros(4,3);

observer_v2 = ss(Aobs2, Bobs2, Cobs2, Dobs2);
observer_discrete_v2 = c2d(observer_v2, T);




%% Evaluate trajectory following:
% WARNING: RUN THIS AFTER RUNNING THE SIMULINK SIMULATION TO PLOT X,Y
% TRACKING RESULTS
plot(out.X.Data, out.Y.Data)
hold on
plot(out.X_des.Data, out.Y_des.Data)


