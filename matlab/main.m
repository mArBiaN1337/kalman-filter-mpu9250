clc
clearvars
close all

imu_data = readtable("imu_data_x_vertical.txt","ReadRowNames",false, ...
    "VariableNamingRule","preserve", ...
    "Delimiter","," );

imu_data = renamevars(imu_data, ["ax (m/s^2)","ay (m/s^2)","az (m/s^2)"], ...
                                ["ax", "ay", "az"]);

imu_data = renamevars(imu_data, ["gx (rps)", "gy (rps)", "gz (rps)", "temp (C)"], ...
                                ["gx", "gy", "gz", "temp"]);


struct_imu = table2struct(imu_data, "ToScalar",true);

stu_imu.units = {'m/s^2', 'rps', 'celsius'};

f = 100;
Ts = 1/f;

A = eye(6);
B = zeros(6,1);
C = eye(6);
D = 0;

A(1,4) = Ts;
A(2,5) = Ts;
A(3,6) = Ts;

sys = ss(A, B, C, D, Ts);

Q = 0.1;
R = 1;

[kalmf, L, P] = kalman(sys, Q, R);










