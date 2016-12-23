%% Start with clearing and cleaning stuff.

clear all
clc
close all
%% Define the frames

L0 = Link('d', 0.25, 'a', 0, 'alpha', pi/2)
L1 = Link('d', 0, 'a', 0.5, 'alpha', 0)
L2 = Link('d', 0, 'a', 0.5, 'alpha', 0)
L3 = Link('d', 0, 'a', 0.5, 'alpha', 0)

% Use the SerialLink method to create the manipulator

simple_robot = SerialLink([L0 L1 L2 L3], 'name', 'Simple manipulator')
%% Generate as an example a point in joint space q

q_input = [   0, ...
              pi/2, ...
             -pi/2, ...
             -pi/2 ];
 
simple_robot.plot(q_input)

%% Inverse kinematics

M = [1 1 1 0 0 0]

x_initial = simple_robot.fkine(q_input);

x_final = eye(4);

x_final(1,4) = 0.6;
x_final(2,4) = 0.6;
x_final(3,4) = 0.6;

trajectory = ctraj(x_initial,x_final,100);

q_guess = q_input

simple_robot.ikine(trajectory(:,:,1), q_guess, M)

for i = 1:size(trajectory,3)

joint_traj(i,:) = simple_robot.ikine(trajectory(:,:,i), q_guess, M)
q_guess = joint_traj(i,:);

end

simple_robot.plot(joint_traj)

simple_robot.fkine(q_guess)
