function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;
kp1=200;kp2=200;kp3=200;
kd1=0.001;kd2=0.001;kd3=40;
kp_phi=400;kp_theta=200;kp_si=80;
kd_phi=0;kd_theta=0;kd_si=0;
r1ddot=des_state.acc(1)+kd1*(des_state.vel(1)-state.vel(1))+kp1*(des_state.pos(1)-state.pos(1));
r2ddot=des_state.acc(2)+kd2*(des_state.vel(2)-state.vel(2))+kp2*(des_state.pos(2)-state.pos(2));
r3ddot=des_state.acc(3)+kd3*(des_state.vel(3)-state.vel(3))+kp3*(des_state.pos(3)-state.pos(3));
phiPlusthetaDes=(1/params.gravity)*[r1ddot -r2ddot;r2ddot r1ddot]*[sin(des_state.yaw);cos(des_state.yaw)];
F=params.mass*(params.gravity+r3ddot);
% Moment
M = zeros(3,1);
M=[kp_phi*(phiPlusthetaDes(1)-state.rot(1))-kd_phi* state.omega(1);
    kp_theta*(phiPlusthetaDes(2)-state.rot(2))-kd_theta* state.omega(2);
    kp_si*(des_state.yaw-state.rot(3))-kd_si*(des_state.yawdot-state.omega(3))];
% =================== Your code ends here ===================

end
