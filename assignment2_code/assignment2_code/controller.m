function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
u1 = 0;
u2 = 0;
kp_y=18;kd_y=5;
kp=300;kd=25;
kp_ang=800;kd_ang=10;
error_p=des_state.pos-state.pos;
error_d=des_state.vel-state.vel;
u1=params.mass*( des_state.acc(2)+kp*error_p(2)+kd*error_d(2)+params.gravity);
des_ang=(-1/params.gravity)*(des_state.acc(1)+kp_y*error_p(1)+kd_y*error_d(1));
u2=params.Ixx*(kp_ang*(des_ang-state.rot)+kd_ang*(-state.omega));
% FILL IN YOUR CODE HERE

end

