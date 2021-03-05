function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u=0;
error=zeros(2,1);
% FILL IN YOUR CODE HERE
if s_des(1)==0,
kp=0;
kd=0;
else
kp=300;
kd=25;
end
error=s_des-s;
u=params.mass*(kp*error(1)+kd*error(2)+params.gravity);
end

