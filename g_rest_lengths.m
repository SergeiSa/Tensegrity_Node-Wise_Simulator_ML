function out1 = g_rest_lengths(in1)
%G_REST_LENGTHS
%    OUT1 = G_REST_LENGTHS(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    22-Jan-2020 16:24:16

rho1 = in1(1,:);
rho2 = in1(2,:);
rho3 = in1(3,:);
rho4 = in1(4,:);
rho5 = in1(5,:);
rho6 = in1(6,:);
out1 = reshape([0.0,0.0,0.0,0.0,0.0,0.0,rho1,0.0,0.0,0.0,0.0,0.0,rho2,rho4,0.0,0.0,0.0,0.0,rho3,0.0,0.0,0.0,0.0,0.0,0.0,rho5,0.0,0.0,0.0,0.0,0.0,0.0,rho6,0.0,0.0,0.0],[6,6]);
