function [z_deep,tip_trajectory_global] = coordinate_transformation(xt,yt,rotation_C,yaw)
global dOC H B0 b
%用于将挖掘轨迹和物料面转化成全局坐标
tip_trajectory_global_x = rotation_C(1)-(dOC+H*tan(B0))*sin(yaw)-xt*sin(yaw);
tip_trajectory_global_y = rotation_C(2)+(dOC+H*tan(B0))*cos(yaw)+xt*cos(yaw);
tip_trajectory_global_z = rotation_C(3)-rotation_C(3)+yt;

cc1111 = ones(1,length(tip_trajectory_global_z));
x5 = tip_trajectory_global_x.^5;
x4y = tip_trajectory_global_x.^4.*tip_trajectory_global_y;
x3y2 = tip_trajectory_global_x.^3.*tip_trajectory_global_y.^2;
x2y3 = tip_trajectory_global_x.^2.*tip_trajectory_global_y.^3;
xy4 = tip_trajectory_global_x.*tip_trajectory_global_y.^4;

x4 = tip_trajectory_global_x.^4;
x3y = tip_trajectory_global_x.^3.*tip_trajectory_global_y;
x2y2 = tip_trajectory_global_x.^2.*tip_trajectory_global_y.^2;
xy3 = tip_trajectory_global_x.*tip_trajectory_global_y.^3;
y4 = tip_trajectory_global_y.^4;

x3 = tip_trajectory_global_x.^3;
x2y = tip_trajectory_global_x.^2.*tip_trajectory_global_y;
xy2 = tip_trajectory_global_x.*tip_trajectory_global_y.^2;
y3 = tip_trajectory_global_y.^3;

x2 = tip_trajectory_global_x.^2;
xy = tip_trajectory_global_x.*tip_trajectory_global_y;
y2 = tip_trajectory_global_y.^2;
x = tip_trajectory_global_x;
y = tip_trajectory_global_y;

z_wuliao = b(1)*x5+b(2)*x4y+b(3)*x3y2+b(4)*x2y3+b(5)*xy4+...
    b(6)*x4+b(7)*x3y+b(8)*x2y2+b(9)*xy3+b(10)*y4+...
    b(11)*x3+b(12)*x2y+b(13)*xy2+b(14)*y3+...
    b(15)*x2+b(16)*xy+b(17)*y2+...
    b(18)*x+b(19)*y+b(20)*cc1111;

z_deep=z_wuliao-tip_trajectory_global_z;
tip_trajectory_global_x=tip_trajectory_global_x';
tip_trajectory_global_y=tip_trajectory_global_y';
tip_trajectory_global_z=tip_trajectory_global_z';
tip_trajectory_global=[tip_trajectory_global_x tip_trajectory_global_y tip_trajectory_global_z];
end

