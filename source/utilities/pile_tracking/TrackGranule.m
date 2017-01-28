%% Load a Bispheres Made Pile 
granule_bisphere = importdata('settling_granule_bisphere.dat');
%%
% ChTime,Nbodies,Ncontacts,
% % pos.x,pos.y,pos.z,
% % % vel.x,vel.y,vel.z
figure,plot(granule_bisphere(:,1),granule_bisphere(:,7),'r',...
granule_bisphere(:,1),granule_bisphere(:,8),'g',...
granule_bisphere(:,1),granule_bisphere(:,9),'b') 
legend('Vx','Vy','Vz','Location','Best')
title('Bisphere Pile Modeling, 13114 bodies, tSIM=3053s')
xlabel('World Time [s]')
ylabel('Velocity Components [m/s]')
grid on

%%
% Granule Trajectory
x = granule_bisphere(:,4);
y = granule_bisphere(:,5);
z = granule_bisphere(:,6);
plot3(x,y,z,'r');
zlim([0.0,0.5])
xlabel('x pos [m]')
ylabel('y pos [m]')
zlabel('z pos [m]')
title('Heap Top particle trajectory')
grid on
% IT TAKES A LONG TIME
% % %if you want to animate it
% % for i=1:length(x)
% %   plot3(x(i),y(i),z(i),'*r');
% %   hold on;
% %   pause(0.01);
% % end

%% Load a Cones Made Pile 
%granule_cone = importdata('settling_granule_cone_1e-4.dat');
granule_cone = importdata('settling_granule_bispher-cone_1e-5.dat');
%% 
% ChTime,Nbodies,Ncontacts,
% % pos.x,pos.y,pos.z,
% % % vel.x,vel.y,vel.z
figure,plot(granule_cone(:,1),granule_cone(:,7),'r',...
granule_cone(:,1),granule_cone(:,8),'g',...
granule_cone(:,1),granule_cone(:,9),'b') 
legend('Vx','Vy','Vz','Location','Best')
% % title('Cone Pile Modeling, 13114 bodies, itself stopped after .163s real')
title('Cone Pile Modeling, 13114 bodies, by hand stopped after .195s real')
xlabel('World Time [s]')
ylabel('Velocity Components [m/s]')
grid on

