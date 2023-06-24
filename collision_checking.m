%% RBE 550 : Transmission Assignment
% Upasana Mahanti

clc;
clear all;
close all;


box1 = collisionBox(0.1,2,1);
T = trvec2tform([10 10 10]);
box1.Pose = T;

box2 = collisionBox(0.1,0.5,1);
T = trvec2tform([10 10.75 11]);
box2.Pose = T;
box3 = collisionBox(0.1,0.5,1);
T = trvec2tform([10 9.25 11]);
box3.Pose = T;
box4 = collisionBox(0.1,0.5,1);
T = trvec2tform([10 10.75 9]);
box4.Pose = T;
box5 = collisionBox(0.1,0.5,1);
T = trvec2tform([10 9.255 9]);
box5.Pose = T;
box6 = collisionBox(0.1,2,0.4);
T = trvec2tform([10 10 8.3]);
box6.Pose = T;
box7 = collisionBox(0.1,2,0.4);
T = trvec2tform([10 10 11.7]);
box7.Pose = T;
box8 = collisionBox(0.1,2,1);
T = trvec2tform([13 10 10]);
box8.Pose = T;
box9 = collisionBox(0.1,0.5,1);
T = trvec2tform([13 10.75 11]);
box9.Pose = T;
box10 = collisionBox(0.1,0.5,1);
T = trvec2tform([13 9.25 11]);
box10.Pose = T;
box11 = collisionBox(0.1,0.5,1);
T = trvec2tform([13 10.75 9]);
box11.Pose = T;
box12 = collisionBox(0.1,0.5,1);
T = trvec2tform([13 9.25 9]);
box12.Pose = T;
box13 = collisionBox(0.1,2,0.4);
T = trvec2tform([13 10 8.3]);
box13.Pose = T;
box14 = collisionBox(0.1,2,0.4);
T = trvec2tform([13 10 11.7]);
box14.Pose = T;
bottom_plate = collisionBox(3,2,0.1);
T = trvec2tform([11.5 10 8.2]);
bottom_plate.Pose = T;
side_plate1 = collisionBox(3,0.1,3.7);
T = trvec2tform([11.5 11 10]);
side_plate1.Pose = T;
side_plate2 = collisionBox(3,0.1,3.7);
T = trvec2tform([11.5 9 10]);
side_plate2.Pose = T;

main_shaft = collisionCylinder(0.2,3.4);
matZ = axang2tform([0 1 0 pi/2]);
matZ(1,4) = 11.2;
matZ(2,4) = 10;
matZ(3,4) = 11;
main_shaft.Pose = matZ;
main_shaft_bearings = collisionCylinder(0.5,2);
matZ = axang2tform([0 1 0 pi/2]);
matZ(1,4) = 11.4;
matZ(2,4) = 10;
matZ(3,4) = 11;
main_shaft_bearings.Pose = matZ;
bottom_shaft = collisionCylinder(0.2,4);
matZ = axang2tform([0 1 0 pi/2]);
matZ(1,4) = 11.5;
matZ(2,4) = 10;
matZ(3,4) = 9;
bottom_shaft.Pose = matZ;



box_obstacles = [box1 box2 box3 box4 box5 box6 box7 box8 box9 box10 box11 box12 box13 box14 side_plate1 side_plate2];

cylinder_obstacles = bottom_shaft;
hold on
figure(1)
axis on

[~,patchObj] = show(box1);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box2);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box3);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box4);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box5);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box6);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box7);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box8);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box9);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box10);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box11);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box12);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box13);
patchObj.FaceColor = [0 0 0];
[~,patchObj] = show(box14);
patchObj.FaceColor = [0 0 0];

show(bottom_plate)
show(side_plate1)


[~,patchObj] = show(bottom_shaft);
patchObj.FaceColor = [0 0 1];

[~,patchObj] = show(main_shaft);
patchObj.FaceColor = [0 0 1];

[~,patchObj] = show(main_shaft_bearings);
patchObj.FaceColor = [0 0 1];

xlim([0 20])
ylim([0 20])
zlim([0 20])
xlabel('X')
ylabel('Y')
zlabel('Z')

% Set the title and the view angle
% title('3D Environment with Obstacles', 'FontSize', 14);
view(45,45); % Custom view angle