clear
clc

d4lp = load('dataforlocalplanner.mat');

X = d4lp.dataforlocalplanner.X;
Y = d4lp.dataforlocalplanner.Y;
Z = d4lp.dataforlocalplanner.Z;
T = d4lp.dataforlocalplanner.temperarure_soil;

pathX = d4lp.dataforlocalplanner.astar_xpos;
pathY = d4lp.dataforlocalplanner.astar_ypos;

pathZ = zeros(length(pathX),1);
for i = 1:length(pathX)
    pathZ(i) = Z(pathX(i), pathY(i));
end

figure
hold on
terr = surf(X,Y,Z,T);
path = plot3(pathX,pathY,pathZ,'-r','LineWidth',3);

set(terr,'linestyle','none')
light