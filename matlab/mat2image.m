clear
clc

rosinit('localhost', 11311)

load("dataforlocalplanner.mat")

Z = dataforlocalplanner.Z;
T = dataforlocalplanner.temperarure_soil;

elevation = uint16(round(mat2gray(Z)*65535));

imagepub = rospublisher("/map/image/elevation","sensor_msgs/Image");
pause(2);

elevimg = rosmessage('sensor_msgs/Image');
elevimg.Encoding = 'mono16';
writeImage(elevimg, elevation);

send(imagepub, elevimg)
pause(2)