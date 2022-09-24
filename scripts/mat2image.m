clear
clc

rosshutdown

rosinit('localhost')

load("dataforlocalplanner.mat")

Z = dataforlocalplanner.Z;
T = dataforlocalplanner.temperarure_soil;

elevation = uint16(round(mat2gray(Z)*65535));
temperature = uint16(round(mat2gray(T)*65535));

elevpub = rospublisher("/map/image/elevation","sensor_msgs/Image", "IsLatching", true);
pause(1);
temppub = rospublisher("/map/image/temperature","sensor_msgs/Image", "IsLatching", true);
pause(1);

elev_layer = rosmessage('sensor_msgs/Image');
elev_layer.Encoding = 'mono16';
writeImage(elev_layer, elevation);

temp_layer = rosmessage('sensor_msgs/Image');
temp_layer.Encoding = 'mono16';
writeImage(temp_layer, temperature);

send(elevpub, elev_layer)
send(temppub, temp_layer)

minZ = min(Z,[],'all');
maxZ = max(Z,[],'all');
minT = min(T,[],'all');
maxT = max(T,[],'all');