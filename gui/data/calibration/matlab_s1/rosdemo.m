clear all
close all
clc
% rosinit('http://localhost:11311')

% Load the required MATLAB toolboxes
import vision.*

% Specify the ArUco marker dictionary and the marker size (in meters)
markerDict = '4x4_1000';
markerSize = 0.02; % 20 mm = 0.02 meters

numMarkers = 12;
markerSize = 50; % in pixels
markerFamily = "DICT_5X5_1000";
ids = 1:numMarkers;
imgs = generateArucoMarker(markerFamily,ids,markerSize);

tiledlayout(3,4,TileSpacing="compact")
for i = 1:numMarkers
    nexttile
    imshow(imgs(:,:,i))
    title("ID = " + ids(i))
end