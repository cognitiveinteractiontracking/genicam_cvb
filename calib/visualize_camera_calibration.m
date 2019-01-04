clear all;
close all;

calibrationFile = 'jai_5000_m.yaml';

YamlStruct = ReadYaml(calibrationFile);

% cameraParams = estimateCameraParameters