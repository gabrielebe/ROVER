function [calVector] = AccCalibration(Aport)
%AccCalibration Summary of this function goes here
%   Detailed explanation goes here
     
	calVector.x = 0;
	calVector.y = 0;
	calVector.z = 0;
	
	fprintf(Aport, 'A');
    junk = fscanf(Aport, '%u');
	junk = fscanf(Aport, '%u');
	fprintf(Aport, 'A');
	xSample = fscanf(Aport, '%u');
	ySample = fscanf(Aport, '%u');
	zSample = fscanf(Aport, '%u');

    xVoltage = (xSample * 5.0) / 1024;
    yVoltage = (ySample * 5.0) / 1024;
    zVoltage = (zSample * 5.0) / 1024;
    
    calVector.x = (xVoltage - 1.65) / 0.3;
	calVector.y = (yVoltage - 1.65) / 0.3;
	calVector.z = ((zVoltage - 0.3) - 1.65) / 0.3;
end
