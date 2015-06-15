function [accVector] = AccRead(Aport, calVector)
%AccRead Summary of this function goes here
%   Detailed explanation goes here

	accVector.x = 0;
	accVector.y = 0;
	accVector.z = 0;
	
	fprintf(Aport, 'A');
	if Aport.BytesAvailable > 3
		xSample = fscanf(Aport, '%u');
		ySample = fscanf(Aport, '%u');
		zSample = fscanf(Aport, '%u');
	end
	
    xAccVoltage = (xSample * 5.0) / 1024;
    yAccVoltage = (ySample * 5.0) / 1024;
    zAccVoltage = (zSample * 5.0) / 1024;
    
	xAccVoltageRaw = (xAccVoltage - 1.65) / 0.3;
    yAccVoltageRaw = (yAccVoltage - 1.65) / 0.3;
    zAccVoltageRaw = (zAccVoltage - 1.65) / 0.3;
	
	xg = xAccVoltageRaw - calVector.x;
	yg = yAccVoltageRaw - calVector.y;
	zg = zAccVoltageRaw - calVector.z;
    
	gravityConstant = 9.80665;
	accVector.x = xg * gravityConstant;
    accVector.y = yg * gravityConstant;
    accVector.z = zg * gravityConstant;
end
