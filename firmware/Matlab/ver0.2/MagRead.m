function [heading] = MagRead(Aport)
%MagRead Summary of this function goes here
%   Detailed explanation goes here

    compassData = zeros(1, 3);
    heading = 0; 
	fprintf(Aport, 'M');
	compassData(1) = fscanf(Aport, '%f') * 0.73;
	compassData(2) = fscanf(Aport, '%f') * 0.73;
	compassData(3) = fscanf(Aport, '%f') * 0.73;
	calibratedData = CompassCompensate(compassData);
    calibratedData = compassData;

    if and(calibratedData(2), calibratedData(1))
        headingTemp = atan2(calibratedData(2), calibratedData(1));
    end
    
    if headingTemp < 0
        headingTemp = headingTemp + 2*pi;
    end
    
    if headingTemp > (2*pi)
        headingTemp = headingTemp - 2*pi;
    end
    
    heading = headingTemp * (360/(2*pi));
end
