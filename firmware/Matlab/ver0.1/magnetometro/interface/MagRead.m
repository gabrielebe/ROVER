function [magVector, heading] = MagRead(Aport)
%MagRead Summary of this function goes here
%   Detailed explanation goes here

    magVector.x = 0;
	magVector.y = 0;
	magVector.z = 0;
	heading = 0;
    
	fprintf(Aport, 'M');
	if Aport.BytesAvailable > 3
		compassData.x = fscanf(Aport, '%u');
		compassData.y = fscanf(Aport, '%u');
		compassData.z = fscanf(Aport, '%u');
	end
	
	rawData.x = compassData.x * 0.73;
	rawData.y = compassData.y * 0.73;
	rawData.z = compassData.z * 0.73;
	calibratedData = CompassCompensate(rawData);
	
	enter = and((~calibratedData.y), (~calibratedData.x));

    if enter
        headingTemp = atan2(calibratedData.y, calibratedData.x);
    end
    
    if headingTemp < 0
        headingTemp = headingTemp + 2 * PI;
    end
    
    if headingTemp > (2 * PI)
        headingTemp = headingTemp - 2 * PI;
    end
    
    heading = headingTemp * (360/(2*PI));
end
