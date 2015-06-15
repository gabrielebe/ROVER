function [calibratedData] = CompassCompensate(compassData)
%CompassCompensate Summary of this function goes here
%   Detailed explanation goes here

    calibratedData = zeros(1, 3);
    calibrationMatrix = [1.019, 0.001, 0.003;...
                         0.032, 0.97, -0.048;...
                         -0.02, -0.017, 1.274];
    
    bias = [-34.311, -212.987, -122.717];
    compassData = compassData - bias;
    
    for i=1:3
        for j=1:3
            calibratedData(i) = calibratedData(i) + calibrationMatrix(i,j) * compassData(1);
        end
    end
end
