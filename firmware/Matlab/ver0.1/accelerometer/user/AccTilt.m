function [tiltData] = AccTilt(Aport, calVector)
%AccTilt Summary of this function goes here
%   Detailed explanation goes here

    tiltData.roll = 0;
    tiltData.pitch = 0;

    accData = AccRead(Aport, calVector);
    gravityConstant = 9.80665;
    gx = accData.x / gravityConstant;
    gy = accData.y / gravityConstant;
    gz = accData.z / gravityConstant;

    tiltData.roll  = atan2(gy, gz) * (360/(2*PI));
    tiltData.pitch = atan2(gx, sqrt(-(gy^2) + gz^2)) * (360/(2*PI));   
end
