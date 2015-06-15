comPort = 'COM6';
Aport = SetupSerial(comPort);
calVector = AccCalibration(Aport);

measure = 100; 
for i=1:measure
	accVector = AccRead(Aport, calVector)
    tiltData = AccTilt(Aport, calVector)
    pause(0.1);
end

CloseSerial
