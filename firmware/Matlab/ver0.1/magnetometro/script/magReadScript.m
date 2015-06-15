comPort = 'COM6';
Aport = SetupSerial(comPort);
fprintf(Aport, 'M');
junk = fscanf(Aport, '%u');
junk = fscanf(Aport, '%u');

measure = 100; 
for i=1:measure
	[magVector, heading] = MagRead(Aport)
end

CloseSerial
