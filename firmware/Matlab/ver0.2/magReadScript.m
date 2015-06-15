comPort = 'COM6';
Aport = SetupSerial(comPort);
fprintf(Aport, 'M');
junk = fscanf(Aport, '%u');
junk = fscanf(Aport, '%u');

measure = 100; 
for i=1:measure
	heading = MagRead(Aport)
    pause(0.1);
end

CloseSerial
