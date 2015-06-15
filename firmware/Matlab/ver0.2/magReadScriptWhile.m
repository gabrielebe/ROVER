comPort = 'COM6';
Aport = SetupSerial(comPort);
fprintf(Aport, 'M');
junk = fscanf(Aport, '%u');
junk = fscanf(Aport, '%u');
 
while 1
	heading = MagRead(Aport)
    pause(0.1);
end
