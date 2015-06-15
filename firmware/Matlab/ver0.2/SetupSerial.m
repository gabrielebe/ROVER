function Aport = SetupSerial(comPort)
%SetupSerial Summary of this function goes here
%   Detailed explanation goes here

    Aport = serial(comPort);
    set(Aport, 'DataBits', 8);
    set(Aport, 'StopBits', 1);
    set(Aport, 'BaudRate', 9600);
    set(Aport, 'Parity','none');
    fopen(Aport);

    a = 'b';
    while(a ~= 'a')
        a = fread(Aport, 1, 'uchar');
    end

    fprintf(Aport, '%c', 'a');
    disp('Serial communication initialized!')
end
