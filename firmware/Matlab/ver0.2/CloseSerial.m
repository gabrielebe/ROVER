clc
clear all

if ~isempty(instrfind)
	fclose(instrfind);
	delete(instrfind);
end

disp('Kill serial communication after 3 seconds...');
pause(3)
close all
clc
