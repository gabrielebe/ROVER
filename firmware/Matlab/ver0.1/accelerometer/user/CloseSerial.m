clc
clear all

if ~isempty(instrfind)
	fclose(instrfind);
	delete(instrfind);
end

disp('Kill serial communication after 3 seconds...');
delay(3000)
close all
clc
