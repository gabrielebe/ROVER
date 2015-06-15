plotTitleX.acc = 'Acceleration on x axis: a_x(t)';
plotTitleX.vel = 'Velocity on x axis: v_x(t)';
plotTitleX.spa = 'Space on x axis: s_x(t)';
plotTitleY.acc = 'Acceleration on y axis: a_y(t)';
plotTitleY.vel = 'Velocity on y axis: v_y(t)';
plotTitleY.spa = 'Space on y axis: s_y(t)';
plotTitleZ.acc = 'Acceleration on z axis: a_z(t)';
plotTitleZ.vel = 'Velocity on z axis: v_z(t)';
plotTitleZ.spa = 'Space on z axis: s_z(t)';
xLabel = 'Time [s]';
yLabel.acc = 'Acceleration [m/s^2]';
yLabel.vel = 'Velocity [m/s]';
yLabel.spa = 'Space [m]';
yAssMin = -35;
yAssMax = 35;

figure(1);

subplot(2, 2, 1)
xPlot.spa = plot(nan, nan);
title(plotTitleX.spa, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 10);
ylabel(yLabel.spa, 'FontSize', 10);
grid on;

subplot(2, 2, 2)
xPlot.vel = plot(nan, nan);
title(plotTitleX.vel, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 10);
ylabel(yLabel.vel, 'FontSize', 10);
grid on;

subplot(2, 2, 3)
xPlot.acc = plot(nan, nan);
title(plotTitleX.acc, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 10);
ylabel(yLabel.acc, 'FontSize', 10);
grid on;

figure(2);

subplot(2, 2, 1)
yPlot.spa = plot(nan, nan);
title(plotTitleY.spa, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 10);
ylabel(yLabel.spa, 'FontSize', 10);
grid on;

subplot(2, 2, 2)
yPlot.vel = plot(nan, nan);
title(plotTitleY.vel, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 10);
ylabel(yLabel.vel, 'FontSize', 10);
grid on;

subplot(2, 2, 3)
yPlot.acc = plot(nan, nan);
title(plotTitleY.acc, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 10);
ylabel(yLabel.acc, 'FontSize', 10);
grid on;

figure(3);

subplot(2, 2, 1)
zPlot.spa = plot(nan, nan);
title(plotTitleZ.spa, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 10);
ylabel(yLabel.spa, 'FontSize', 10);
grid on;

subplot(2, 2, 2)
zPlot.vel = plot(nan, nan);
title(plotTitleZ.vel, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 10);
ylabel(yLabel.vel, 'FontSize', 10);
grid on;

subplot(2, 2, 3)
zPlot.acc = plot(nan, nan);
title(plotTitleZ.acc, 'FontSize', 15);
xlabel(xLabel, 'FontSize', 10);
ylabel(yLabel.acc, 'FontSize', 10);
grid on;


comPort = 'COM6';
Aport = SetupSerial(comPort);
calVector = AccCalibration(Aport);

x.spa = 0;
x.vel = 0;
x.acc = 0;
y.spa = 0;
y.vel = 0;
y.acc = 0;
z.spa = 0;
z.vel = 0;
z.acc = 0;
spaVector.x = 0;
spaVector.y = 0;
spaVector.z = 0;
velVector.x = 0;
velVector.y = 0;
velVector.z = 0;
time = 0;
i = 0;
dt = 0.1;

while(ishandle(xPlot.spa))
	time = get(xPlot.spa, 'XData');
	x.spa = get(xPlot.spa, 'YData');
	x.vel = get(xPlot.vel, 'YData');
	x.acc = get(xPlot.acc, 'YData');
	y.spa = get(yPlot.spa, 'YData');
	y.vel = get(yPlot.vel, 'YData');
	y.acc = get(yPlot.acc, 'YData');
	z.spa = get(zPlot.spa, 'YData');
	z.vel = get(zPlot.vel, 'YData');
	z.acc = get(zPlot.acc, 'YData');
	
	accVector = AccRead(Aport, calVector);
	time = [time i];
	velVector.x = velVector.x + accVector.x * dt;
	spaVector.x = spaVector.x + velVector.x * dt;
	velVector.y = velVector.y + accVector.y * dt;
	spaVector.y = spaVector.y + velVector.y * dt;
	velVector.z = velVector.z + accVector.z * dt;
	spaVector.z = spaVector.z + velVector.z * dt;
	x.spa = [x.spa spaVector.x];
	x.vel = [x.vel velVector.x];
	x.acc = [x.acc accVector.x];
	y.spa = [y.spa spaVector.y];
	y.vel = [y.vel velVector.y];
	y.acc = [y.acc accVector.y];
	z.spa = [z.spa spaVector.z];
	z.vel = [z.vel velVector.z];
	z.acc = [z.acc accVector.z];
	
	set(xPlot.spa, 'XData', time, 'YData', x.spa);
	set(xPlot.vel, 'XData', time, 'YData', x.vel);
	set(xPlot.acc, 'XData', time, 'YData', x.acc);
	set(yPlot.spa, 'XData', time, 'YData', y.spa);
	set(yPlot.vel, 'XData', time, 'YData', y.vel);
	set(yPlot.acc, 'XData', time, 'YData', y.acc);
	set(zPlot.spa, 'XData', time, 'YData', z.spa);
	set(zPlot.vel, 'XData', time, 'YData', z.vel);
	set(zPlot.acc, 'XData', time, 'YData', z.acc);
	
	drawnow
	pause(dt);
	i = i + dt;
end

CloseSerial
