x 			= load("x.mat");
vel 		= load("xd.mat");
accel 		= load("xdd.mat");
realx 		= load("realx.mat");
realvel 	= load("realxd.mat");
realaccel 	= load("realxdd.mat");

set(gca, "linewidth", 4, "fontsize", 12)

subplot(2,3,1)
hold on;
plot(realx(:,1), 'g', "linewidth", 3);
plot(x(:,1), 'r', "linewidth", 3);
title("position X");
hold off;

subplot(2,3,2)
hold on;
plot(realx(:,2), 'g', "linewidth", 3);
plot(x(:,2), 'r', "linewidth", 3);
title("Position Y");
hold off;

subplot(2,3,3)
hold on;
plot(realx(:,3), 'g', "linewidth", 3);
plot(x(:,3), 'r', "linewidth", 3);
title("Position Z");
hold off;

subplot(2,3,4)
hold on;
plot(realvel(:,1), 'g', "linewidth", 3);
plot(vel(:,1), 'r', "linewidth", 3);
title("Velocity X");
hold off;

subplot(2,3,5)
hold on;
plot(realvel(:,2), 'g', "linewidth", 3);
plot(vel(:,2), 'r', "linewidth", 3);
title("Velocity Y");
hold off;

subplot(2,3,6)
hold on;
plot(realvel(:,3), 'g', "linewidth", 3);
plot(vel(:,3), 'r', "linewidth", 3);
title("Velocity Z");
hold off;

hold off;
title("Z");
