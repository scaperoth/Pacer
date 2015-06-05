x = load ("x.mat");
y = load ("xd.mat");
z = load ("xdd.mat");

set(gca, "linewidth", 4, "fontsize", 12)

subplot(2,2,1)
plot(x(:,1), 'r', "linewidth", 3);
hold on;
#plot(y(:,1), 'g', "linewidth", 3);
#plot(z(:,1), 'b', "linewidth", 3);
title("X: RGB: Position, Vel, Accel");

subplot(2,2,2)
hold on;
plot(x(:,2), 'r', "linewidth", 3);
#plot(y(:,2), 'g', "linewidth", 3);
#plot(z(:,2), 'b', "linewidth", 3);
title("Y: RGB: Position, Vel, Accel");

subplot(2,2,3)
plot(x(:,3), 'r', "linewidth", 3);
hold on;
#plot(y(:,3), 'g', "linewidth", 3);
#plot(z(:,3), 'b', "linewidth", 3);
title("Z: RGB: Position, Vel, Accel");