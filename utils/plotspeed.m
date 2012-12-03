# Octave file to draw PID controller performance chart

grid on
hold on

tS = dlmread("target_speed.dat", "\t");
tx = tS(:, 1);
ty = tS(:, 2);
plot(tx, ty, "color", "r")

oS = dlmread("observed_speed.dat", "\t");
ox = oS(:, 1);
oy = oS(:, 2);
plot(ox, oy, "color", "b")

#plot(regdatasmooth(ox,oy,"d", 3), "color", "g");
