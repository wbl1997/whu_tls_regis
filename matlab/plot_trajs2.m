function plot_traj2()
folder='/media/pi/BackupPlus/jhuai/results/front_back_snapshots/20240113/data1';
% folder='/media/pi/BackupPlus/jhuai/results/fastlio/20240113/data1';
% folder='/media/pi/BackupPlus/jhuai/results/kissicp/homebrew/20240113/data1';
files = dir([folder, '/**/*poses.txt']);

close all;
for i=1:length(files)
f = [files(i).folder, '/', files(i).name];
g = [files(i).folder, '/fastlio.log'];
[s, out] = check_log(g);
if s == 2
    fprintf("%s OK. Gone outside of TLS? %d\n", files(i).folder, out == 2);
else
    fprintf("Something wrong, s = %d, out = %d.\n", s, out);
end
[p, n, e] = fileparts(f);
[q, m, e] = fileparts(p);
ln = [m, '/', n];
d = readmatrix(f);
figure
plot3(d(:, 2), d(:, 3), d(:, 4)); hold on;
plot3(d(1, 2), d(1, 3), d(1, 4), 'bo');
plot3(d(end, 2), d(end, 3), d(end, 4), 'ks');
axis equal; grid on;
xlabel('x');
ylabel('y');
zlabel('z');
title(ln);
end
end

function [s, out] = check_log(logfile)
fid = fopen(logfile, 'r');
s = 0;
out = 0;
if fid == -1
    return;
end
while ~feof(fid)
    tline = fgetl(fid);
    if contains(tline, 'The lidar pose is out of the TLS trajectory')
        out = out + 1;
    elseif contains(tline, 'Finished processing bag file')
        s = s + 1;
    elseif contains(tline, 'process has finished cleanly')
        s = s + 1;
    elseif contains(tline, 'Distance2 to reference trajectory is too large')
        out = out + 1;
    end
end
fclose(fid);
end
