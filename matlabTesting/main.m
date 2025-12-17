clc; clear; close all;

targetPosition = [0.1725 0.4805 0.0];
followTime = 0.5;
frequency = 125;
transformW2R = [-0.3840, -0.9233, 0.0005,  0.7021;
                 0.9233, -0.3840, 0.0018, -0.7111;
                -0.0015,  0.0012, 1.0000, -0.0334;
                 0.0000,  0.0000, 0.0000,  1.0000];
fileNumber = 321;

input = [targetPosition, followTime, frequency, transformW2R(1,1:4), transformW2R(2,1:4), transformW2R(3,1:4), transformW2R(4,1:4), fileNumber];

output = throwCalc(input);

status = output(1);

if status < 9000000

    qStart = output(2:7);

    output = output(8:end);
    qd = reshape(output, 6, []);

    releaseIndex = size(qd, 2) - round(followTime * frequency);

end