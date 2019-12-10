close all;
clear all;

%set intial parameters
armLen = [0.4,0.4];
origin = [0,0];
samples = 50000;

%limits of wanted area
xupper = -0.1;
xlower = -0.5;

yupper = 0.5;
ylower = 0.1;

%Generate uniformly distributed set of angle data points
data = pi + -pi*rand(2,samples);

%pass parameters into forward kinematics model
[~,DP2] = RevoluteForwardKinematics2D(armLen, data, origin);


idx = 1;
%iterate over every sample
for i=1:samples
    if(xlower <DP2(1,i) && DP2(1,i)< xupper)%if within x bounds
        if(ylower <DP2(2,i) && DP2(2,i)< yupper)%within y bounds
            dataAngles(:,idx) = data(:,i);%add to new dataset
            idx= idx+1;%increment index
        end
    end
end

[~,SP2] = RevoluteForwardKinematics2D(armLen, dataAngles, origin);

figure;
hold on;
grid on;

subplot(2,2,1);
plot(SP2(1,:),SP2(2,:),'b.');
title('10580008:Random Endpoint Data');

subplot(2,2,2);
plot(dataAngles(1,:),dataAngles(2,:),'b.');
title('10580008:Random Joint Angle Data');


newSamples = size(dataAngles,2);
[weight1, weight2] = TrainNetwork(SP2,newSamples,dataAngles);
