%function [weight1, weight2] = TrainNetwork(P2,samples,training_data,data)

%set intial parameters
armLen = [0.4,0.4];
origin = [0,0];
samples = 5000;

%Generate 1000 uniformly distributed set of angle data points
training_data = pi + -pi*rand(2,samples);
data = pi + -pi*rand(2,samples);

%pass parameters into forward kinematics model
[~,P2] = RevoluteForwardKinematics2D(armLen, training_data, origin);

%Implement 2-layer network
x = [P2(1,:);P2(2,:); ones(1,samples)]; %augment input with ones

%set learning rate
alpha = 0.01;
nodes = 20;


%randomly generate weight matrix
maxlimit = sqrt(2/5);
minlimit = - maxlimit;
w1 = abs((maxlimit - minlimit)* randn(nodes,3) -minlimit);
w2 = abs((maxlimit - minlimit)* randn(2,nodes+1) -minlimit);

for i=1:1000
    for j=1:samples
    %Calculate internal activations of layer 1
    net2 = w1 * x(:,j);
    
    a2 = 1./(1+exp(-net2));
    
    %Augment a2 to account for bias term in W2
    a2_hat = [a2 ; 1];
    
    %Calculate output activations of layer 2
    net3 = w2 * a2_hat;
    
    %set target data
    t = training_data(:,i);
    
    %Calculate output layer delta term
    delta3 = -(t-net3);%.*net3.*(1-net3);
    
    %Back propagation to calculate input layer delta term
    delta2 = (w2'*delta3).*a2_hat.*(1-a2_hat);
    
    %Calculate error gradients of w1 and w2
    dedw1 = delta2 * x(:,i)';
    dedw2 = delta3 * a2_hat';
    dedw1 = dedw1(1:nodes,:);
    
    
    %Update W1 and W2 by moving down towards minimum error
    w1 = w1 -alpha*dedw1;
    w2 = w2 -alpha*dedw2;
    end
end


[~,bP2] = RevoluteForwardKinematics2D(armLen, data, origin);

net3 = useNetwork(bP2,w1,w2,samples);

[~,P2] = RevoluteForwardKinematics2D(armLen, net3, origin);

%plot regenerated
figure;
hold on;
grid on;

plot(P2(1,:),P2(2,:),'m.');
title('10580008:Via Inv and Fwd Model Endpoint');

figure;
hold on;
grid on;
plot(net3(1,:),net3(2,:),'m.');
title('10580008:Inverse Model JointAngle');


weight1 = w1;
weight2 = w2;
%end