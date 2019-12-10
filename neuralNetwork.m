%function [weight1, weight2] = neuralNetwork()
%1.1. Display workspace of revolute arm
%set intial parameters
armLen = [0.4,0.4];
origin = [0,0];
samples = 15000;

%Generate 1000 uniformly distributed set of angle data points
training_data = pi + -pi*rand(2,samples);
data = pi + -pi*rand(2,samples);

%pass parameters into forward kinematics model
[~,P2] = RevoluteForwardKinematics2D(armLen, training_data, origin);

%plot points
figure;
hold on;
grid on;
plot(P2(1,:),P2(2,:),'m.');
plot(origin(1),origin(2),'b*');
title('10580008: ArmEndpoint');
ylabel('Y[m]');
xlabel('X[M]');
legend('EndPoint','Origin');

%the effective workspace of the arm is non symetrical


%1.2 Configurations of a revolute arm
%10 uniformly distributed set of angle data points between 0 – pi
data_arm = pi + -pi*rand(10);
%pass parameters into forward kinematics model
[aP1,aP2] = RevoluteForwardKinematics2D(armLen, data_arm, origin);
%plot points
figure;
grid on;
hold on;

for i =1:10
    plot(0,0,'g*','LineWidth',2); %origin
    plot(aP1(1,i),aP1(2,i),'mo','LineWidth',2); %point 1
    plot(aP2(1,i),aP2(2,i),'co','LineWidth',2); %point 2
    plot([0;aP1(1,i)],[0;aP1(2,i)],'-b'); %line from origin to point 1
    plot([aP1(1,i);aP2(1,i)],[aP1(2,i);aP2(2,i)],'-b'); %line from point 2 to point 1
end

title('10580008: Arm Configuration');
ylabel('Y[m]');
xlabel('X[M]');
hold off;


%Implement 2-layer network
x = [P2(1,:);P2(2,:); ones(1,samples)]; %augment input with ones

%set learning rate
alpha = 0.00001;
nodes = 30;

%randomly generate weight matrix
maxlimit = sqrt(6/5);
minlimit = - maxlimit;
w1 = abs((maxlimit - minlimit)* randn(nodes,3) -minlimit);
w2 = abs((maxlimit - minlimit)* randn(2,nodes+1) -minlimit);

for i=1:10000
    %Calculate internal activations of layer 1
    net2 = w1 * x;
    
    a2 = 1./(1+exp(-net2));
    
    %Augment a2 to account for bias term in W2
    a2_hat = [a2 ; ones(1,samples)];
    
    %Calculate output activations of layer 2
    net3 = w2 * a2_hat;
    
    %set target data
    t = training_data();
    
    %Calculate output layer delta term
    delta3 = -(t-net3);%.*net3.*(1-net3);
    
    %Back propagation to calculate input layer delta term
    delta2 = (w2'*delta3).*a2_hat.*(1-a2_hat);
    
    %Calculate error gradients of w1 and w2
    dedw1 = delta2 * x';
    dedw2 = delta3 * a2_hat';
    dedw1 = dedw1(1:nodes,:);
    
    
    %Update W1 and W2 by moving down towards minimum error
    w1 = w1 -alpha*dedw1;
    w2 = w2 -alpha*dedw2;
    
    %calculate mean square error
    uSquError(:,i) = mean((t-net3).^2,2) ;
    
end

%plot error
figure;
hold on;
grid on;
plot(uSquError(1,:),'c');
plot(uSquError(2,:),'m');

[bP1,bP2] = RevoluteForwardKinematics2D(armLen, data, origin);

net3 = useNetwork(bP2,w1,w2,samples);

[P1,P2] = RevoluteForwardKinematics2D(armLen, net3, origin);

%plot regenerated
figure;
hold on;
grid on;
subplot(2,2,1);
plot(bP2(1,:),bP2(2,:),'b.');
title('10580008:Random Endpoint Data');
subplot(2,2,2);
plot(data(1,:),data(2,:),'b.');
title('10580008:Random Joint Angle Data');
subplot(2,2,3);
plot(P2(1,:),P2(2,:),'m.');
title('10580008:Via Inv and Fwd Model Endpoint');
subplot(2,2,4);
plot(net3(1,:),net3(2,:),'m.');
title('10580008:Inverse Model JointAngle');


weight1 = w1;
weight2 = w2;


   

%end