function net3 = useNetwork(point,w1,w2,numPoints)


%Implement 2-layer network
x = [point(1,:);point(2,:); ones(1,numPoints)]; %augment input with ones

%Calculate internal activations of layer 1
net2 = w1 * x;

a2 = 1./(1+exp(-net2));

%Augment a2 to account for bias term in W2
a2_hat = [a2 ; ones(1,numPoints)];

%Calculate output activations of layer 2
net3 = w2 * a2_hat;

end