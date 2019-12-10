function action = GreedyActionSelection(state, e, Q)
if(rand < e)
    %random action if e
    action = randi([1 4],1);
else
    %find action with highest q value
    r = Q(state, :);
    [~, action] = max(r);
end
end