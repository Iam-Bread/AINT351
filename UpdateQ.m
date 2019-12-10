function Q = UpdateQ(Q, state, action, resultingState, reward, alpha,gamma)
Q(state,action) = Q(state,action) + alpha*(reward + gamma*max(Q(resultingState, :)) - Q(state,action));
end