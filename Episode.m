function [Q steps] = Episode(Q, alpha, gamma, explorationRate,maze)
% implements a Q-learning episode

% initialize state to a random starting state at the start of each episode
state = maze.RandomStatingState();

% loop that runs until the goal state is reached
running=1;
steps=0;
while(running==1)
% % make greedy action selection
action = GreedyActionSelection(state, explorationRate, Q);

% get the next state due to that action
nextState = maze.tm(state, action);

% get the reward from the action on the current state
reward = maze.RewardFunction(state, action);

% update the Q- table
Q = UpdateQ(Q, state, action, nextState, reward, alpha,gamma);
% update steps
steps = steps +1;
% update the state
state = nextState;

% termination if reaches goal state
if(reward == 10)
    running = 0;
end

end