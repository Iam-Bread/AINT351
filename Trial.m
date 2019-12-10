function [Q steps] = Trial(alpha, gamma, episodes, explorationRate,maze)
% trial function that runs 'trials' episodes
% initialize the Q-table once, 11 states and 4 actions
maze = maze.InitQTable;
Q = maze.QValues;

% trial function that runs episodes.
for tidx = 1:episodes
% run an episode and record steps
[Q steps(tidx)] = Episode(Q, alpha, gamma,explorationRate,maze);
end

end
