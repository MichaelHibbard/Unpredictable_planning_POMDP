function oF = createObsFunctionNoisyMDP(r,c,absorb,target)

%%%%%%%%%%%%%%%%%%%%%%%
%    ____ ____ ____    %
%   |  i |  i |  i |   %
%   |__p_|__p_|_ p_|   %
%   |  i |  c |  i |   %
%   |__p_|__p_|__p_|   %
%   |  i |  i |  i |   %
%   |__p_|__p_|__p_|   %
%                      %
%%%%%%%%%%%%%%%%%%%%%%%%

% Additionally, assume that if the agent is along a wall / edge / corner,
% the "incorrect" probability of thinking it is in a nonexistent state is
% added to the correct probability.

% cP = correct probability of observation
% iP = incorrect probability of obsrvation
% oF = observation function
% c = column
% r = r

% Probability of correctly observing the system state / prob. of
% incorrectly viewing
cP = 0.9;
iP = (1 - cP)/8;

% number of states in the MDP
numStates = r*c;

% The state space is the observations.
oF = zeros(numStates,numStates);

for state = 1:numStates
    
    % add to 'correct' observation function
    oF(state,state) = cP;
    
    % Get the x and y locations - recall matlab's indexing starts at one!
    if mod(state,c) ~= 0
        x = floor(state/r) + 1;
        y = mod(state,c);
    else
        x = floor(state/r);
        y = c;
    end
    
    % If the state is in the upper left corner
    if x == 1 && y == 1
        oF(state,state) = oF(state,state) + 5*iP;
        % right center
        oF(state,c*(x-1)+y+1) = iP;
        % below right corner
        oF(state,c*(x)+y+1) = iP;
        % bottom center
        oF(state,c*(x)+y) = iP;
        %
        % If the state is in the upper right corner
    elseif x == 1 && y == c
        oF(state,state) = oF(state,state) + 5*iP;
        % bottom center
        oF(state,c*(x)+y) = iP;
        % below left corner
        oF(state,c*(x)+y-1) = iP;
        % left center
        oF(state,c*(x-1)+y-1) = iP;
        %
        % If the state is in the lower left corner
    elseif x == r && y == 1
        oF(state,state) = oF(state,state) + 5*iP;
        % above center
        oF(state,c*(x-2)+y) = iP;
        % above right corner
        oF(state,c*(x-2)+y+1) = iP;
        % right center
        oF(state,c*(x-1)+y+1) = iP;
        %
        % If the state is in the lower right corner
    elseif x == r && y == c
        oF(state,state) = oF(state,state) + 5*iP;
        % above left corner
        oF(state,c*(x-2)+y-1) = iP;
        % above center
        oF(state,c*(x-2)+y) = iP;
        % left center
        oF(state,c*(x-1)+y-1) = iP;
        %
        % If the state is along the top edge (non-corner)
    elseif x == 1
        oF(state,state) = oF(state,state) + 3*iP;
        % right center
        oF(state,c*(x-1)+y+1) = iP;
        % below right corner
        oF(state,c*(x)+y+1) = iP;
        % bottom center
        oF(state,c*(x)+y) = iP;
        % below left corner
        oF(state,c*(x)+y-1) = iP;
        % left center
        oF(state,c*(x-1)+y-1) = iP;
        %
        % If the state is along the bottom edge (non-corner)
    elseif x == r
        oF(state,state) = oF(state,state) + 3*iP;
        % above left corner
        oF(state,c*(x-2)+y-1) = iP;
        % above center
        oF(state,c*(x-2)+y) = iP;
        % above right corner
        oF(state,c*(x-2)+y+1) = iP;
        % right center
        oF(state,c*(x-1)+y+1) = iP;
        % left center
        oF(state,c*(x-1)+y-1) = iP;
        %
        % If the state is along the left edge (non-corner)
    elseif y == 1
        oF(state,state) = oF(state,state) + 3*iP;
        % above center
        oF(state,c*(x-2)+y) = iP;
        % above right corner
        oF(state,c*(x-2)+y+1) = iP;
        % right center
        oF(state,c*(x-1)+y+1) = iP;
        % below right corner
        oF(state,c*(x)+y+1) = iP;
        % bottom center
        oF(state,c*(x)+y) = iP;
        %
        % If the state is along the right edge (non-corner)
    elseif y == c
        oF(state,state) = oF(state,state) + 3*iP;
        % above left corner
        oF(state,c*(x-2)+y-1) = iP;
        % above center
        oF(state,c*(x-2)+y) = iP;
        % bottom center
        oF(state,c*(x)+y) = iP;
        % below left corner
        oF(state,c*(x)+y-1) = iP;
        % left center
        oF(state,c*(x-1)+y-1) = iP;
        %
        % Just a regular old state
    else
        % above left corner
        oF(state,c*(x-2)+y-1) = iP;
        % above center
        oF(state,c*(x-2)+y) = iP;
        % above right corner
        oF(state,c*(x-2)+y+1) = iP;
        % right center
        oF(state,c*(x-1)+y+1) = iP;
        % below right corner
        oF(state,c*(x)+y+1) = iP;
        % bottom center
        oF(state,c*(x)+y) = iP;
        % below left corner
        oF(state,c*(x)+y-1) = iP;
        % left center
        oF(state,c*(x-1)+y-1) = iP;
    end
end

%end