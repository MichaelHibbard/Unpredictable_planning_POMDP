%%% This function generates a grid world environment with given number of
%%% rows and columns. It start counting the rows starting from the top and
%%% the columns from the left. Actions can be chosen either 4 or 5, where
%%% the order is [left,right,up,down,stay]. Wall input is the location of
%%% walls such that [[row_1,col_1]; [row_2,col_2]]. Absorb input is the
%%% location of absorbing states such that [[row_1,col_1]; [row_2,col_2]].

function pTot=grid_world2(row,column,act,wall,absorb,prob,eps,tHorizon)

numStatesNominal = row*column;
P=zeros(numStatesNominal,numStatesNominal,act);
ProbIntended = prob-eps/3;
ProbSlipSide = (1-prob)/2-eps/3;
ProbSlipBack = eps;

% WORKS FOR GRIDS OF SIZE > 3x3! (otherwise mod() terms get screwed up)
for state=1:numStatesNominal
    for nextState=1:numStatesNominal
        % If the next state is to the right and current state not on right edge
        if nextState == state + 1 && mod(state, column) ~= 0
            P(state, nextState, 1) = ProbSlipBack;
            P(state, nextState, 2) = ProbIntended;
            P(state, nextState, 3) = ProbSlipSide;
            P(state, nextState, 4) = ProbSlipSide;
            % If the next state is to the right and current state is on
            % right edge
        elseif nextState == state + 1 && mod(state, column) == 0
            P(state, nextState, 1) = 0;
            P(state, nextState, 2) = 0;
            P(state, nextState, 3) = 0;
            P(state, nextState, 4) = 0;
            % If the next state is to the left and current state not on
            % left edge
        elseif nextState == state - 1 && mod((state-1), column) ~= 0
            P(state, nextState, 1) = ProbIntended;
            P(state, nextState, 2) = ProbSlipBack;
            P(state, nextState, 3) = ProbSlipSide;
            P(state, nextState, 4) = ProbSlipSide;
            % If the next state is to the left and current state is on left
            % edge
        elseif nextState == state - 1 && mod(state, column) == 0
            P(state, nextState, 1) = 0;
            P(state, nextState, 2) = 0;
            P(state, nextState, 3) = 0;
            P(state, nextState, 4) = 0;
            % If the next state is below and current state not in bottom
            % row
        elseif nextState == (state + column) && (state + column) <= numStatesNominal
            P(state, nextState, 1) = ProbSlipSide;
            P(state, nextState, 2) = ProbSlipSide;
            P(state, nextState, 3) = ProbSlipBack;
            P(state, nextState, 4) = ProbIntended;
            % If the next state is below but current state in bottom row
        elseif nextState == state + column && state + column > numStatesNominal
            P(state, nextState, 1) = 0;
            P(state, nextState, 2) = 0;
            P(state, nextState, 3) = 0;
            P(state, nextState, 4) = 0;
            % If the next state is above and current state not in top row
        elseif nextState == (state - column) && (state - column) >= 0
            P(state, nextState, 1) = ProbSlipSide;
            P(state, nextState, 2) = ProbSlipSide;
            P(state, nextState, 3) = ProbIntended;
            P(state, nextState, 4) = ProbSlipBack;
            % If the next state is above and current state in top row
        elseif nextState == (state + column) && (state - column) < 0
            P(state, nextState, 1) = 0;
            P(state, nextState, 2) = 0;
            P(state, nextState, 3) = 0;
            P(state, nextState, 4) = 0;
        end
    end
    % Whatever probability has not been allocated goes to remaining
    % in same state.
    P(state, state, 1) = 1 - sum(P(state, :, 1));
    P(state, state, 2) = 1 - sum(P(state, :, 2));
    P(state, state, 3) = 1 - sum(P(state, :, 3));
    P(state, state, 4) = 1 - sum(P(state, :, 4));
end


% Change transition probabilities for absorbing states
for state=1:size(absorb,1)
    x=absorb(state,:);
    P(column*(x(1)-1)+x(2),:,:)=zeros(row*column,act);
    P(column*(x(1)-1)+x(2),column*(x(1)-1)+x(2),:)=ones(1,act);
    %P(row*(x(1)-1)+x(2),row*(x(1)-1)+x(2),1)=1;
end

% Now, use the transition matrix to construct for the product with the time
% horizon
numStates = row*column*tHorizon;
pTot = zeros(numStates,numStates,act);
for time = 1:tHorizon
    if time ~= tHorizon
        pTot((time-1)*numStatesNominal+1:time*numStatesNominal,time*numStatesNominal+1:(time+1)*numStatesNominal,:)...
            = P(:,:,:);
    else
        for action = 1:act
            pTot((time-1)*numStatesNominal+1:time*numStatesNominal,(time-1)*numStatesNominal+1:time*numStatesNominal,action)...
                = eye(numStatesNominal,numStatesNominal);
        end
    end
end

% Don't worry about walls for now
% collision with wall
%     for state=1:size(wall,1)
%         x=wall(state,:);
%         P(column*(x(1)-1)+x(2),:,:)=zeros(row*column,act);
%         P(column*(x(1)-1)+x(2),column*(x(1)-1)+x(2),:)=ones(1,act);
%         for k=1:act
%             precessor=P(:,row*(x(1)-1)+x(2),k);
%             for iterate=1:length(precessor)
%                 if precessor(iterate)~=0
%                    dummy=P(iterate,column*(x(1)-1)+x(2),k);
%                    P(iterate,column*(x(1)-1)+x(2),k)=0;
%                    P(iterate,iterate,k)=P(iterate,iterate,k)+dummy;
%                 end
%             end
%         end
%     end
end

