function [lambda,eta,V,slack2,slack1,cvx_optval]=CCCP_inner_problem3(tau,initLambda,initV,initEta,TF,transFuncReduced,absorb,absorbProduct,target,targetProduct,init,initReduced,unreachable,numMemory,cost,costLBound,obsFunction,discount)
% We will use reduced transition function to define optimization variables.
% To keep track of what is reachable and what is not, we use both reduced
% and full transition matrices.

% Recall that:
% lambdas are the probability of selecting an action in a memory node for
%   some observation
% etas are the reward for the state in the product MDP (essentially
%   equivalent to the probability of reaching the target state)
% Vs are the entropy of each state in the product MDP

numStates = size(transFuncReduced,1);       % number of reachable states that we consider for optimization
numNominalStates = size(TF,1)/numMemory;    % number of states in the original MDP
numActions = size(transFuncReduced,3);      % number of actions
numObs = size(obsFunction,2);               % number of observations

cvx_solver MOSEK
cvx_begin quiet

% Define variables using REACHABLE STATES in the PRODUCT!!
variables lambda(numMemory,numActions,numObs) eta(numStates) V(numStates) slack2(numStates) slack1(numStates)
memNode=1;          % counter for memory states
staCounter=1;       % counter for reachable states

% Get some easy stuff out of the way
lambda >= 1e-4; % Probability of action selection must be positive (make sure no entropy terms NaN)
slack1 >= 0;    % Slack variables must be positive
slack2 >= 0;
V <= 1e4;       % Cap the maximum entropy for all states
V >= 0;         % Entropy must be positive
eta <= 1;       % Expected reward for reaching cannot exceed 1
eta >= 0;       % Expected reward for reaching must be positive (this may
% be redundant?)
eta(initReduced) >= costLBound;

% Ensure feasible probability distributions
for mem = 1:numMemory
    
    for obs = 1:numObs
        sum(lambda(mem,:,obs)) == 1;
    end
    
    % iterate through the states on the product MDP
    for state = 1:size(TF,1)

        % skip over states that are unreachable in the product MDP
        if ~ismember(state,unreachable)
            
            % check if the state is a target state
            if ismember(state,target)
                V(staCounter) == 0;         % No entropy in final state
                eta(staCounter) == 1;       % Reward of 1 in final state
                slack1(staCounter) == 0;    % No slack variable for reward - exact
                slack2(staCounter) == 0;    % No slack variable for entropy - exact
                
                % if absorbing but not a target state
            elseif ismember(state,absorb)
                eta(staCounter) == 0;       % No reward - cannot reach target
                V(staCounter) == 0;         % No entropy - returns to itself
                slack1(staCounter) == 0;    % No slack variable for reward - exact
                slack2(staCounter) == 0;    % No slack variable for entropy - exact
                
                % just a regular state
            else
                
%                convexConcaveEtaTot = 0;
%                convexConcaveEntrTot = 0;
                convexPartEntr = [];
                concavePartEntr = [];
                convexPartEta = [];
                concavePartEta = [];
                entrVec = [];
                
                % Find all the states that can be reached in one step
                succStates = find(sum(transFuncReduced(staCounter,:,:),3));
                
                % Encode the local reward.
                probReachAccepting = 0;
                
                % need to iterate through the next state
                for succ = succStates
                    
                    % Initialize transition probabilities to successors - we
                    % care about path entropy, not action entropy!
                    sumTransProb = 0;
                    
                    % iterate through the observations at each state
                    for obs = 1:numObs
                        % iterate through the actions at each state
                        for act = 1:numActions
                            % save some computation time
                            if transFuncReduced(staCounter,succ,act) ~= 0 && obsFunction(staCounter,obs) ~= 0
                                
                                % For the local entropy
                                sumTransProb = sumTransProb + transFuncReduced(staCounter,succ,act)...
                                    *obsFunction(staCounter,obs)*lambda(memNode,act,obs);
                                
                                % successor is a regular state
                                if ~ismember(succ,absorbProduct) && ~ismember(succ,targetProduct)
                                    
                                    % % NOW WE HAVE A DISCOUNT FACTOR HERE, WHICH
                                    % % GOES ALONG WITH OUR OTHER CONSTANTS
                                    % % constant used in convexification
                                    constantEntr = discount*1/2*transFuncReduced(staCounter,succ,act)...
                                        *obsFunction(staCounter,obs);
                                    
                                    constantEta = 1/2*transFuncReduced(staCounter,succ,act)...
                                        *obsFunction(staCounter,obs);

%                                     % linearize around initial point (see Murat's paper) (for entropy)
%                                     convexPartEntr = constantEntr*square(lambda(memNode,act,obs) - V(succ));
%                                     concavePartEntr = constantEntr*(initLambda(memNode,act,obs)*initLambda(memNode,act,obs) + initV(succ)*initV(succ)...
%                                     - 2*initLambda(memNode,act,obs)*lambda(memNode,act,obs) - 2*initV(succ)*V(succ));
%                                     convexConcaveEntrTot = convexConcaveEntrTot + convexPartEntr + concavePartEntr;
%                                     
%                                     % Convexification procedure (for reach)
%                                     convexPartEta = constantEta*square(lambda(memNode,act,obs) - eta(succ));
%                                     concavePartEta = constantEta*(initLambda(memNode,act,obs)*initLambda(memNode,act,obs) + initEta(succ)*initEta(succ)...
%                                     - 2*initLambda(memNode,act,obs)*lambda(memNode,act,obs) - 2*initEta(succ)*eta(succ));
%                                     convexConcaveEtaTot = convexConcaveEtaTot + convexPartEta + concavePartEta;
                                    
                                    convexPartEntr = [convexPartEntr, sqrt(constantEntr)*(lambda(memNode,act,obs) - V(succ))];
                                    concavePartEntr = [concavePartEntr, constantEntr*(initLambda(memNode,act,obs)*initLambda(memNode,act,obs) + initV(succ)*initV(succ)...
                                        - 2*initLambda(memNode,act,obs)*lambda(memNode,act,obs) - 2*initV(succ)*V(succ))];
                                    
                                    convexPartEta = [convexPartEta, sqrt(constantEta)*(lambda(memNode,act,obs) - eta(succ))];
                                    concavePartEta = [concavePartEta, constantEta*(initLambda(memNode,act,obs)*initLambda(memNode,act,obs) + initEta(succ)*initEta(succ)...
                                        - 2*initLambda(memNode,act,obs)*lambda(memNode,act,obs) - 2*initEta(succ)*eta(succ))];
                                end
                            end
                        end
                    end
                    
                    entrVec = [entrVec, sumTransProb];
                    
                    % For local reward, need to find reach probability (to any
                    % accepting state), so add up probability over all
                    % successors.
                    if ismember(succ,targetProduct)
                        probReachAccepting = probReachAccepting + sumTransProb;
                    end
                    
                end
                
                %             % Constrain transitions for each state to sum to 1
                %             probSum = 0;
                %             for succ = succStates
                %                 for act = 1:numActions
                %                     for obs = 1:numObs
                %                         if transFuncReduced(staCounter,succ,act) ~= 0
                %                             probSum = probSum + transFuncReduced(staCounter,succ,act)*...
                %                                 obsFunction(staCounter,obs)*lambda(memNode,act,obs);
                %                         end
                %                     end
                %                 end
                %             end
                %             probSum == 1;
                
                % constraint for entropy Bellman - need to convert to bits.
                V(staCounter) - log2(exp(1))*sum(entr(entrVec)) + sum_square(convexPartEntr) + sum(concavePartEntr) <= slack1(staCounter);
                
                % constraint for expected reward Bellman
                eta(staCounter) - probReachAccepting + sum_square(convexPartEta) + sum(concavePartEta) <= slack2(staCounter);
                
            end
            
            % Iterate the state counter to keep track of difference between nominal and actual states.
            staCounter = staCounter+1;
        end
        
        % Check if we need to update the memory node as well (each time we iterate
        % through the number of states in the original MDP on the product MDP)
        if mod(state,numNominalStates) == 0
            memNode = memNode + 1;
        end
        
    end
    
    maximize(V(initReduced) - tau*(sum(slack1) + sum(slack2)));
    cvx_end
end


