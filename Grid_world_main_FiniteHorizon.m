clear all
clc

costLowerBound = 0.9;
stringIndicator = 22;

tHorizon = 18;

% Construct a Gridworld with an observation function and finite time horizon.
% tHorizon = 20;

% Discount Factor
discount = 1;

% parameters for the state space
row = 4;                                    % rows in the gridworld
column = 4;                                 % columns in the gridworld
numStatesNominal = row*column;              % Number of states in the original MDP
numMemoryStates = 1;                        % Memory states in the FSC
act = 4;                                    % Number of available actions - up, down, left, right
wall = [];                                  % walls - not absorbing but cannot transition through these
absorb = [[1,4];[4,1];[4,4]];               % states that can only transition to themselves
prob = 0.95;                                % probability of successfully transitioning to desired state
eps = 0.005;                                % probability of slipping backwards (required for finite entropy)
init = 1;                                   % initial state in the POMDP
target = [16];                              % target state in the POMDP
targetLoc = [[4,4]];                        % For simplicty making the observation function

% Construct the transition probability matrix for the gridworld
pMatrix = grid_world2(row,column,act,wall,absorb,prob,eps,tHorizon);
numStates = size(pMatrix,1);

% Construct the observation matrix for the gridworld
obsFunction = CreateObservationMatrix(row,column,absorb,targetLoc,tHorizon);

% Construct the deterministic finite state controller
FSC = FSC_absorbing(numMemoryStates,act);

% Construct the product transition probabilities
pProduct = product_MDP(pMatrix,FSC);

% Construct the product observation function
obsProduct = product_Obs(obsFunction,FSC);

% Need to convert the absorb [row column] to state numbers
% THIS IS FOR THE STATE-TIME PRODUCT!
absorbStateNums = zeros(size(absorb,1)*(tHorizon-1)+numStatesNominal,1);
staCounter = 1;
for time = 1:tHorizon-1
    for state = 1:size(absorb,1)
        absorbStateNums(staCounter,1) = (time-1)*numStatesNominal + column*(absorb(state,1)-1) + absorb(state,2);
        staCounter = staCounter + 1;
    end
end
% Now append that all final states are absorbing
for state = 1:numStatesNominal
    absorbStateNums(size(absorb,1)*(tHorizon-1)+state,1) = numStatesNominal*(tHorizon-1) + state;
end

% Now, need to do the same for the target locations.
targetStateTime = zeros(length(target)*tHorizon,1);
staCounter = 1;
for time = 1:tHorizon
    for targ = 1:length(target)
        targetStateTime(staCounter,1) = (time-1)*numStatesNominal + target(targ);
        staCounter = staCounter + 1;
    end
end

% Initialize some stuff for removing unreachable states
targetProduct = zeros(numMemoryStates*length(targetStateTime),1);
absorbProduct = zeros(numMemoryStates*length(absorbStateNums),1);
% essentially, for each state on the product that has a target state on the
% MDP, set that state to be a target (and same for absorbing states)
for k = 1:length(targetStateTime)
    targetProduct(numMemoryStates*(k-1)+1:numMemoryStates*k) = (0:numMemoryStates-1)*numStates+targetStateTime(k);
end
for k = 1:length(absorbStateNums)
    absorbProduct(numMemoryStates*(k-1)+1:numMemoryStates*k) = (0:numMemoryStates-1)*numStates+absorbStateNums(k);
end

% Remove the unreachable states from the transition probability matrix -
% BECAUSE WE HAVE PRODUCT WITH TIME HORIZON, WILL ALSO HAVE UNREACHABLE
% STATES
% if numMemoryStates > 1

pNominal = pProduct;
[reachable,~] = find_reachable(pProduct,init);
unreachable = sort(setdiff(1:size(pProduct,1),reachable),'descend');
disp(unreachable)

pProduct(unreachable,:,:) = [];    % states transitioning from
pProduct(:,unreachable,:) = [];    % states transitioning to

% Update the target states on the product MDP according to removed states
targetProduct = targetProduct(~ismember(targetProduct,unreachable));
absorbProduct = absorbProduct(~ismember(absorbProduct,unreachable));
absorbProduct = sort(setdiff(absorbProduct,targetProduct),'descend');
absorbOriginal = absorbProduct;
targetOriginal = targetProduct;

% Get indices for unreachable and absorbing states?
for k=1:length(targetProduct)
    numSmallerStates=unreachable(unreachable<targetProduct(k));
    targetProduct(k)=targetProduct(k)-length(numSmallerStates);
end

for k=1:length(absorbProduct)
    numSmallerStates=unreachable(unreachable<absorbProduct(k));
    absorbProduct(k)=absorbProduct(k)-length(numSmallerStates);
end

numSmallerStates=unreachable(unreachable<init);
initReduced=init-length(numSmallerStates);
disp(initReduced)
% else
%     pNominal = pProduct;
%     absorbOriginal = absorbProduct;
%     targetOriginal = targetProduct;
%     initReduced = init;
%     % Because we have the product with time, we WILL have some unreachable
%     % states!
%     [reachable,~] = find_reachable(pProduct,init);
%     unreachable = s
% end

% Define the cost function
cost=zeros(size(pProduct,1),act);
for i=1:size(pProduct,1)
    if ~ismember(i,targetProduct)
        for j=1:act
            cost(i,j)=cost(i,j)+sum(pProduct(i,targetProduct,j));
        end
    end
end

initLambda=rand(numMemoryStates,act,size(obsFunction,2));

for i=1:numMemoryStates
    initLambda(i,:,:)=initLambda(i,:,:)/sum(sum(initLambda(i,:,:)));
end

initEta=rand(size(pProduct,1),1);
initV=zeros(size(pProduct,1),1);

[lambda,V,eta]=CCCP(initLambda,initV,initEta,pNominal,pProduct,absorbOriginal,absorbProduct,targetOriginal,targetProduct,init,initReduced,unreachable,numMemoryStates,cost,costLowerBound,obsProduct,discount)

filename1 = ['lambda-0' num2str(stringIndicator) '0.mat'];
filename2 = ['reach-0' num2str(stringIndicator) '0.mat'];
filename3 = ['entropy-0' num2str(stringIndicator) '0.mat'];
save(filename1,'lambda')
save(filename2,'eta')
save(filename3,'V')

stringIndicator = stringIndicator + 2;

% Simulate the resulting environment
% numTrials = 250;    % Number of iterations to test the policy
% runSimulation_Gridworld(lambda,pMatrix,numMemoryStates,row,column,numStates,obsFunction,tHorizon,numTrials,init,size(obsFunction,2));     % Run the simulation



