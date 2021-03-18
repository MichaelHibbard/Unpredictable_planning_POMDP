% This function computes the expected entropy for a given set of lamda
% parameters. THIS ASSUMES A 1-FSC WITH THE STATE x PRODUCT PROBLEM
% FORMULATION.
% obsFunc - The observation function used for the POMDP
% pMat - The transition probability matrix
% lamVals - The values of lambda (parameters of the FSC)

% read in the input files
obsFunc = readmatrix('obsFunc.mat');
pMat = readmatrix('pMatrix.mat');
lamVals = readmatrix('lambda-5x5.mat');

% time horizon considered - DOUBLE CHECK IF YOU UPDATE THE PROBLEM!
tHorizon = 20;

% Recall that ""total"" entropy is the sum of the entropy over all states.
% Recall H(s) = xi(s)*L(s), xi(s) = residence time, L(s) = local entropy  

numStates = size(pMat,1);

% construct a transition matrix that finds total probability of
% transitioning to a successor state
totProbTrans = zeros(numStates,numStates);

% index through each state, successor pair and determine the probability of
% transitioning to that successor
for state = 1:numStates
    for successor = 1:numStates
        probTrans = 0;
        for obs = 1:size(obsFunc,2)
            for act = 1:size(pMat,3)
                probTrans = probTrans + obsFunc(state,obs)*lambda(1,act,obs)*pMat(state,successor,act);
            end
        end
        totProbTrans(state,successor) = probTrans;
    end
end

% construct the local entropies - recall path entropy!
localEnts = zeros(numStates,1);
for state = 1:numStates
    for successor = 1:numStates
        if totProbTrans(state,successor) ~=0
            localEnts(state) = localEnts(state) + -1*totProbTrans(state,successor)*log2(totProbTrans(state,successor));
        end
    end
end

% Now - because we are using the product with the time horizon, the
% expected residence times are fairly simple to compute as they have a
% maximum of one per time step.
expectedResidences = zeros(numStates,tHorizon);
expectedResidences(1,1) = 1; % ASSUMING A UNIQUE INITIAL STATE IN THE UPPER-LEFT CORNER!!!
for time = 1:tHorizon-1
    for successor = 1:numStates
        expectedResTime = 0;
        for state = 1:numStates
            expResTime = expResTime + expResidences(state,time)*totProbTrans(state,successor);
        end
        expectedResidences(successor,time+1) = expectedResTime;
    end
end

% Now, sum over all time horizons to get total expected residence
expectedTotalResidences = sum(expectedResidences,2);

% Multiply to get the total expected entropy.
Entropy = sum(expectedTotalResidences.*localEnts)

% Now, find the probability of reaching the target state.
target = 25;
