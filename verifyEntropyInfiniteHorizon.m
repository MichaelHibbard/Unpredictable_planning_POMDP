% This function computes the expected entropy for a given set of lamda
% parameters. THIS ASSUMES A 1-FSC!
clear;clc;close;

% change this value if necessary
init = 1;
target = [20];


% read in the input files
obsFunc1 = load('obsFunctionMaze.mat');
pMat1 = load('pMatrixMaze.mat');
lambda1 = load('lambda-5009.mat');

obsFunc = obsFunc1.obsFunction;
pMat = pMat1.pMatrix;
lambda = lambda1.lambda;

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

% construct the local entropies
localEnts = zeros(numStates,1);
for state = 1:numStates
    for successor = 1:numStates
        if totProbTrans(state,successor) ~=0
            localEnts(state) = localEnts(state) + -1*totProbTrans(state,successor)*log2(totProbTrans(state,successor));
        end
    end
end

% Value iteration - determine the maximum entropy.
maxError = 1e6;
tol = 1e-6;
valFuncEntr = zeros(numStates,1);
valFuncEntrPH = zeros(numStates,1);
while maxError > tol
    for state = 1:numStates
        valFuncEntrPH(state) = localEnts(state);
        for nextState = 1:numStates
            valFuncEntrPH(state) = valFuncEntrPH(state) + totProbTrans(state,nextState)*valFuncEntr(nextState);
        end
    end
    maxError = max(valFuncEntrPH - valFuncEntr);
    valFuncEntr = valFuncEntrPH(:);
end

% Value iteration - probability of reaching target state
maxError = 1e6;
tol = 1e-6;
valFuncReach = zeros(numStates,1);
valFuncReachPH = zeros(numStates,1);
while maxError > tol
    for state = 1:numStates
        if ismember(state,target)
            valFuncReachPH(state) = 1;
        else
            valFuncReachPH(state) = 0;
            for nextState = 1:numStates
                valFuncReachPH(state) = valFuncReachPH(state) + totProbTrans(state,nextState)*valFuncReach(nextState);
            end
        end
    end
    maxError = max(valFuncReachPH - valFuncReach);
    valFuncReach = valFuncReachPH(:);
end

disp(valFuncReach(init))
disp(valFuncEntr(init))
disp('----------')
