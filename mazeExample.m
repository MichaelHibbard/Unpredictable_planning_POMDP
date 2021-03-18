% Problem for maze scenario. Hard code for now. Possibly look into Airsim
% neighborhood environment to display.

close;clear all;clc;

% Construct the maze example

numStates = 13;
numObservations = 16;
numAbsorbingStates = 2;
numTargetState = 1;
numActions = 4;

numMemoryStates = 1;

% Construct the transition probability matrix

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% ROUGH REPRESENTATION OF THE MAZE CONSIDERED %%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% S           I                 I %
% % % % % % %   % % % % % % % %   %
%             I       % %   % %   %
%   % % % % %   % % I     I     I %
%   % % % % %   % %   % % % % %   %
%   % % % % %   % % I       E %   %
%   % % % % %   % %   % % % % %   %
%   % % % % %   % % I           I %
%             I       % % % % % T %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% basic maze dynamics
discount = 1;
tP = 0.95;
s = (1-0.95)/3;
init = 1;
absorb = [9,13];
absorb_state_nums = [9,13];
target = 13;

% observation function
obsFunction = zeros(numStates,numObservations);
obsFunction(1,1) = 1;
obsFunction(2,13) = 1;
obsFunction(3,9) = 1;
obsFunction(4,16) = 1;
obsFunction(5,14) = 1;
obsFunction(6,15) = 1;
obsFunction(7,12) = 1;
obsFunction(8,14) = 1;
obsFunction(9,1) = 1;
obsFunction(10,14) = 1;
obsFunction(11,12) = 1;
obsFunction(12,15) = 1;
obsFunction(13,1) = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% transition probabability matrix  %
% actions are (1) LEFT             %
%             (2) RIGHT            %
%             (3) UP               %
%             (4) DOWN             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% trivial states
pMatrix = zeros(numStates,numStates,4);
pMatrix(1,2,:) = 1;
pMatrix(9,9,:) = 1;
pMatrix(13,13,:) = 1;

% non-trivial states
pMatrix(2,1,1) = tP; pMatrix(2,2,1) = s; pMatrix(2,3,1) = s; pMatrix(2,4,1) = s;
pMatrix(2,1,2) = s; pMatrix(2,2,2) = s; pMatrix(2,3,2) = tP; pMatrix(2,4,2) = s;
pMatrix(2,1,3) = s; pMatrix(2,2,3) = tP; pMatrix(2,3,3) = s; pMatrix(2,4,3) = s;
pMatrix(2,1,4) = s; pMatrix(2,2,4) = s; pMatrix(2,3,4) = s; pMatrix(2,4,4) = tP;
pMatrix(3,2,1) = tP; pMatrix(3,3,1) = 2*s; pMatrix(3,7,1) = s;
pMatrix(3,2,2) = s; pMatrix(3,3,2) = tP+s; pMatrix(3,7,2) = s;
pMatrix(3,2,3) = s; pMatrix(3,3,3) = tP+s; pMatrix(3,7,3) = s;
pMatrix(3,2,4) = s; pMatrix(3,3,4) = 2*s; pMatrix(3,7,4) = tP; 
pMatrix(4,2,1) = s; pMatrix(4,5,1) = s; pMatrix(4,12,1) = tP+s;
pMatrix(4,2,2) = s; pMatrix(4,5,2) = tP; pMatrix(4,12,2) = 2*s;
pMatrix(4,2,3) = tP; pMatrix(4,5,3) = s; pMatrix(4,12,3) = 2*s;
pMatrix(4,2,4) = s; pMatrix(4,5,4) = s; pMatrix(4,12,4) = tP+s;
pMatrix(5,4,1) = s; pMatrix(5,5,1) = tP; pMatrix(5,6,1) = s; pMatrix(5,8,1) = s;
pMatrix(5,4,2) = s; pMatrix(5,5,2) = s; pMatrix(5,6,2) = tP; pMatrix(5,8,2) = s;
pMatrix(5,4,3) = tP; pMatrix(5,5,3) = s; pMatrix(5,6,3) = s; pMatrix(5,8,3) = s;
pMatrix(5,4,4) = s; pMatrix(5,5,4) = s; pMatrix(5,6,4) = s; pMatrix(5,8,4) = tP;
pMatrix(6,5,1) = tP; pMatrix(6,6,1) = 2*s; pMatrix(6,7,1) = s;
pMatrix(6,5,2) = s; pMatrix(6,6,2) = 2*s; pMatrix(6,7,2) = tP;
pMatrix(6,5,3) = s; pMatrix(6,6,3) = tP+s; pMatrix(6,7,3) = s;
pMatrix(6,5,4) = s; pMatrix(6,6,4) = tP+s; pMatrix(6,7,4) = s;
pMatrix(7,3,1) = s; pMatrix(7,6,1) = tP; pMatrix(7,7,1) = s; pMatrix(7,11,1) = s;
pMatrix(7,3,2) = s; pMatrix(7,6,2) = s; pMatrix(7,7,2) = tP; pMatrix(7,11,2) = s;
pMatrix(7,3,3) = tP; pMatrix(7,6,3) = s; pMatrix(7,7,3) = s; pMatrix(7,11,3) = s;
pMatrix(7,3,4) = s; pMatrix(7,6,4) = s; pMatrix(7,7,4) = s; pMatrix(7,11,4) = tP;
pMatrix(8,5,1) = s; pMatrix(8,8,1) = tP; pMatrix(8,9,1) = s; pMatrix(8,10,1) = s;
pMatrix(8,5,2) = s; pMatrix(8,8,2) = s; pMatrix(8,9,2) = tP; pMatrix(8,10,2) = s;
pMatrix(8,5,3) = tP; pMatrix(8,8,3) = s; pMatrix(8,9,3) = s; pMatrix(8,10,3) = s;
pMatrix(8,5,4) = s; pMatrix(8,8,4) = s; pMatrix(8,9,4) = s; pMatrix(8,10,4) = tP;
pMatrix(10,8,1) = s; pMatrix(10,10,1) = tP; pMatrix(10,11,1) = s; pMatrix(10,12,1) = s;
pMatrix(10,8,2) = s; pMatrix(10,10,2) = s; pMatrix(10,11,2) = tP; pMatrix(10,12,2) = s;
pMatrix(10,8,3) = tP; pMatrix(10,10,3) = s; pMatrix(10,11,3) = s; pMatrix(10,12,3) = s;
pMatrix(10,8,4) = s; pMatrix(10,10,4) = s; pMatrix(10,11,4) = s; pMatrix(10,12,4) = tP;
pMatrix(11,7,1) = s; pMatrix(11,10,1) = tP; pMatrix(11,11,1) = s; pMatrix(11,13,1) = s;
pMatrix(11,7,2) = s; pMatrix(11,10,2) = s; pMatrix(11,11,2) = tP; pMatrix(11,13,2) = s;
pMatrix(11,7,3) = tP; pMatrix(11,10,3) = s; pMatrix(11,11,3) = s; pMatrix(11,13,3) = s;
pMatrix(11,7,4) = s; pMatrix(11,10,4) = s; pMatrix(11,11,4) = s; pMatrix(11,13,4) = tP;
pMatrix(12,4,1) = tP+s; pMatrix(12,10,1) = s; pMatrix(12,12,1) = s;
pMatrix(12,4,2) = 2*s; pMatrix(12,10,2) = tP; pMatrix(12,12,2) = s;
pMatrix(12,4,3) = tP+s; pMatrix(12,10,3) = s; pMatrix(12,12,3) = s;
pMatrix(12,4,4) = 2*s; pMatrix(12,10,4) = s; pMatrix(12,12,4) = tP;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Construct the deterministic finite state controller
FSC = FSC_absorbing(numMemoryStates,numActions);

% Construct the product transition probabilities
P_product = product_MDP(pMatrix,FSC);

% Construct the product observation function
Obs_Product = product_Obs(obsFunction,FSC);

% Need to convert the absorb [row column] to state numbers
% absorb_state_nums = zeros(size(absorb,1),1);
% for state = 1:size(absorb,1)
%     absorb_state_nums(state,1) = column*(absorb(state,1)-1) + absorb(state,2);
% end

% Initialize some stuff for removing unreachable states
target_product = zeros(numMemoryStates*length(target),1);
absorb_product = zeros(numMemoryStates*length(absorb),1);
% essentially, for each state on the product that has a target state on the
% MDP, set that state to be a target (and same for absorbing states)
for k = 1:length(target)
    target_product(numMemoryStates*(k-1)+1:numMemoryStates*k) = (0:numMemoryStates-1)*numStates+target(k);
end
for k = 1:length(absorb)
    absorb_product(numMemoryStates*(k-1)+1:numMemoryStates*k) = (0:numMemoryStates-1)*numStates+absorb_state_nums(k);
end

% Remove the unreachable states from the transition probability matrix
if numMemoryStates > 1
    
    P_nominal = P_product;
    [reachable,~] = find_reachable(P_product,init);
    unreachable = sort(setdiff(1:size(P_product,1),reachable),'descend');
    disp(unreachable)
    
    P_Product(unreachable,:,:) = [];    % states transitioning from
    P_Product(:,unreachable,:) = [];    % states transitioning to
    
    % Update the target states on the product MDP according to removed states
    target_product = target_product(~ismember(target_product,unreachable));
    absorb_product = absorb_product(~ismember(absorb_product,unreachable));
    absorb_product = sort(setdiff(absorb_product,target_product),'descend');
    absorb_original = absorb_product;
    target_original = target_product;
    
    % Get indices for unreachable and absorbing states?
    for k=1:length(target_product)
        num_smaller_states=unreachable(unreachable<target_product(k));
        target_product(k)=target_product(k)-length(num_smaller_states);
    end
    
    for k=1:length(absorb_product)
        num_smaller_states=unreachable(unreachable<absorb_product(k));
        absorb_product(k)=absorb_product(k)-length(num_smaller_states);
    end
    
    
    num_smaller_states=unreachable(unreachable<init);
    init_reduced=init-length(num_smaller_states);
    disp('found bitch')
    disp(init_reduced)
else
    P_nominal = P_product;
    absorb_original = absorb_product;
    target_original = target_product;
    init_reduced = init;
    unreachable = [];
end


% Define the cost function
cost=zeros(size(P_product,1),numActions);
for i=1:size(P_product,1)
    if ~ismember(i,target_product)
        for j=1:numActions
             cost(i,j)=cost(i,j)+sum(P_product(i,target_product,j));
        end
    end
end


init_lambda=rand(numMemoryStates,numActions,size(obsFunction,2));

stringindicator = 5000;
for cost_lower_bound = [0.5 0.9]
    
    for i=1:numMemoryStates
        init_lambda(i,:,:)=init_lambda(i,:,:)/sum(sum(init_lambda(i,:,:)));
    end
    
    init_eta=rand(size(P_product,1),1);
    init_V=zeros(size(P_product,1),1);
    
    [lambda,V,eta]=CCCP(init_lambda,init_V,init_eta,P_nominal,P_product,absorb_original,absorb_product,target_original,target_product,init,init_reduced,unreachable,numMemoryStates,cost,cost_lower_bound,Obs_Product,discount)
    
    filename1 = ['lambda-' num2str(stringindicator) '.mat'];
    filename2 = ['reach-' num2str(stringindicator) '.mat'];
    filename3 = ['entropy-' num2str(stringindicator) '.mat'];
    save(filename1,'lambda')
    save(filename2,'eta')
    save(filename3,'V')
    
    stringindicator = stringindicator + 1;
end
% % Simulate the resulting environment
% T_horizon = 50;    % Time horizon to run simulation over
% num_trials = 250;    % Number of iterations to test the policy
% runSimulation_Gridworld1(lambda,P_matrix,num_memory_states,row,column,numStates,obs_function,T_horizon,num_trials,init,size(obs_function,2));     % Run the simulation

