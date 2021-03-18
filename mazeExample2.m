% Problem for maze scenario. Hard code for now. Possibly look into Airsim
% neighborhood environment to display.

close;clear all;clc;

% Construct the maze example

numStates = 24;
numObservations = 16;
numAbsorbingStates = 3;
numTargetState = 20;
numActions = 4;

numMemoryStates = 1;

% basic maze dynamics
discount = 1;
tP = 0.99;
s = (1-tP)/3;
init = 9;
absorb = [1,6,12,13,16,20,22];
absorb_state_nums = [1,6,12,13,16,20,22];
target = 20;

% observation function

% 1.	None
% 2.	Up
% 3.	Down
% 4.	Left
% 5.	Right
% 6.	Up, Left
% 7.	Up, Down
% 8.	Up, Right
% 9.	Left, Down
% 10.	Left, Right
% 11.	Down, Right
% 12.	Up, Left, Down
% 13.	Left, Down, Right
% 14.	Down, Right, Up
% 15.	Right, Up, Left
% 16.	Up, Down, Left, Right

obsFunction = zeros(numStates,numObservations);
obsFunction(1,1) = 1;
obsFunction(2,13) = 1;
obsFunction(3,13) = 1;
obsFunction(4,15) = 1;
obsFunction(5,12) = 1;
obsFunction(6,1) = 1;
obsFunction(7,13) = 1;
obsFunction(8,14) = 1;
obsFunction(9,14) = 1;
obsFunction(10,16) = 1;
obsFunction(11,16) = 1;
obsFunction(12,1) = 1;
obsFunction(13,1) = 1;
obsFunction(14,14) = 1;
obsFunction(15,16) = 1;
obsFunction(16,1) = 1;
obsFunction(17,14) = 1;
obsFunction(18,13) = 1;
obsFunction(19,12) = 1;
obsFunction(20,1) = 1;
obsFunction(21,15) = 1;
obsFunction(22,1) = 1;
obsFunction(23,6) = 1;
obsFunction(24,11) = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% transition probabability matrix  %
% actions are (1) UP               %
%             (2) DOWN             %
%             (3) LEFT             %
%             (4) RIGHT            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% trivial states
pMatrix = zeros(numStates,numStates,4);
pMatrix(1,1,:) = 1;
pMatrix(6,6,:) = 1;
pMatrix(12,12,:) = 1;
pMatrix(16,16,:) = 1;
pMatrix(20,20,:) = 1;
pMatrix(22,22,:) = 1;
pMatrix(13,13,:) = 1;

% non-trivial states
pMatrix(2,2,1) = tP; pMatrix(2,1,1) = s; pMatrix(2,3,1) = s; pMatrix(2,4,1) = s;
pMatrix(2,2,2) = s; pMatrix(2,1,2) = s; pMatrix(2,3,2) = s; pMatrix(2,4,2) = tP;
pMatrix(2,2,3) = s; pMatrix(2,1,3) = tP; pMatrix(2,3,3) = s; pMatrix(2,4,3) = s;
pMatrix(2,2,4) = s; pMatrix(2,1,4) = s; pMatrix(2,3,4) = tP; pMatrix(2,4,4) = s;

pMatrix(3,2,1) = s; pMatrix(3,3,1) = tP; pMatrix(3,6,1) = s; pMatrix(3,10,1) = s;
pMatrix(3,2,2) = s; pMatrix(3,3,2) = s; pMatrix(3,6,2) = s; pMatrix(3,10,2) = tP;
pMatrix(3,2,3) = tP; pMatrix(3,3,3) = s; pMatrix(3,6,3) = s; pMatrix(3,10,3) = s;
pMatrix(3,2,4) = s; pMatrix(3,3,4) = s; pMatrix(3,6,4) = tP; pMatrix(3,10,4) = s;

pMatrix(4,2,1) = tP; pMatrix(4,4,1) = 2*s; pMatrix(4,7,1) = s;
pMatrix(4,2,2) = s; pMatrix(4,4,2) = tP+s; pMatrix(4,7,2) = s;
pMatrix(4,2,3) = s; pMatrix(4,4,3) = tP+s; pMatrix(4,7,3) = s;
pMatrix(4,2,4) = s; pMatrix(4,4,4) = 2*s; pMatrix(4,7,4) = tP;

pMatrix(5,5,1) = tP+s; pMatrix(5,8,1) = s; pMatrix(5,24,1) = s;
pMatrix(5,5,2) = 2*s; pMatrix(5,8,2) = tP; pMatrix(5,24,2) = s;
pMatrix(5,5,3) = 2*s; pMatrix(5,8,3) = s; pMatrix(5,24,3) = tP;
pMatrix(5,5,4) = tP+s; pMatrix(5,8,4) = s; pMatrix(5,24,4) = s;

pMatrix(7,4,1) = s; pMatrix(7,7,1) = tP; pMatrix(7,8,1) = s; pMatrix(7,9,1) = s;
pMatrix(7,4,2) = s; pMatrix(7,7,2) = s; pMatrix(7,8,2) = s; pMatrix(7,9,2) = tP;
pMatrix(7,4,3) = s; pMatrix(7,7,3) = s; pMatrix(7,8,3) = tP; pMatrix(7,9,3) = s;
pMatrix(7,4,4) = tP; pMatrix(7,7,4) = s; pMatrix(7,8,4) = s; pMatrix(7,9,4) = s;

pMatrix(8,5,1) = tP; pMatrix(8,7,1) = s; pMatrix(8,8,1) = s; pMatrix(8,14,1) = s;
pMatrix(8,5,2) = s; pMatrix(8,7,2) = s; pMatrix(8,8,2) = s; pMatrix(8,14,2) = tP;
pMatrix(8,5,3) = s; pMatrix(8,7,3) = s; pMatrix(8,8,3) = tP; pMatrix(8,14,3) = s;
pMatrix(8,5,4) = s; pMatrix(8,7,4) = tP; pMatrix(8,8,4) = s; pMatrix(8,14,4) = s;

pMatrix(9,7,1) = tP; pMatrix(9,9,1) = s; pMatrix(9,10,1) = s; pMatrix(9,15,1) = s;
pMatrix(9,7,2) = s; pMatrix(9,9,2) = s; pMatrix(9,10,2) = s; pMatrix(9,15,2) = tP;
pMatrix(9,7,3) = s; pMatrix(9,9,3) = tP; pMatrix(9,10,3) = s; pMatrix(9,15,3) = s;
pMatrix(9,7,4) = s; pMatrix(9,9,4) = s; pMatrix(9,10,4) = tP; pMatrix(9,15,4) = s;

pMatrix(10,3,1) = tP; pMatrix(10,6,1) = s; pMatrix(10,9,1) = s; pMatrix(10,11,1) = s;
pMatrix(10,3,2) = s; pMatrix(10,6,2) = s; pMatrix(10,9,2) = s; pMatrix(10,11,2) = tP;
pMatrix(10,3,3) = s; pMatrix(10,6,3) = s; pMatrix(10,9,3) = tP; pMatrix(10,11,3) = s;
pMatrix(10,3,4) = s; pMatrix(10,6,4) = tP; pMatrix(10,9,4) = s; pMatrix(10,11,4) = s;

pMatrix(11,10,1) = tP; pMatrix(11,12,1) = s; pMatrix(11,15,1) = s; pMatrix(11,19,1) = s;
pMatrix(11,10,2) = s; pMatrix(11,12,2) = s; pMatrix(11,15,2) = s; pMatrix(11,19,2) = tP;
pMatrix(11,10,3) = s; pMatrix(11,12,3) = s; pMatrix(11,15,3) = tP; pMatrix(11,19,3) = s;
pMatrix(11,10,4) = s; pMatrix(11,12,4) = tP; pMatrix(11,15,4) = s; pMatrix(11,19,4) = s;

pMatrix(14,8,1) = tP; pMatrix(14,14,1) = s; pMatrix(14,15,1) = s; pMatrix(14,18,1) = s;
pMatrix(14,8,2) = s; pMatrix(14,14,2) = s; pMatrix(14,15,2) = s; pMatrix(14,18,2) = tP;
pMatrix(14,8,3) = s; pMatrix(14,14,3) = tP; pMatrix(14,15,3) = s; pMatrix(14,18,3) = s;
pMatrix(14,8,4) = s; pMatrix(14,14,4) = s; pMatrix(14,15,4) = tP; pMatrix(14,18,4) = s;

pMatrix(15,9,1) = tP; pMatrix(15,11,1) = s; pMatrix(15,15,1) = s; pMatrix(15,21,1) = s;
pMatrix(15,9,2) = s; pMatrix(15,11,2) = s; pMatrix(15,15,2) = s; pMatrix(15,21,2) = tP;
pMatrix(15,9,3) = s; pMatrix(15,11,3) = s; pMatrix(15,15,3) = tP; pMatrix(15,21,3) = s;
pMatrix(15,9,4) = s; pMatrix(15,11,4) = tP; pMatrix(15,15,4) = s; pMatrix(15,21,4) = s;

pMatrix(17,13,1) = tP; pMatrix(17,17,1) = s; pMatrix(17,20,1) = s; pMatrix(17,21,1) = s;
pMatrix(17,13,2) = s; pMatrix(17,17,2) = s; pMatrix(17,20,2) = tP; pMatrix(17,21,2) = s;
pMatrix(17,13,3) = s; pMatrix(17,17,3) = tP; pMatrix(17,20,3) = s; pMatrix(17,21,3) = s;
pMatrix(17,13,4) = s; pMatrix(17,17,4) = s; pMatrix(17,20,4) = s; pMatrix(17,21,4) = tP;

pMatrix(18,13,1) = s; pMatrix(18,14,1) = s; pMatrix(18,18,1) = tP; pMatrix(18,21,1) = s;
pMatrix(18,13,2) = s; pMatrix(18,14,2) = s; pMatrix(18,18,2) = s; pMatrix(18,21,2) = tP;
pMatrix(18,13,3) = tP; pMatrix(18,14,3) = s; pMatrix(18,18,3) = s; pMatrix(18,21,3) = s;
pMatrix(18,13,4) = s; pMatrix(18,14,4) = tP; pMatrix(18,18,4) = s; pMatrix(18,21,4) = s;

pMatrix(19,11,1) = tP; pMatrix(19,19,1) = s; pMatrix(19,22,1) = s; pMatrix(19,23,1) = s;
pMatrix(19,11,2) = s; pMatrix(19,19,2) = s; pMatrix(19,22,2) = s; pMatrix(19,23,2) = tP;
pMatrix(19,11,3) = s; pMatrix(19,19,3) = s; pMatrix(19,22,3) = tP; pMatrix(19,23,3) = s;
pMatrix(19,11,4) = s; pMatrix(19,19,4) = tP; pMatrix(19,22,4) = s; pMatrix(19,23,4) = s;

pMatrix(21,15,1) = s; pMatrix(21,17,1) = s; pMatrix(21,18,1) = tP; pMatrix(21,21,1) = s;
pMatrix(21,15,2) = s; pMatrix(21,17,2) = s; pMatrix(21,18,2) = s; pMatrix(21,21,2) = tP;
pMatrix(21,15,3) = s; pMatrix(21,17,3) = tP; pMatrix(21,18,3) = s; pMatrix(21,21,3) = s;
pMatrix(21,15,4) = tP; pMatrix(21,17,4) = s; pMatrix(21,18,4) = s; pMatrix(21,21,4) = s;

pMatrix(23,16,1) = tP; pMatrix(23,19,1) = s; pMatrix(23,23,1) = 2*s;
pMatrix(23,16,2) = s; pMatrix(23,19,2) = s; pMatrix(23,23,2) = tP+s;
pMatrix(23,16,3) = s; pMatrix(23,19,3) = tP; pMatrix(23,23,3) = 2*s;
pMatrix(23,16,4) = s; pMatrix(23,19,4) = s; pMatrix(23,23,4) = tP+s;

pMatrix(24,24,1) = tP+s; pMatrix(24,5,1) = s; pMatrix(24,13,1) = s;
pMatrix(24,24,2) = 2*s; pMatrix(24,5,2) = s; pMatrix(24,13,2) = tP;
pMatrix(24,24,3) = tP+s; pMatrix(24,5,3) = s; pMatrix(24,13,3) = s;
pMatrix(24,24,4) = 2*s; pMatrix(24,5,4) = tP; pMatrix(24,13,4) = s;

save('pMatrixMaze.mat','pMatrix')
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

stringindicator = 6000;
for cost_lower_bound = [0.5 0.9]
    for t = 1:1
        
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
end
% % Simulate the resulting environment
% T_horizon = 50;    % Time horizon to run simulation over
% num_trials = 250;    % Number of iterations to test the policy
% runSimulation_Gridworld1(lambda,P_matrix,num_memory_states,row,column,numStates,obs_function,T_horizon,num_trials,init,size(obs_function,2));     % Run the simulation

