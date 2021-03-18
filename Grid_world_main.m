clear all
clc

% Construct a Gridworld with an observation function.
stringindicator = 9000;

% parameters
row = 10;               % rows in the gridworld
column = 10;            % columns in the gridworld
num_memory_states = 1;  % Memory states in the FSC
act = 4;                % Number of available actions - up, down, left, right
wall = [];              % walls - not absorbing but cannot transition through these

% absorb = [];
% for c = 1:column
%     absorb = [absorb;[1,c];[column,c]];
% end
% for r = 2:row-1
%     absorb = [absorb;[r,1];[r,c]];
% end
% absorb = [absorb;[11,11]];
absorb = [[5,5];[10,10]];

prob = 0.95;             % probability of successfully transitioning to desired state
eps = 0.01;            % probability of slipping backwards (required for finite entropy)
init = 1;               % initial state in the POMDP
target = 100;             % target state in the POMDP
targetLoc = [10,10];      % For simplicty making the observation function
%

% Construct the transition probability matrix for the gridworld
P_matrix = grid_world3(row,column,act,wall,absorb,prob,eps);
num_states = size(P_matrix,1);

% Construct the observation matrix for the gridworld
obs_function = CreateObservationMatrix1(row,column,absorb,targetLoc);

% Construct the deterministic finite state controller
FSC = FSC_absorbing(num_memory_states,act);

% Construct the product transition probabilities
P_product = product_MDP(P_matrix,FSC);

% Construct the product observation function
Obs_Product = product_Obs(obs_function,FSC);

% Need to convert the absorb [row column] to state numbers
absorb_state_nums = zeros(size(absorb,1),1);
for state = 1:size(absorb,1)
    absorb_state_nums(state,1) = column*(absorb(state,1)-1) + absorb(state,2);
end

% Initialize some stuff for removing unreachable states
target_product = zeros(num_memory_states*length(target),1);
absorb_product = zeros(num_memory_states*length(absorb),1);

% Essentially, for each state on the product that has a target state on the
% MDP, set that state to be a target (and same for absorbing states)
for k = 1:length(target)
    target_product(num_memory_states*(k-1)+1:num_memory_states*k) = (0:num_memory_states-1)*num_states+target(k);
end
for k = 1:length(absorb)
    absorb_product(num_memory_states*(k-1)+1:num_memory_states*k) = (0:num_memory_states-1)*num_states+absorb_state_nums(k);
end

% Remove the unreachable states from the transition probability matrix
if num_memory_states > 1
    
    P_nominal = P_product;
    [reachable,~] = find_reachable(P_product,init);
    unreachable = sort(setdiff(1:size(P_product,1),reachable),'descend');
    %disp(unreachable)
    
    P_Product(unreachable,:,:) = [];    % states transitioning from
    P_Product(:,unreachable,:) = [];    % states transitioning to
    
    % Update the target states on the product MDP according to removed states
    target_product = target_product(~ismember(target_product,unreachable));
    absorb_product = absorb_product(~ismember(absorb_product,unreachable));
    absorb_product = sort(setdiff(absorb_product,target_product),'descend');
    absorb_original = absorb_product;
    target_original = target_product;
    
    % Get indices for unreachable and absorbing states
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
    disp(init_reduced)
else
    P_nominal = P_product;
    absorb_original = absorb_product;
    target_original = target_product;
    init_reduced = init;
    unreachable = [];
end


% Define the cost function
cost=zeros(size(P_product,1),act);
for i=1:size(P_product,1)
    if ~ismember(i,target_product)
        for j=1:act
             cost(i,j)=cost(i,j)+sum(P_product(i,target_product,j));
        end
    end
end


cost_lower_bound=0.9;
discount = 0.9;
init_lambda=rand(num_memory_states,act,size(obs_function,2));

for i=1:num_memory_states
        init_lambda(i,:,:)=init_lambda(i,:,:)/sum(sum(init_lambda(i,:,:)));
end

init_eta=rand(size(P_product,1),1);
init_V=zeros(size(P_product,1),1);

[lambda,V,eta]=CCCP(init_lambda,init_V,init_eta,P_nominal,P_product,absorb_original,absorb_product,target_original,target_product,init,init_reduced,unreachable,num_memory_states,cost,cost_lower_bound,Obs_Product,discount)

filename1 = ['lambda-' num2str(stringindicator) '.mat'];
filename2 = ['reach-' num2str(stringindicator) '.mat'];
filename3 = ['entropy-' num2str(stringindicator) '.mat'];
save(filename1,'lambda')
save(filename2,'eta')
save(filename3,'V')

stringindicator = stringindicator + 1;

% Simulate the resulting environment
T_horizon = 50;    % Time horizon to run simulation over
num_trials = 250;    % Number of iterations to test the policy
runSimulation_Gridworld1(lambda,P_matrix,num_memory_states,row,column,num_states,obs_function,T_horizon,num_trials,init,size(obs_function,2));     % Run the simulation




