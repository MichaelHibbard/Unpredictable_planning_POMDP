%%%% Convex concave procedure for unpredictable planning under partial observability
% Inputs:

% init_lambda: initial values for FSCs action selection function. For a
% k-FSC and a POMDP with n actions, init_lambda has size(k,n). 

% init_V: initial values for variables representing the entropy of states.
% For a POMDP with m states, init_V has size(m,1).

% init_eta: initial values for variables representing expected reward of
% states. For a POMDP with m states, init_eta has size(m,1).

% TF: transition function of POMDP. For a POMDP with m states and n actions
% TF has size(m,m,n). (state,state,action)

% TF_reduced: transition function of the product of POMDP and FSC.
% Unreachable states from the initial states are removed from the graph. If
% there are m reachable states and n actions, TF_reduced has size(m,m,n).

% absorb: absorbing states in POMDP excluding target states. 

% absorb_reduced: absorbing states in the product of POMDP and FSC.
% Unreachable states from the initial state are removed. Target states are
% excluded.

% target: target states in POMDP. 

% target_reduced: target states in the product of POMDP and FSC.
% Unreachable states from the initial state are removed. 

% init: initial state in POMDP. 

% init_reduced: initial state in the product of POMDP and FSC.

% unreachable: The set of unreachable states in the product of POMDP and FSC

% num_memory: Number of memory states in FSC.

% cost, cost_bound: WE DONT USE THEM. I AM JUST LAZY TO REMOVE THEM.

function [initial_lambda,initial_V,initial_eta]=CCCP(init_lambda,init_V,init_eta,TF,TF_reduced,absorb,absorb_product, target, target_product,init,init_reduced,unreachable,num_memory,cost,cost_bound,obsFunction,discount)
    %%%%%%%% PARAMETERS
    tau=0.5; %% use between 0.5-5
    multiplier=1.2; %% use between 1.2-1.7
    max_iter=150;
    %%%%%%%%%%%%%%%%%%%
    dummy_optval=-10; % assign an initial dummy optimal value for while loop
    epsilon=1e-6; % if the slack variable is less then epsilon, stop.
    tau_max=250; % maximum weighting on the slack variables
    initial_eta=init_eta; % initialize variables
    initial_lambda=init_lambda;
    initial_V=init_V;
    for k=1:max_iter
        % Solve the convexified problem and obtain the optimal values
        [lambda,eta,V,slack_2,slack_1,optval]=CCCP_inner_problem2(tau,initial_lambda,initial_V,initial_eta,TF,TF_reduced,absorb,absorb_product, target, target_product,init,init_reduced,unreachable,num_memory,cost,cost_bound,obsFunction,discount);
        initial_lambda=lambda; % update the initial points to optimal values of convexified problem
        initial_V=V;
        initial_eta=eta;
        fprintf('Num iteration: %d,  Residual %0.5f,  Reach %0.4f, Entropy %0.2f\n', k, tau*(sum(slack_2)+sum(slack_1)), eta(init_reduced), V(init_reduced))
        % disp(initial_lambda)
        if mod(k,25) == 0
            [reach_val,reach_val2]=verify_reach(lambda,TF,init,init_reduced,num_memory,absorb,target,target_product,obsFunction); % check real reachability value
            disp(reach_val)
            disp(reach_val2)
            disp('-----')
        end

         % stop if the stopping criteria is satisfied.
        if tau*(sum(slack_2)+sum(slack_1))<=epsilon && abs(optval-dummy_optval)<=10*epsilon
            break;
        end
        
        if mod(k,25) == 0
            disp(initial_lambda)
        end
        
        % update the optimal value of the program and the weighting of
        % slack variables.
        dummy_optval=optval;
        tau=min(tau*multiplier,tau_max);
    end
end