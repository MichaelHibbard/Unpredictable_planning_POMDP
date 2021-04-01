function [lambda,eta,V,slack_2,slack_1,cvx_optval]=CCCP_inner_problem2(tau,init_lambda,init_V,init_eta,TF,TF_reduced,absorb,absorb_product,target,target_product,init,init_reduced,unreachable,num_memory,cost,cost_bound,obsFunction,discount)
% We will use reduced transition function to define optimization variables.
% To keep track of what is reachable and what is not, we use both reduced
% and full transition matrices.

num_nominal_states=size(TF,1)/num_memory; % number of states in POMDP
num_states=size(TF_reduced,1); % Number of REACHABLE states in the product
num_actions=size(TF_reduced,3); % number of actions
num_obs=size(obsFunction,2);    % number of observations
cvx_solver MOSEK
cvx_begin quiet

% Define variables using REACHABLE STATES in the PRODUCT!!
variables lambda(num_memory,num_actions,num_obs) eta(num_states) V(num_states) slack_2(num_states) slack_1(num_states)
memory_node=1; % counter for memory states
state_counter=1; % counter for reachable states

% disp(num_nominal_states)
% disp(num_states)
% disp(num_obs)

tic;

state_counter_absorb_list = [];
state_counter_target_list = [];
P_sspr_list = [];

for s=1:size(TF,1) % To keep track of memory shifts, we use original TF
    
    % if the state is unreachable, skip.
    % dont forget to increase state counter for each reachable state
    if ~ismember(s,unreachable)
        
        % REMEMBER that the rest of the code will run if s is reachable!!!!
        if ismember(s,target) % if the state s is target state
            
            state_counter_target_list = [state_counter_target_list, state_counter];
            state_counter=state_counter+1; % increase the counter
            
        elseif ismember(s,absorb) % if the state is absorbing but not target do the following
            
            % Save to list, but can do all of these at once later on.
            state_counter_absorb_list = [state_counter_absorb_list, state_counter];
            state_counter=state_counter+1;
            
        else   % For all other reachable states do the following
            
            oo = find(obsFunction(state_counter,:));    % Non-zero observations are constant for a fixed state.
            successors=find(sum(TF_reduced(state_counter,:,:),3)); % find successor states for a fixed state
            
            % stack_cons_reach=0; % we will add all reachability constraints to this stack
            % stack_cons_entr=0; % we will add all entropy constraints to this stack
            % target_successor_stack=0; % if the successor is target state, we will use this stack
            
            % Group the successor states into their associated type
            target_succs = intersect(successors,target_product);
            absorb_succs = setdiff(intersect(successors,absorb_product),target_succs); % Target states are also absorbing - need to be careful!
            gen_succs = setdiff(successors,union(target_succs,absorb_succs));
            
            % May not be necessary but want to be sure of its shape
            obs_vector = reshape(obsFunction(state_counter,oo),1,length(oo));
            lambda_matrix = reshape(lambda(memory_node,:,oo),num_actions,length(oo));
            init_lambda_matrix = reshape(init_lambda(memory_node,:,oo),num_actions,length(oo));
            target_matrix = reshape(TF_reduced(state_counter,target_succs,:),length(target_succs),num_actions);
            absorb_matrix = reshape(TF_reduced(state_counter,absorb_succs,:),length(absorb_succs),num_actions);
            gen_matrix = reshape(TF_reduced(state_counter,gen_succs,:),length(gen_succs),num_actions);
            
            % NOTE: I BELIEVE THAT MORE CARE MUST BE TAKEN IF OBSERVATIONS
            % ARE NOT DETERMINISTIC
            
            % Target states
            if length(target_succs) >= 1
                target_successors = obs_vector*lambda_matrix'*target_matrix';
                target_successor_stack = sum(target_successors);
            else
                target_successors = [];
                target_successor_stack = 0;
            end
            
            % Absorbing states
            if length(absorb_succs) >= 1
                absorb_successors = obs_vector*lambda_matrix'*absorb_matrix';
            else
                absorb_successors = [];
            end
            
            % General successor states
            if length(gen_succs) >= 1
                
                % First, we need to construct the stacks of variables. Can
                % loop through here (should be efficient, simply appending
                % to lists, not performing any CVX operations)
                c_stack = [];
                lam_stack = [];
                lam_init_stack = [];
                reach_stack = [];
                reach_init_stack = [];
                entr_stack = [];
                entr_init_stack = [];
                for succ = gen_succs
                    c_stack_cur = (1/2)*discount*reshape(TF_reduced(state_counter,succ,:),num_actions,1);
                    c_stack = [c_stack;c_stack_cur];
                    lam_stack_cur = reshape(lambda(memory_node,:,oo),num_actions,1);
                    lam_stack = [lam_stack;lam_stack_cur];
                    lam_init_stack_cur = reshape(init_lambda(memory_node,:,oo),num_actions,1);
                    lam_init_stack = [lam_init_stack;lam_init_stack_cur];
                    reach_stack_cur = repmat(eta(succ),num_actions,1);
                    reach_stack = [reach_stack;reach_stack_cur];
                    reach_init_stack_cur = repmat(init_eta(succ),num_actions,1);
                    reach_init_stack = [reach_init_stack;reach_init_stack_cur];
                    entr_stack_cur = repmat(V(succ),num_actions,1);
                    entr_stack = [entr_stack;entr_stack_cur];
                    entr_init_stack_cur = repmat(init_V(succ),num_actions,1);
                    entr_init_stack = [entr_init_stack;entr_init_stack_cur];
                end
                
                %                 convexified_entropy_stack = c_stack.*(entr_init_stack + lam_init_stack).^2 ...
                %                                         - c_stack.*(entr_stack.^2 + lam_stack.^2) ...
                %                                         + 2*c_stack.*(entr_init_stack + lam_init_stack).*(entr_stack - entr_init_stack) ...
                %                                         + 2*c_stack.*(entr_init_stack + lam_init_stack).*(lam_stack - lam_init_stack);
                %                 convexified_reach_stack = c_stack.*(reach_init_stack + lam_init_stack).^2 ...
                %                                         - c_stack.*(reach_stack.^2 + lam_stack.^2) ...
                %                                         + 2*c_stack.*(reach_init_stack + lam_init_stack).*(reach_stack - reach_init_stack) ...
                %                                         + 2*c_stack.*(reach_init_stack + lam_init_stack).*(lam_stack - lam_init_stack);
                convexified_entropy_stack = c_stack.*(entr_init_stack + lam_init_stack).^2 ...
                    - sum_square((c_stack.^(1/2)).*entr_stack) - sum_square((c_stack.^(1/2)).*lam_stack) ...
                    + 2*c_stack.*(entr_init_stack + lam_init_stack).*(entr_stack - entr_init_stack) ...
                    + 2*c_stack.*(entr_init_stack + lam_init_stack).*(lam_stack - lam_init_stack);
                convexified_reach_stack = sum(c_stack.*(reach_init_stack + lam_init_stack).^2) ...
                    - sum_square((c_stack.^(1/2)).*reach_stack) - sum_square((c_stack.^(1/2)).*lam_stack) ...
                    + sum(2*c_stack.*(reach_init_stack + lam_init_stack).*(reach_stack - reach_init_stack)) ...
                    + sum(2*c_stack.*(reach_init_stack + lam_init_stack).*(lam_stack - lam_init_stack));
                
                % find P(s,s') by summing up over all actions
                P_s_succ = obs_vector*lambda_matrix'*gen_matrix';
                
            else
                P_s_succ = [];
                %                 stack_cons_reach = 0;
                %                 stack_cons_entr = 0;
                convexified_reach_stack = 0;
                convexified_entropy_stack = 0;
            end
            
            % add P(s,s') to entropy vector
            entr_vec = [target_successors, absorb_successors, P_s_succ];
            
            % Encode that state transitions must be feasible probability
            % distributions. P_ss' = P_sspr
            P_sspr_list = [P_sspr_list, sum(obsFunction(state_counter,:)*(squeeze(lambda(memory_node,:,:))')*(squeeze(TF_reduced(state_counter,successors,:))'))];
            
            % Reachability constraint eta(s)<= \sum_s' P(s,s')eta (s')
            % where P(s,s')=\sum_a P(s,a',s')lambda(q,a)
            %             eta(state_counter)+stack_cons_reach-target_successor_stack<=slack_2(state_counter);
            %             V(state_counter)+stack_cons_entr-log2(exp(1))*sum(entr(entr_vec))<= slack_1(state_counter);
            eta(state_counter) - sum(convexified_reach_stack) - target_successor_stack <= slack_2(state_counter);
            V(state_counter) - sum(convexified_entropy_stack) - log2(exp(1))*sum(entr(entr_vec)) <= slack_1(state_counter);
            
            if ismember(state_counter,init_reduced)
                % val = stack_cons_reach;
                val = sum(convexified_reach_stack);
            end
            
            state_counter=state_counter+1;
            
        end
        
    end
    
    if rem(s,num_nominal_states)==0
        memory_node=memory_node+1;
    end
    
end

% Constrain that transitions must be valid probability distributions
P_sspr_list == 1;

% Constraints shared by absorbing and target states
slack_2([state_counter_absorb_list,state_counter_target_list])==0;
slack_1([state_counter_absorb_list,state_counter_target_list])==0;
V([state_counter_absorb_list,state_counter_target_list])==0;

% Reach constraints different for absorbing / target
eta(state_counter_absorb_list)==0;
eta(state_counter_target_list)==1;

% Explicitly write this in terms of underlying variables.
lambda>=1e-5;

eta>=0;
slack_2>=0;
slack_1>=0;
eta(init_reduced)>=cost_bound;

eta<=1;
V<=10000;
V>=0;

% MURATS RESULTS:
% Change V(init_reduced) to eta(init_reduced)
% Uncomment the following line
% eta(init_reduced)<=cost_bound;
maximize(V(init_reduced)-tau*(sum(slack_1)+sum(slack_2)));
cvx_end
%disp(slack_2)
disp(val)
%disp('Value')
%disp(eta(init))
%  disp(dot(lambda(2,:),cost(4,:)));

toc;

end