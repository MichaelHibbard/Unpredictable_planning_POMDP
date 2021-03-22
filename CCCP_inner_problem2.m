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
        if ismember(s,absorb) % if the state is absorbing but not target do the following
            
            % Save to list, but can do all of these at once later on.
            state_counter_absorb_list = [state_counter_absorb_list, state_counter];
            state_counter=state_counter+1;
            
        elseif ismember(s,target) % if the state s is target state
            
            state_counter_target_list = [state_counter_target_list, state_counter];
            state_counter=state_counter+1; % increase the counter
            
        else   % For all other reachable states do the following
            
            oo = find(obsFunction(state_counter,:));    % Non-zero observations are constant for a fixed state.
            successors=find(sum(TF_reduced(state_counter,:,:),3)); % find successor states
            
            % stack_cons_reach=0; % we will add all reachability constraints to this stack
            % stack_cons_entr=0; % we will add all entropy constraints to this stack
            % target_successor_stack=0; % if the successor is target state, we will use this stack
            
            % Group the successor states into their associated type
            target_succs = intersect(successors,target_product);
            absorb_succs = intersect(successors,absorb_product);
            gen_succs = setdiff(successors,union(target_succs,absorb_succs));
            
            % NOTE: I BELIEVE THAT MORE CARE MUST BE TAKEN IF OBSERVATIONS
            % ARE NOT DETERMINISTIC
            
            % Target states
            if length(target_succs) == 1 
                target_successors = squeeze(obsFunction(state_counter,oo))*squeeze(lambda(memory_node,:,oo))*squeeze(TF_reduced(state_counter,target_succs,:));
                target_successor_stack = sum(target_successors);
            elseif length(target_succs) > 1
                target_successors = squeeze(obsFunction(state_counter,oo))*squeeze(lambda(memory_node,:,oo))*(squeeze(TF_reduced(state_counter,target_succs,:))');
                target_successor_stack = sum(target_successors);
            else
                target_successors = [];
                target_successor_stack = 0;
            end
            
            % Absorbing states
            if length(absorb_succs) == 1
                absorb_successors = squeeze(obsFunction(state_counter,oo))*(squeeze(lambda(memory_node,:,oo)))*squeeze(TF_reduced(state_counter,absorb_succs,:));
            elseif length(absorb_succs) > 1
                absorb_successors = squeeze(obsFunction(state_counter,oo))*(squeeze(lambda(memory_node,:,oo)))*(squeeze(TF_reduced(state_counter,absorb_succs,:))');
            else
                absorb_successors = [];
            end
            
            if ~isempty(gen_succs)
                % find P(s,s') by summing up over all actions
                P_s_succ = squeeze(obsFunction(state_counter,oo))*(squeeze(lambda(memory_node,:,oo)))*squeeze(TF_reduced(state_counter,gen_succs,:))';
                % Find P(s,s') using the initial lambda values around which we perform the linearization.
                P_s_succ_init = squeeze(obsFunction(state_counter,oo))*(squeeze(init_lambda(memory_node,:,oo)))*squeeze(TF_reduced(state_counter,gen_succs,:))';
                
                % Now we find convex and concave parts of the P(s,a,s')lambda(q,a)eta(s). Note that both lambda(s,a) and eta(s) are variables.
                % write the term as difference of convex functions f_1(x)-f_2(x). Then linearize the concave part where we use initial values around which the linearization is performed !!
                convex_part=0.5*square(P_s_succ'-eta(gen_succs));
                concave_part=0.5*P_s_succ_init'.*P_s_succ_init'-P_s_succ_init'.*P_s_succ'+0.5*init_eta(gen_succs).*init_eta(gen_succs)-eta(gen_succs).*init_eta(gen_succs);
                stack_cons_reach = sum(convex_part) + sum(concave_part);

                % Repeat difference-of-convex process
                % for P(s,a,s')lambda(s,a)eta(s).
                convex_part_entropy = 0.5*square(P_s_succ'-V(gen_succs));
                concave_part_entropy = 0.5*P_s_succ_init'.*P_s_succ_init'-P_s_succ_init'.*P_s_succ'+0.5*init_V(gen_succs).*init_V(gen_succs)-V(gen_succs).*init_V(gen_succs);
                stack_cons_entr = sum(convex_part_entropy) + sum(concave_part_entropy);
            else
                P_s_succ = [];
                stack_cons_reach = 0;
                stack_cons_entr = 0;
            end
            
            % add P(s,s') to entropy vector
            entr_vec = [target_successors, absorb_successors, P_s_succ];
            
            % Encode that state transitions must be feasible probability
            % distributions. P_ss' = P_sspr
            P_sspr_list = [P_sspr_list, sum(obsFunction(state_counter,:)*(squeeze(lambda(memory_node,:,:))')*(squeeze(TF_reduced(state_counter,successors,:))'))];
            
            % Reachability constraint eta(s)<= \sum_s' P(s,s')eta (s')
            % where P(s,s')=\sum_a P(s,a',s')lambda(q,a)
            eta(state_counter)+stack_cons_reach-target_successor_stack<=slack_2(state_counter);
            V(state_counter)+stack_cons_entr-log2(exp(1))*sum(entr(entr_vec))<= slack_1(state_counter);
            
            if ismember(state_counter,init_reduced)
                val = stack_cons_reach;
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
disp(toc-tic)

end