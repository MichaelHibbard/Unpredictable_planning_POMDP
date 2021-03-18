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

for s=1:size(TF,1) % To keep track of memory shifts, we use original TF
    
    if ~ismember(s,unreachable) % if the state is unreachable, skip. % dont forget to increase state counter for each reachable state
        
        % REMEMBER that the rest of the code will run if s is reachable!!!!
        if ismember(s,absorb) % if the state is absorbing but not target do the following
            % set everything to zero and increase the state
            % counter since you have counted one more reachable
            % state
            slack_2(state_counter)==0;
            slack_1(state_counter)==0;
            eta(state_counter)==0;
            V(state_counter)==0;
            state_counter=state_counter+1;
            
        elseif ismember(s,target) % if the state s is target state
            slack_2(state_counter)==0;
            slack_1(state_counter)==0;
            eta(state_counter)==1; % set its reachability value to 1 !!
            V(state_counter)==0;
            state_counter=state_counter+1; % increase the counter
            
        else   % For all other reachable states do the following
            successors=find(sum(TF_reduced(state_counter,:,:),3)); % find successor states
            stack_cons_reach=0; % we will add all reachability constraints to this stack
            stack_cons_entr=0; % we will add all entropy constraints to this stack
            target_successor_stack=0; % if the successor is target state, we will use this stack
            entr_vec=[];
            for succ=successors % for each successor state do the following
                P_s_succ=0;
                P_s_succ_init=0;
                target_successor=0;
                absorb_successor=0;
                if ismember(succ,target_product) % if the successor is a reachable target state
                    for obs=1:num_obs
                        if obsFunction(state_counter,obs)~=0
                            for aa=1:num_actions % for each actions of the state
                                if TF_reduced(state_counter,succ,aa)~=0 % find the action that transitions to the target state
                                    % add the value P(s,a,s')lambda(q,a) to the reachability probability
                                    target_successor=target_successor+lambda(memory_node,aa,obs)*obsFunction(state_counter,obs)*TF_reduced(state_counter,succ,aa);
                                end
                            end
                        end
                    end
                    target_successor_stack=target_successor_stack+target_successor; % if there are other target successors, keep adding up.
                    % remember local entropy= \sum_s'
                    % (P(s,s')log P(s,s')). We find each
                    % P(s,s') and then keep them in an array to
                    % calculate the entropy of the array later
                    entr_vec=[entr_vec, target_successor];
                    
                    % If the successor is absorbing, we dont
                    % care about reachability constraint (since the reach value of absorbing is 0 !) but
                    % care about the entropy, i.e., P(s,s') value.
                elseif ismember(succ,absorb_product)
                    for obs=1:num_obs
                        if obsFunction(state_counter,obs)~=0
                            for aa=1:num_actions
                                if TF_reduced(state_counter,succ,aa)~=0
                                    absorb_successor=absorb_successor+lambda(memory_node,aa,obs)*obsFunction(state_counter,obs)*TF_reduced(state_counter,succ,aa);
                                end
                            end
                        end
                    end
                    % Add P(s,s') to the entropy array
                    entr_vec=[entr_vec, absorb_successor];
                
                else % if the successor state is neither absorbing nor target, then do the following
                    
                    for obs=1:num_obs
                        if obsFunction(state_counter,obs) ~= 0
                            for act=1:num_actions
                                if TF_reduced(state_counter,succ,act)~=0  % if the transition probability is not zero
                                    % find P(s,s') by summing up over all actions
                                    P_s_succ=P_s_succ+lambda(memory_node,act,obs)*obsFunction(state_counter,obs)*TF_reduced(state_counter,succ,act);
                                    % Find P(s,s') using the initial lambda values around which we perform the linearization.
                                    P_s_succ_init=P_s_succ_init+init_lambda(memory_node,act,obs)*obsFunction(state_counter,obs)*TF_reduced(state_counter,succ,act);
                                end
                            end
                        end
                    end
                    
                    % add P(s,s') to entropy vector
                    entr_vec=[entr_vec, P_s_succ];
                    act=0;
                    
                    % Now we find convex and concave parts of the P(s,a,s')lambda(q,a)eta(s). Note that both lambda(s,a) and eta(s) are variables.
                    % write the term as difference of convex functions f_1(x)-f_2(x). Then linearize the concave part where we use initial values around which the linearization is performed !!
                    convex_part=0.5*square(P_s_succ-eta(succ));
                    concave_part=0.5*P_s_succ_init*P_s_succ_init-P_s_succ_init*P_s_succ+0.5*init_eta(succ)*init_eta(succ)-eta(succ)*init_eta(succ);
                    stack_cons_reach=stack_cons_reach+convex_part+concave_part;
                    
                    % do the same difference of convex stuff
                    % for P(s,a,s')lambda(s,a)eta(s).
                    convex_part_entropy=0.5*square(P_s_succ-V(succ));
                    concave_part_entropy=0.5*P_s_succ_init*P_s_succ_init-P_s_succ_init*P_s_succ+0.5*init_V(succ)*init_V(succ)-V(succ)*init_V(succ);
                    stack_cons_entr=stack_cons_entr+convex_part_entropy+concave_part_entropy;
                    
                end

            end
            
            % Encode that state transitions must be feasible probability
            % distributions. P_ss' = P_sspr
            P_sspr = 0;
            for succ=successors
                for obs=1:num_obs
                    for action=1:num_actions
                        P_sspr = P_sspr + lambda(memory_node,action,obs)*obsFunction(state_counter,obs)*TF_reduced(state_counter,succ,action);
                    end
                end
            end
            P_sspr == 1;
            
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

% Explicity write this in terms of underlying variables.
%             for k=1:num_memory
%                 sum(lambda(k,:))==1;
%             end

% Explicitly write this in terms of underlying variables. Don't
% need to?
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
end