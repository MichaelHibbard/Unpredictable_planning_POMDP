function [val,val2Final]=verify_reach(lambda,P_Product,init,init_reduced,num_memory,absorb_product,target,target_product,obsFunction,discount)
num_states=size(P_Product,1)/num_memory;
Induced_pMC=zeros(size(P_Product,1),size(P_Product,2));
active_memory=1;
for i=1:size(P_Product,1)
    for j=1:size(P_Product,2)
        for o=1:size(obsFunction,2)
            %Induced_pMC(i,j)=Induced_pMC(i,j) + obsFunction(i,o)*dot(lambda(active_memory,:,o),squeeze(P_Product(i,j,:)));
            for a = 1:size(P_Product,3)
                Induced_pMC(i,j)=Induced_pMC(i,j) + obsFunction(i,o)*lambda(active_memory,a,o)*P_Product(i,j,a);
            end
        end
    end
    if rem(i,num_states)==0
        active_memory=active_memory+1;
    end
end
[reachable,~]=find_reachable(Induced_pMC,init);
unreach=sort(setdiff(1:size(Induced_pMC,1),reachable),'descend');
Induced_pMC(unreach,:,:)=[];
Induced_pMC(:,unreach,:)=[];
%[~,G]=find_reachable(P_Product,init);
%plot(G,'Layout','layered')
% Update the target states on the product MDP according to removed states
absorb_product=vertcat(absorb_product,target);
absorb_product=absorb_product(~ismember(absorb_product,unreach));
absorb_product=sort(absorb_product,'descend');

for k=1:length(absorb_product)
    num_smaller_states=unreach(unreach<absorb_product(k));
    absorb_product(k)=absorb_product(k)-length(num_smaller_states);
end
reachable_target=target_product(ismember(target_product,absorb_product));

cost=zeros(size(Induced_pMC,1),1);
for i=1:size(Induced_pMC,1)
    cost(i,1)=cost(i,1)+sum(Induced_pMC(i,reachable_target));
end

% Induced_pMC(target,:,:)=[];
% Induced_pMC(:,target,:)=[];
% cost(target)=[];
Induced_pMC(absorb_product,:,:)=[];
Induced_pMC(:,absorb_product,:)=[];
cost(absorb_product)=[];
inverse_matrix=(eye(size(Induced_pMC,1))-Induced_pMC)\cost;
val=inverse_matrix(init_reduced);
%disp(inverse_matrix)

% Also try verifying using value iteration
val2 = zeros(num_states,1);
val2New = zeros(num_states,1);
val2(target_product) = 1;
maxError = 1;
while maxError >= 1e-6
    for state = 1:num_states
        if ~ismember(state,target)
            reward = 0;
            for succ = 1:num_states
                for act = 1:size(P_Product,3)
                    for obs = 1:size(obsFunction,2)
                        reward = reward + lambda(1,act,obs)*P_Product(state,succ,act)*obsFunction(state,obs)*val2(succ);
                    end
                end
            end
            val2New(state,1) = reward;
        else
            val2New(state,1) = 1;
        end
    end
    maxError = max(abs(val2 - val2New));
    val2(:,1) = val2New(:,1);
end
val2Final = val2(init_reduced,1);

