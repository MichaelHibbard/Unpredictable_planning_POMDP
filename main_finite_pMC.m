clear all
clc
%rng(0)
% Initial MDP
num_memory_state=2;
P=dummy_transition();
act=size(P,3);
target=[5];
absorb=[4,5,6];
init=1;
num_states=size(P,1);
% Finite state controller and the product MDP
FSC=FSC_absorbing(num_memory_state,act);
P_Product=product_MDP(P,FSC);
target_product=zeros(num_memory_state*length(target),1);
absorb_product=zeros(num_memory_state*length(absorb),1);
for k=1:length(target)
    target_product(num_memory_state*(k-1)+1:num_memory_state*k)=(0:num_memory_state-1)*num_states+target(k);
end

for k=1:length(absorb)
    absorb_product(num_memory_state*(k-1)+1:num_memory_state*k)=(0:num_memory_state-1)*num_states+absorb(k);
end
% Find and remove unreachable states
P_nominal=P_Product;
[reachable,~]=find_reachable(P_Product,init);
unreachable=sort(setdiff(1:size(P_Product,1),reachable),'descend');
disp('here')

P_Product(unreachable,:,:)=[];
P_Product(:,unreachable,:)=[];
%[~,G]=find_reachable(P_Product,init);
%plot(G,'Layout','layered')

size(P_Product)

Obs_Product = ones(num_memory_state*num_states,1);

% Update the target states on the product MDP according to removed states
target_product=target_product(~ismember(target_product,unreachable));
absorb_product=absorb_product(~ismember(absorb_product,unreachable));
absorb_product=sort(setdiff(absorb_product,target_product),'descend');
absorb_original=absorb_product;
target_original=target_product;

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

% Define the cost function
cost=zeros(size(P_Product,1),act);
for i=1:size(P_Product,1)
    if ~ismember(i,target_product)
        for j=1:act
             cost(i,j)=P_Product(i,target_product,j);
        end
    end
end

%%%%%%%%%%%%%%% REACHABILITY PROBABILITY
cost_lower_bound=1-1e-4;
%%%%%%%%%%%%%%%%
init_lambda=rand(num_memory_state,act);
for i=1:num_memory_state
    init_lambda(i,:)=init_lambda(i,:)/sum(init_lambda(i,:));
end


init_eta=rand(size(P_Product,1),1);
init_V=rand(size(P_Product,1),1);

disp(absorb_product)
disp(target_product)

[lambda,V,eta]=CCCP(init_lambda,init_V,init_eta,P_nominal,P_Product,absorb_original,absorb_product,target_original,target_product,init,init_reduced,unreachable,num_memory_state,cost,cost_lower_bound,Obs_Product)

entropyMurat=lambda(1,1)*log2(lambda(1,1))+lambda(1,2)*log2(lambda(1,2))+lambda(2,1)*log2(lambda(2,1))+lambda(2,2)*log2(lambda(2,2));
disp('Final maximum entropy using Murats results')
disp(entropyMurat)
