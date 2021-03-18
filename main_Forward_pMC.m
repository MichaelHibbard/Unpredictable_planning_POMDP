clear all
clc
%rng(0)
% Initial MDP

%%%%%%%% PARAMETERS
row=3;
col=5;
init=round(row/2);
target=[row*(col-1)+round(row/2)]; % use sth like row*(col-1)+K
num_memory_state=1;
%%%%%%%%%%

P=Forward_MDP(row,col);
act=size(P,3);
absorb=[row*(col-1)+1:1:row*col];
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
disp(unreachable)

P_Product(unreachable,:,:)=[];
P_Product(:,unreachable,:)=[];
%[~,G]=find_reachable(P_Product,init);
%plot(G,'Layout','layered')

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
             cost(i,j)=cost(i,j)+sum(P_Product(i,target_product,j));
        end
    end
end

disp(target_product)
disp(absorb_product)

cost_lower_bound=1-1e-4;
init_lambda=rand(num_memory_state,act);
for i=1:num_memory_state
    init_lambda(i,:)=init_lambda(i,:)/sum(init_lambda(i,:));
end


init_eta=rand(size(P_Product,1),1);
init_V=zeros(size(P_Product,1),1);

[lambda,V,eta]=CCCP(init_lambda,init_V,init_eta,P_nominal,P_Product,absorb_original,absorb_product,target_original,target_product,init,init_reduced,unreachable,num_memory_state,cost,cost_lower_bound,eye(13),1)



