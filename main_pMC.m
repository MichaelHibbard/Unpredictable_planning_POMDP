clear all
clc
%rng(0)
% Initial MDP
row=4;
col=6;
num_states=row*col;
act=4;
absorb=[1,col;2,1];
target=[1,col];
init=1;
P=grid_world2(row,col,act,[],absorb,0.99,0.0001);
% Finite state controller and the product MDP
num_memory_state=3;
FSC=FSC_absorbing(num_memory_state,act);
P_Product=product_MDP(P,FSC);
% Convert points to state numbers 
absorb=sort(cartesiantoreal(col,absorb));
target=sort(cartesiantoreal(col,target));
target_product=zeros(num_memory_state*length(target),1);
absorb_product=zeros(num_memory_state*length(absorb),1);


for k=1:length(absorb)
    absorb_product(num_memory_state*(k-1)+1:num_memory_state*k)=(0:num_memory_state-1)*num_states+absorb(k);
end

for k=1:length(target)
    target_product(num_memory_state*(k-1)+1:num_memory_state*k)=(0:num_memory_state-1)*num_states+target(k);
end

% Find and remove unreachable states
P_nominal=P_Product;
[reachable,~]=find_reachable(P_Product,init);
unreachable=sort(setdiff(1:size(P_Product,1),reachable),'descend');
P_Product(unreachable,:,:)=[];
P_Product(:,unreachable,:)=[];
%[~,G]=find_reachable(P_Product,init);
%plot(G,'Layout','layered')

size(P_Product)

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
disp(absorb_product)

% Define the cost function
cost=zeros(size(P_Product,1),act);
for i=1:size(P_Product,1)
    if ~ismember(i,target_product)
        for j=1:act
             cost(i,j)=P_Product(i,target_product,j);
        end
    end
end


cost_lower_bound=0.95;
init_lambda=rand(num_memory_state,act);
for i=1:num_memory_state
    init_lambda(i,:)=init_lambda(i,:)/sum(init_lambda(i,:));
end



%init_lambda=ones(num_memory_state,act)/4;

init_eta=zeros(size(P_Product,1),1);
init_V=zeros(size(P_Product,1),1);

[lambda,V,eta]=CCCP(init_lambda,init_V,init_eta,P_nominal,P_Product,absorb_original,absorb_product,target_original,target_product,init,unreachable,num_memory_state,cost,cost_lower_bound)


