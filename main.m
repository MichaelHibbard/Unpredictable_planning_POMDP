clear all
clc

row=5;
col=5;
num_states=row*col;
act=4;
absorb=[row,col;1,4];
target=[row,col];
P=grid_world(row,col,act,[],absorb,1);
init=zeros(num_states,1);
init(1)=1; 

absorb=sort(cartesiantoreal(row,absorb));
target=sort(cartesiantoreal(row,target));

cost=zeros(num_states,act);
cost_vector=zeros(num_states*act,1);
for i=1:row*col
    if ~ismember(i,target)
        for j=1:act
        cost(i,j)=P(i,target,j);
        end
    end
end

for k=1:act
    cost_vector(num_states*(k-1)+1:num_states*k)=cost(:,k);
end

[policy,lambda,cvx_optval]=min_cost_LP(P,init,absorb,target,cost_vector);


