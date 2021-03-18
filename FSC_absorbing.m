function P=FSC_absorbing(num_states,num_act)
    P=zeros(num_states,num_states,num_act);
    for state=1:num_states-1
        P(state,state+1,:)=ones(num_act,1);
    end
    P(num_states,num_states,:)=ones(num_act,1);
end