function P_product=product_MDP(TF,FSC)
    num_states=size(TF,1);
    num_actions=size(TF,3);
    num_memory=size(FSC,1);
    
    P_product=zeros(num_states*num_memory,num_states*num_memory,num_actions);
    for memory=1:num_memory
        for state=1:num_states
            for act=1:num_actions
                for succ_mem=1:num_memory
                    P_product(num_states*(memory-1)+state,num_states*(succ_mem-1)+1:num_states*succ_mem,act)...
                        =TF(state,:,act)*FSC(memory,succ_mem,act);
                end
            end
        end
    end
end