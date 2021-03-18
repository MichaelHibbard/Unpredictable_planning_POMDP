function obs_Product = product_Obs( obsFunction, FSC )

num_states = size(obsFunction,1);
num_obs = size(obsFunction,2);
num_memory_states = size(FSC,1);

obs_Product = zeros(num_states*num_memory_states,num_obs);
for memoryState = 1:num_memory_states
    for state = 1:num_states
        obs_Product(num_states*(num_memory_states-1)+state,:) = obsFunction(state,:);
    end
end
end

