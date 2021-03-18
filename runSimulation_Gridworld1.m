function runSimulation_Gridworld1(lambda,P,num_memory_states,row,column,num_states,obs_function,T_horizon,num_trials,init,num_Obs)
% Run the entropy maximization simulation on a Gridworld. Assumes
% observations are error state left, right, up, down, and none.


% matrix storing state residences over successive trials
simulationMatrix = zeros(num_trials,T_horizon,num_states);
% initial state is constant across all trials
simulationMatrix(:,1,init) = 1;

% create a vector for memory states. Use fact that transitions are
% deterministic and the final state self-loops.
current_memory_state = zeros(T_horizon,1);
current_memory_state(1:num_memory_states,1) = 1:1:num_memory_states;
current_memory_state(num_memory_states+1:end,1) = num_memory_states;

for trial = 1:num_trials
    for time = 1:T_horizon-1
        % Access the current state and memory node
        currentState = find(simulationMatrix(trial,time,:));
        % generate random values for observation made, action selection,
        % and transition probability.
        rand_obs = rand();
        rand_act = rand();
        rand_transProb = rand();
        
        % Determine the next observation, action, and resulting state
        if rand_obs < sum(obs_function(currentState,1))
            if rand_act < lambda(current_memory_state(time),1,1) 
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,1))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:2,1))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,2))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:3,1))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,3))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            else
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,4))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            end
        elseif rand_obs < sum(obs_function(currentState,1:2))
            if rand_act < lambda(current_memory_state(time),1,2)
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,1))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:2,2))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,2))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:3,2))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,3))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            else
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,4))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            end
        elseif rand_obs < sum(obs_function(currentState,1:3))
            if rand_act < lambda(current_memory_state(time),1,3)
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,1))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:2,3))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,2))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:3,3))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,3))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            else
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,4))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            end
        elseif rand_obs < sum(obs_function(currentState,1:4))
            if rand_act < lambda(current_memory_state(time),1,4)
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,1))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:2,4))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,2))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:3,4))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,3))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            else
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,4))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            end
            %
        elseif rand_obs < sum(obs_function(currentState,1:5))
            if rand_act < lambda(current_memory_state(time),1,5)
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,1))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:2,5))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,2))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:3,5))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,3))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            else
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,4))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            end
            %
        elseif rand_obs < sum(obs_function(currentState,1:6))
            if rand_act < lambda(current_memory_state(time),1,6)
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,1))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:2,6))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,2))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:3,6))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,3))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            else
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,4))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            end
            %
        elseif rand_obs < sum(obs_function(currentState,1:7))
            if rand_act < lambda(current_memory_state(time),1,7)
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,1))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:2,7))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,2))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:3,7))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,3))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            else
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,4))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            end
            %
        elseif rand_obs < sum(obs_function(currentState,1:8))
            if rand_act < lambda(current_memory_state(time),1,8)
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,1))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:2,8))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,2))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:3,8))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,3))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            else
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,4))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            end
            %
        else
            if rand_act < lambda(current_memory_state(time),1,9)
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,1))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:2,9))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,2))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            elseif rand_act < sum(lambda(current_memory_state(time),1:3,9))
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,3))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            else
                for nextState = 1:num_states
                    if rand_transProb < sum(P(currentState,1:nextState,4))
                        simulationMatrix(trial,time+1,nextState)=1;
                        break
                    end
                end
            end
        end
    end
end

% Create a gif showing the residences over time
% Combine all of the trials into a single heatmap
numStaNom = row*column;
combinedProbs = zeros(T_horizon,numStaNom);
for time=1:T_horizon
    for state=1:row*column
        combinedProbs(time,state) = sum(simulationMatrix(:,time,state))/num_trials;
    end
end

file_name = 'stateDistOverTime.gif';
% Create a gif and store it for each time step
set(0,'DefaultFigureVisible','off')
for time=1:T_horizon
    
    gridWorldProbs=reshape(combinedProbs(time,:),[row,column])';
    heatmap(gridWorldProbs,'ColorbarVisible','off','Title','State Distribution over Time');
    x1 = gcf;
    
    frame = getframe(x1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if time==1
        imwrite(imind,cm,file_name,'gif','Loopcount',inf);
    else
        imwrite(imind,cm,file_name,'gif','WriteMode','append');
    end
    
end
set(0,'DefaultFigureVisible','on')
end

