function obsFunction = CreateObservationMatrix1(row,column,absorb,target)
% This function is used to construct the observation function for a
% gridworld with error states. The observation (in its current form) is
% constructed such that the agent deterministically makes one of the
% following observations: no absorbing states, absorbing state down,
% absorbing state up, absorbing state up, absorbing state left, absorbing
% state right. If there are n absorbing states in the vicinity, makes
% observation with probability 1/n. [no obs, down, up, left, right]
% nObs = 9;
% num_states = row*column;
% obsFunction = zeros(num_states,nObs);
% 
% % Go through all states and set possible observations
% for i=1:row
%     for j=1:column
%         
%         counter = 0;
%         
%         if ismember([i+1,j],absorb,'rows')
%             if ismember([i+1,j],target,'rows')
%                 % If target state below
%                 obsFunction(column*(i-1)+j,6)=1;
%                 counter = counter + 1;
%             else
%                 % If absorbing state below
%                 obsFunction(column*(i-1)+j,2)=1;
%                 counter = counter + 1;
%             end
%         end
%         
%         if ismember([i-1,j],absorb,'rows')
%             if ismember([i-1,j],target,'rows')
%                 % If target state above
%                 obsFunction(column*(i-1)+j,7)=1;
%                 counter = counter + 1;
%             else
%                 % If absorbing state above
%                 obsFunction(column*(i-1)+j,3)=1;
%                 counter = counter + 1;
%             end
%         end
%         
%         if ismember([i,j-1],absorb,'rows')
%             if ismember([i,j-1],target,'rows')
%                 % If target state to the left
%                 obsFunction(column*(i-1)+j,8)=1;
%                 counter = counter + 1;
%             else
%                 disp(i)
%                 disp(j-1)
%                 disp(column*(i-1)+j)
%                 % If absorbing state to the left
%                 obsFunction(column*(i-1)+j,4)=1;
%                 counter = counter + 1;
%             end
%         end
%         
%         if ismember([i,j+1],absorb,'rows')
%             if ismember([i,j+1],target,'rows')
%                 % If target state to the right
%                 obsFunction(column*(i-1)+j,9)=1;
%                 counter = counter + 1;
%             else
%                 % If absorbing state to the right
%                 obsFunction(column*(i-1)+j,5)=1;
%                 counter = counter + 1;
%             end
%         end
%         
%         if counter == 0
%             % If no absorbing states surrounding
%             obsFunction(column*(i-1)+j,1)=1;
%         end
%         
%     end
% end

% NOW: ASSUME THAT THERE ARE ONLY 7 OBSERVATIONS (BECAUSE WE WILL NEVER
% OBSERVE A TARGET ABOVE OR A TARGET TO THE LEFT BASED ON THE 5x5 EXAMPLE
% THAT WE ARE USING.
nObs = 7;
num_states = row*column;
obsFunction = zeros(num_states,nObs);

% Go through all states and set possible observations
for i=1:row
    for j=1:column
        
        counter = 0;
        
        if ismember([i+1,j],absorb,'rows')
            if ismember([i+1,j],target,'rows')
                % If target state below
                obsFunction(column*(i-1)+j,6)=1;
                counter = counter + 1;
            else
                % If absorbing state below
                obsFunction(column*(i-1)+j,2)=1;
                counter = counter + 1;
            end
        end
        
        if ismember([i-1,j],absorb,'rows')
            if ismember([i-1,j],target,'rows')
%                 % If target state above
%                 obsFunction(column*(i-1)+j,7)=1;
%                 counter = counter + 1;
            else
                % If absorbing state above
                obsFunction(column*(i-1)+j,3)=1;
                counter = counter + 1;
            end
        end
        
        if ismember([i,j-1],absorb,'rows')
            if ismember([i,j-1],target,'rows')
%                 % If target state to the left
%                 obsFunction(column*(i-1)+j,8)=1;
%                 counter = counter + 1;
            else
                % If absorbing state to the left
                obsFunction(column*(i-1)+j,4)=1;
                counter = counter + 1;
            end
        end
        
        if ismember([i,j+1],absorb,'rows')
            if ismember([i,j+1],target,'rows')
                % If target state to the right
                obsFunction(column*(i-1)+j,7)=1;
                counter = counter + 1;
            else
                % If absorbing state to the right
                obsFunction(column*(i-1)+j,5)=1;
                counter = counter + 1;
            end
        end
        
        if counter == 0
            % If no absorbing states surrounding
            obsFunction(column*(i-1)+j,1)=1;
        end
        
    end
end

% Probability of making observation
for state=1:num_states
    obsFunction(state,:) = obsFunction(state,:)./sum(obsFunction(state,:));
end
    
end