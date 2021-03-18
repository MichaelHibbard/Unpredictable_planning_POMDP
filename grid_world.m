%%% This function generates a grid world environment with given number of
%%% rows and columns. It start counting the rows starting from the top and
%%% the columns from the left. Actions can be chosen either 4 or 5, where
%%% the order is [left,right,up,down,stay]. Wall input is the location of
%%% walls such that [[row_1,col_1]; [row_2,col_2]]. Absorb input is the
%%% location of absorbing states such that [[row_1,col_1]; [row_2,col_2]].

function P=grid_world(row,column,act,wall,absorb,prob)
    P=zeros(row*column,row*column,act);
    for k=1:size(P,3)
      for i=1:row
        for j=1:column
            if k==1 % left action
                if j==1
                    P(column*(i-1)+j,column*(i-1)+j,k)=1;
                elseif i==1 
                    P(column*(i-1)+j,column*(i-1)+j-1,k)=prob+(1-prob)/2;
                    P(column*(i-1)+j,column*i+j-1,k)=(1-prob)/2;
                elseif i==row
                    P(column*(i-1)+j,column*(i-1)+j-1,k)=prob+(1-prob)/2;
                    P(column*(i-1)+j,column*(i-2)+j-1,k)=(1-prob)/2;
                else
                    P(column*(i-1)+j,column*(i-1)+j-1,k)=prob;
                    P(column*(i-1)+j,column*(i-2)+j-1,k)=(1-prob)/2;
                    P(column*(i-1)+j,column*(i)+j-1,k)=(1-prob)/2;  
                end
            elseif k==2 % right action
                if j==column
                    P(column*(i-1)+j,column*(i-1)+j,k)=1;
                elseif i==1 
                    P(column*(i-1)+j,column*(i-1)+j+1,k)=prob+(1-prob)/2;
                    P(column*(i-1)+j,column*i+j+1,k)=(1-prob)/2;
                elseif i==row
                    P(column*(i-1)+j,column*(i-1)+j+1,k)=prob+(1-prob)/2;
                    P(column*(i-1)+j,column*(i-2)+j+1,k)=(1-prob)/2;
                else
                    P(column*(i-1)+j,column*(i-1)+j+1,k)=prob;
                    P(column*(i-1)+j,column*(i-2)+j+1,k)=(1-prob)/2;
                    P(column*(i-1)+j,column*(i)+j+1,k)=(1-prob)/2;  
                end 
                
                
            elseif k==3 % up action
                if i==1
                    P(column*(i-1)+j,column*(i-1)+j,k)=1;
                elseif j==1 
                    P(column*(i-1)+j,column*(i-2)+j,k)=prob+(1-prob)/2;
                    P(column*(i-1)+j,column*(i-2)+j+1,k)=(1-prob)/2;
                elseif j==column
                    P(column*(i-1)+j,column*(i-2)+j,k)=prob+(1-prob)/2;
                    P(column*(i-1)+j,column*(i-2)+j-1,k)=(1-prob)/2;
                else
                    P(column*(i-1)+j,column*(i-2)+j,k)=prob;
                    P(column*(i-1)+j,column*(i-2)+j-1,k)=(1-prob)/2;
                    P(column*(i-1)+j,column*(i-2)+j+1,k)=(1-prob)/2;  
                end 
                
            elseif k==4 % down action
                if i==row
                    P(column*(i-1)+j,column*(i-1)+j,k)=1;
                elseif j==1 
                    P(column*(i-1)+j,column*(i)+j,k)=prob+(1-prob)/2;
                    P(column*(i-1)+j,column*(i)+j+1,k)=(1-prob)/2;
                elseif j==column
                    P(column*(i-1)+j,column*(i)+j,k)=prob+(1-prob)/2;
                    P(column*(i-1)+j,column*(i)+j-1,k)=(1-prob)/2;
                else
                    P(column*(i-1)+j,column*(i)+j,k)=prob;
                    P(column*(i-1)+j,column*(i)+j-1,k)=(1-prob)/2;
                    P(column*(i-1)+j,column*(i)+j+1,k)=(1-prob)/2;  
                end 
                
            else % stay action
                P(column*(i-1)+j,column*(i-1)+j,k)=1;
            end
        end
      end
    end
    % absorbing states
    for state=1:size(absorb,1)
        x=absorb(state,:);
        P(column*(x(1)-1)+x(2),:,:)=zeros(row*column,act);
        P(column*(x(1)-1)+x(2),column*(x(1)-1)+x(2),:)=ones(1,act);
        %P(row*(x(1)-1)+x(2),row*(x(1)-1)+x(2),1)=1;
    end
    % collision with wall
    for state=1:size(wall,1)
        x=wall(state,:);
        P(column*(x(1)-1)+x(2),:,:)=zeros(row*column,act);
        P(column*(x(1)-1)+x(2),column*(x(1)-1)+x(2),:)=ones(1,act);
        for k=1:act
            precessor=P(:,row*(x(1)-1)+x(2),k);
            for iterate=1:length(precessor)
                if precessor(iterate)~=0
                   dummy=P(iterate,column*(x(1)-1)+x(2),k);
                   P(iterate,column*(x(1)-1)+x(2),k)=0;
                   P(iterate,iterate,k)=P(iterate,iterate,k)+dummy;
                end
            end
        end      
    end
        
