function P=Forward_MDP(row,col)
    Num_act=3;
    Num_states=row*col;
    P=zeros(Num_states,Num_states,Num_act);
    for i=1:col
        for j=1:row
            if i==col
                P((i-1)*row+j,(i-1)*row+j,1)=1;
                P((i-1)*row+j,(i-1)*row+j,2)=1;
                P((i-1)*row+j,(i-1)*row+j,3)=1;
            elseif j==1
                P((i-1)*row+j,i*row+j,1)=1;
                P((i-1)*row+j,i*row+j+1,2)=1;
                P((i-1)*row+j,(i-1)*row+j+1,3)=1;
            elseif j==row
                P((i-1)*row+j,i*row+j,1)=1;
                P((i-1)*row+j,i*row+j-1,2)=1;
                P((i-1)*row+j,(i-1)*row+j-1,3)=1;
            else
                P((i-1)*row+j,i*row+j-1,1)=1;
                P((i-1)*row+j,i*row+j,2)=1;
                P((i-1)*row+j,i*row+j+1,3)=1;
            end
        end
    end

