% Converts points from (y,x) to state number 

%%% Inputs:
% row_number: Number of rows in the grid world (grid_world.m)
% array: a sequence of points [[y_1,x_1],[y_2,x_2],...]

%%% Outputs:
% states: a sequence of states [s_1,s_2,...]
function states=cartesiantoreal(row_number,array)
    n=size(array,1);
    states=zeros(n,1);
    for k=1:n
         states(k,1)=row_number*(array(k,1)-1)+array(k,2);
    end
end