cvx_begin
variable x(2,3)
x>=0
sum(x(1,:))==1;
sum(x(2,:))==1;
t=0
t=t+x(1,3)+x(2,1);
disp(t)
t<=0.6;

t=0;

t=t+x(1,2)+x(2,2);
t<=0.3;
disp(t)

minimize(x(2,3)+x(1,2)-4)
cvx_end
x