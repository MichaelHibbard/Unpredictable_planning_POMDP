cvx_solver Mosek
cvx_begin quiet

variables x(100)

x <= 1;
x >= 0;

tic;
constant = 0;
for i = 1:100
    constant = constant + square(x(i));
end
toc
% tic;
% constant = [];
% for i = 1:100
%     constant = [constant,x(i)];
% end
% constant2 = sum_square(constant);
% toc

minimize(constant)
cvx_end

disp(constant2)


