function A = world(x_dim, y_dim, x_div, y_div, rows, cols)

%%
% rows = 12;      cols = 9;
% x_dim = 0.152;
% y_dim = 0.152;
% y_div = [0.152 0.152 0.178 0.152 0.152 0.178 0.152 0.152];
% x_div = 0.152 * ones(11,1)';

%%
p0 = zeros(rows,cols,2);
p1 = zeros(rows,cols,2);
p2 = zeros(rows,cols,2);
p3 = zeros(rows,cols,2);
p4 = zeros(rows,cols,2);

for i = 1:rows
    p0(i,:,1) = i*x_dim - x_dim/2 + sum(x_div(1:i-1));
    p1(i,:,1) = i*x_dim + sum(x_div(1:i-1));
    p2(i,:,1) = i*x_dim + sum(x_div(1:i-1));
    p3(i,:,1) = i*x_dim - x_dim + sum(x_div(1:i-1));
    p4(i,:,1) = i*x_dim - x_dim + sum(x_div(1:i-1));
end

for j = 1:cols
    p0(:,j,2) = j*y_dim - y_dim/2 + sum(y_div(1:j-1));
    p1(:,j,2) = j*y_dim - y_dim + sum(y_div(1:j-1));
    p2(:,j,2) = j*y_dim + sum(y_div(1:j-1));
    p3(:,j,2) = j*y_dim + sum(y_div(1:j-1));
    p4(:,j,2) = j*y_dim - y_dim + sum(y_div(1:j-1));
end

P0 = [reshape(p0(:,:,1),rows*cols,1), reshape(p0(:,:,2),rows*cols,1)];
P1 = [reshape(p1(:,:,1),rows*cols,1), reshape(p1(:,:,2),rows*cols,1)];
P2 = [reshape(p2(:,:,1),rows*cols,1), reshape(p2(:,:,2),rows*cols,1)];
P3 = [reshape(p3(:,:,1),rows*cols,1), reshape(p3(:,:,2),rows*cols,1)];
P4 = [reshape(p4(:,:,1),rows*cols,1), reshape(p4(:,:,2),rows*cols,1)];

A(:,:,1) = P1;
A(:,:,2) = P2;
A(:,:,3) = P3;
A(:,:,4) = P4;
A(:,:,5) = P0;

end