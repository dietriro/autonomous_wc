
x_ = size(map, 1);
y_ = size(map, 2);

[X,Y] = meshgrid(1:y_,1:x_);

figure
surf(X, Y, map(1:x_, 1:y_, 1))
% figure
% surf(X, Y, map(1:x_, 1:y_, 2))
% figure
% surf(X, Y, map(1:x_, 1:y_, 3))
% figure
% surf(X, Y, map(1:x_, 1:y_, 4))