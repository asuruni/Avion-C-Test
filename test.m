% a.s.d = 123;
% a.s.f = [1 2;3 4];
% a.g.h = 'sd';
% ite = [1 2 3];
% c = {a, ite; 0,7};
% for i = 2:4
%     c(i,:) = {ite, 9+2*i};
% end

%res = quoi(a);

a={3 1;5 0; 1 0; 1 1; 4 1};

[b d] = max([a{:,1}].*[a{:,2}]);
disp(b)
disp(d)
