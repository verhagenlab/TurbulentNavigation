function [c] = odorFun_plume_center(x,y,t)
% Execute in cmdline before running:
% global plume
% plume = h5read('11282017_10cms_bounded.h5','/dataset7')

global plume % Import global var plume

frame = roundn(t*15,0);

t_idx = mod(frame,3599)+1;


y_vals = linspace(-20,20,216);
x_vals = linspace(10,85.2,406);

[~,x_idx] = min(abs(x_vals - x));
[~,y_idx] = min(abs(y_vals - y));

if abs(y) > 20 | x<10
    c =0;
else
    
    c = plume(y_idx,x_idx,t_idx);
end
end

