clear all
close all
clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

u = 0; % No control input (free response)

tspan = 0:0.1:10;
y0 = [0; 0; pi; .5];
[t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,u), tspan, y0);

for k = 1:length(t)
    drawcartpend_bw(y(k,:),m,M,L);
end

title_list = ["$x$", "$\dot x$", "$\theta$", "$\dot \theta$"];
figure()
for i = 1:4
    subplot(4,1,i)
    plot(t,y(:,i))
    title(title_list(i),'interpreter','latex')
    grid minor
end
xlabel('Time (s)')

% figure(title = 'Control input')