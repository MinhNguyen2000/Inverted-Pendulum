% Author: Steve Brunton (adapted by Minh Nguyen)
% Date: May 23, 2024
% Description:

clear all
close all
clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; % For pendulum up position, s = 1

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s/(M*L)];

% Stability check
lambda = eig(A)

% Controllability
rank(ctrb(A,B))
