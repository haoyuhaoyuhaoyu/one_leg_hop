% derive dynamics through lagrange
clear variables;
close all;
syms pcx pcy sita real;
syms dpcx dpcy dsita real;
syms ddpcx ddpcy ddsita real;
syms fx fy pex pey real;
syms g m I positive;

% q = [pcx, pcy, sita]
% tau = cross([pex-pcx;pey-pcy;0], [fx;fy;0])
