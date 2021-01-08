%---------------------------------------------
%-------------- Deliverable 6.1 --------------
%---------------------------------------------

% Project : Quadcopter control
% Authors : Balestrini, Durand, Hoggett
% 31 dec. 2020

clc
clear
close

quad = Quad();
CTRL= ctrl_NMPC(quad);

sim = quad.sim(CTRL);
quad.plot(sim);
