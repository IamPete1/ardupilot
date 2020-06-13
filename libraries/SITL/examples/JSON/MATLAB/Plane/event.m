    function [value,isterminal,direction] = event(t,x)
%   event.m
%   Definition of Event in FLIGHT.m
%   Time when (-height) passes through zero in an increasing direction
%   i.e., Transition from positive to negative altitude

%   ==================================================================
%   November 11, 2018

%   Called by:
%       odeset.m in FLIGHT.m

    value       =   x(6); % detect positive x(6) = 0
    isterminal  =   1; % stop the integration
    direction   =   1; % positive direction