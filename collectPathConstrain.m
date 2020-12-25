function [c,ce]=collectPathConstrain(phaseT,SP,x,u)
% collect terrain and kinematic constrain as path constrain
% INPUT
%   phaseT=[1,nPhase]=phase duration vector
%   x=[nStates,nGridAll]= states of all collocation points
%   u=[nInputs,nGridAll]= inputs of all collocation points
[cT,ceT]=TerrainConstrain(phaseT,SP,x,u);
[cK,ceK]=KinematicConstrain(phaseT,SP,x,u);
[cP,ceP]=multiPhaseConstrain(phaseT,SP,x,u);
% cK=[];
% ceK=[];
c=[cT;cK;cP];
ce=[ceT;ceK;ceP];