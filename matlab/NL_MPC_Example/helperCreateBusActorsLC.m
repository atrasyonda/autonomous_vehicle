function cellInfo = helperCreateBusActorsLC(varargin) 
% Part of set up script for the Lane Change Example
%
% It returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, SampleTime, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 
%
%   This is a helper for example purposes and may be removed or
%   modified in the future.

%   Copyright 2019 The MathWorks, Inc.

suppressObject = false; 
if nargin == 1 && islogical(varargin{1}) && varargin{1} == false 
    suppressObject = true; 
elseif nargin > 1 
    error('Invalid input argument(s) encountered'); 
end 

cellInfo = { ... 
  { ... 
    'BusActors1', ... 
    '', ... 
    '', ... 
    'Auto', ... 
    '-1', {... 
{'NumActors', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Time', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Actors', [10 1], 'Bus: BusActors1Actors', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'BusActors1Actors', ... 
    '', ... 
    '', ... 
    'Auto', ... 
    '-1', {... 
{'ActorID', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Position', [1 3], 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Velocity', [1 3], 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Roll', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Pitch', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Yaw', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'AngularVelocity', [1 3], 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo) 
end 