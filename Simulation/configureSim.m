function cfg = configureSim()
s=dbstatus('-completenames');
save bkpt s
%% clear
clear; clear global; clear mex; close all hidden;% clc;
%% restore breakpoitns
load bkpt 
dbstop(s);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% BUILD CONFIGURATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% cfg.srcs    = ; % Adds sources for all satellites
cfg.srcs = {};
cfg.srcs_sph{1} = {'../../../../SpheresCore/spheres_physical_ISS.c'}; % Adds sources to sphere 1 (same goes for sphere 2 or sphere 3)
cfg.srcs_sph{2} = {'../../../../SpheresCore/spheres_physical_ISS_VERTIGO.c'};
% Use this cell to add folders that should be on the compiler include path. Use the
% sph_inc cells to add folders to the include paths for specified spheres only.
cfg.incs = {};
cfg.incs_sph{1} = {}; % Adds include paths to sphere 1 (same goes for sphere 2 or sphere 3)

% Use this cell to add global preprocessor definitions
cfg.defines = {};

% Number of satellites
cfg.nSph = 2;

% IF using Windows and you do not have Visual Studio 10 installed
%  - OR -
% You wish to change from the default CMakefile generator, define your generator string
%   See: http://cmake.org/cmake/help/v2.8.8/cmake.html#section_Generators
% cfg.CMakeGen = ['Visual Studio 10 Win64'];  % Example

% Custom user defined CMake init string (appended to CMake initialization string)
% For example, overriding the default compiler selected
% cfg.CMakeInitCustom = ['-DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++']

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% RUN CONFIGURATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load generic global variables
globalvarSPHERES;

%% Configuration Parameters
% Use this section to configure parameters related to the simulation
% environment and execution options

cfg.test_number = 1;
cfg.nSph = 2;
cfg.simMode = 'mex'; % Simulation can run in 'mex' or 'simulink' modes

% Name of Simulink Model to run (if cfg.simMode = 'simulink'), if not set 'SpheresSimulation.slx' will be used
% cfg.simName = 'SpheresSimulation.slx'; 
cfg.simTimeout = 150000; % Simulation timeout in ms
cfg.testLocation = c.LOCATION_ISS;	%Set location to ISS
cfg.animateOn = 1; %Turns the animation on or off
cfg.stepSize = 1000; %How large each major simulation step should be. Affects resolution of simulation "Truth" state

% massCfg: if not defined, defaults to "1" which is standard spheres mass configuration
% cfg.massCfg = [1,1]; 

%% Satellite configuration
% Use this section to initialize the satellites in the simulation

%SPHERES states
initStates = zeros(13, cfg.nSph);
initStates(2,1) = -0.3;
initStates(2,2) =  0.3;
initStates(7,1:cfg.nSph) = 1; %Tank Down for both SPHERES
cfg.initStates = initStates;

%Runtime command
cfg.runtimeCmd = uint8(hex2dec('31'));



%
%
% Copyright (c) 2014, Massachusetts Institute of Technology Space Systems Laboratory
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without modification, are permitted
% provided that the following conditions are met:
% Redistributions of source code must retain the above copyright notice, this list of conditions and
% the following disclaimer.
% Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
% the following disclaimer in the documentation and/or other materials provided with the distribution.
% Neither the name of the Massachusetts Institute of Technology, the MIT Space Systems Laboratory, nor
% the names of its contributors may be used to endorse or promote products derived from this software
% without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
% IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
% FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
% CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
% DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
% WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% 
% ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
