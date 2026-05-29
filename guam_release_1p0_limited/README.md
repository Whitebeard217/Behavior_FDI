% *************************************************************************
% NASA TTT-Autonomous Systems(AS): Intelligent Contingency Management (ICM)
%                       Generic UAM Simulation 
%                       Version 1.0
% *************************************************************************
%
% *************************************************************************
% Point of Contact:
% Michael J. Acheson
% NASA Langley Research Center (LaRC)
% Dynamics Systems and Control Branch (D-316)
% email: michael.j.acheson@nasa.gov, (757) 864-9457
% *************************************************************************
%
% *************************************************************************
% To run a simulation example case, just execute the RUNME.m script at the top level!
% **************************************************************************

% This simulation is a beta version generic UAM simulation.  It includes
% a generic Lift+Cruise aircraft model representative of a NASA Lift+Cruise 
% vehicle configuration.  
% 
% Some of the key simulation components include:
% 1) A simulation architecture (e.g., signal buses)  that supports the most 
% common rigid body 6-DOF frames of reference (e.g., Earth Centered Inertial, 
% Earth Centered Earth Fixed (ECEF), North-East-Down (NED), Navigation, 
% Velocity, Wind, Stability, and Body)
% 2) A simulation architecture that contains most aerospace signals/quantities 
% of typical interest
% 3) A generic architecture that readily supports swapping in and out aircraft 
% models, sensors, actuator models, control algorithms etc..
% 
% Demonstration trajectory flights are found in the Exec_Scripts folder. Demo
% case include: 
%   1) a basic lifting hover and transition to forward flight (./Exec_Scripts/exam_TS_Hover2Cruise_traj.m)
%   2) a cruise climbing right hand turn (./Exec_Scripts/exam_TS_Cruise_Climb_Turn_traj.m)
%   3) a simple sinusoidal input case  (./Exec_Scripts/exam_TS_Sinusoidal_traj.m)
% To execute the simulation, open the main folder 
% in the matlab command line, add the Exec_Scripts folder to the path and 
% execute either the "exam_TS_Cruise_Climb_Turn_traj.m" or "exam_TS_Hover2Cruise_traj.m"
% m-file.  Once the "GUAM" simulink model opens, run the model.  Output data is
% stored in a structure "SimOut" (whereas simulation input data is provided in 
% "SimIn" and "SimPar".  Plotting of the results can be performed by running the m-file
% "./vehicles/Lift+Cruise/Utils/simPlots_GUAM.m".
%
% While the demonstration scripts are available, a user can in general select
% among variants subsystem variants through an input structure "userStruct" 
% then call the simSetup.m script to prepare the simulation for execution.
% The simulation makes use of an input structure: "userStruct" to change the variants 
% of various subsystems (see ./setup/setupVariantStruct.m) that are used, 
% and to specify various "switches" (see ./setup/setupSwitches.m).  If a user
% doesn't provide userStruct.variant selections, then the default choices 
% (vehicle specific) as set in ./vehicles/Lift+Cruise/setup/setupDefaultChoices.m. 
% An example of the available userStruct options are:
%   userStruct.variants:
%       refInputType: Timeseries
%       vehicleType: LiftPlusCruise
%       expType: DEFAULT
%       atmosType: US_STD_ATMOS_76
%       turbType: None
%       ctrlType: BASELINE
%       actType: FirstOrder
%       propType: None
%       fmType: SFunction
%       eomType: Simple
%       sensorType: None
%
% The available options for the UserStruct.variants subfields are enumerations
% specified in the "ClassDef" folder. For example the actuator type variants
% available are found in the ActuatorEnum.m file:
%
% classdef ActuatorEnum < Simulink.IntEnumType
%  enumeration
%    None(1)
%    FirstOrder(2)
%    SecondOrder(3) 
%    FirstOrderFailSurf(4)
%  end
% end
% A user can either write an m-file script or simply call (in the matlab command 
% line):
% >>userStruct.variants.actType = 3; simSetup
% Now the GUAM sim is ready to execute using the SecondOrder actuator model.
% 
% Users have a few options to provide desired trajectories and basic flight 
% manuevers to execute.  The userStruct.variant.refInputType option selects
% (look in the Lift+Cruise Reference Inputs variant subsytem block at the simulation
% top level) between: FOUR_RAMP, ONE_RAMP, TIMESERIES, and DEFAULT options.  
% The timeseries is demonstrated in the Exec_Scripts folder for the m-files 
% that contain "TS" in the name and they end with the suffix "_traj.m". 
% 
% A sample use case of the FOUR_RAMP is also contained in the Exec_Scripts, where 
% Simulink ramp blocks are used to build a simple trajectory.  In this option, 
% the user makes use of an input structure "target" to provide some basic trim
% configuration information and then provides timing and magnitude for the 
% ramps blocks using the SimPar structure. Target field available for user 
% specification include: 'alt', 'tas', 'gndtrack', 'RefInput', 'Rate', and
% 'stopTime'. A sample script "exam_RAMP.m" file demonstrates the use of both
% the user provided target structure and the RAMP refInput variant subsytem. 
%
% Two Aero-propulsive model options are available.  A medium fidelity, first 
% principles strip-thoery model is available with the fmType variant (SFunction 
% and Polynomial options).  The SFunction variant contains a matlab 
% class/object based model that allows a user to 
% build an aircraft configuration (airfoil, number and location of rotors,
% aerodynamic surfaces etc..).  The current aircraft configuration is a generic
% Lift+Cruise configuration (for more details see the matlab
%  /vehicles/Lift+Cruise/AeroPro/SFuntion folder).  The other aeropropulsive model
% option available is a blended polynomial fit database that was constructed
% from higher fidelity data (many CFD) for the generic Lift+Cruise configurtion.
% The variant "SFunction" is valid throughout the flight envelope, whereas the 
% "Polynomial" variant may occasionally fail in some corners of the envelope when 
% users ask for performance data in areas which are not covered by the polynomial
% data.  
%
% Basic simulation execution performance can be viewed via Simulink scopes 
% in the VehicleSimulation/Vehicle Generalized Control/ Lift+Cruise Control/Baseline
% block.

