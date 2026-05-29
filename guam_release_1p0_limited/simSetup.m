% close any previously loaded vehicle-specific model
close_system('GUAM',0);

% setup default base paths
setupPath;

% setup the structure containing enumeration values used to set types in
% SimIn to determine variant selections (used by setupTypes.m)
setupVariantStruct;

% assign type choice selections into SimIn structure
SimIn = setupTypes(SimIn, userStruct.variants);

% set switch settings in SimIn structure
setupSwitches;

% set default units
SimIn.Units=setUnits('ft','slug');
  
% Set reference inputs
if SimIn.Switches.RefTrajOn
    if SimIn.refInputType == RefInputEnum.TIMESERIES  
        % Timeseries trajectory option
        if ~(exist('target','var') && isfield(target,'RefInput'))
            % Setup default trajectory if one is not provided
            time = 0:40;
            target.RefInput.Vel_bIc_des    = timeseries(zeros(length(time),3),time);
            target.RefInput.vel_des        = timeseries(zeros(length(time),3),time);
            target.RefInput.pos_des        = timeseries(zeros(length(time),3),time);
            target.RefInput.chi_des        = timeseries(zeros(length(time),1),time);
            target.RefInput.chi_dot_des    = timeseries(zeros(length(time),1),time);
        end
        Qfields = fieldnames(target.RefInput);
        for Qn = 1:length(Qfields)
            if ~ismatrix(target.RefInput.(Qfields{Qn}).Data)
                fprintf('Reference Input Dimension > 2. Check variable (%s) and time orientation.\n',Qfields{Qn});
            end
        end
    end
else
    if exist('target','var') % target variable specified
        % Check user at least supplied: tas, gndtrack and stopTime
        if ~(isfield(target,'tas') && isfield(target,'gndtrack') && isfield(target,'stopTime'))
            error('For user specified target structure: tas, gndtrack and stopTime must be provided for non-TIMESERIES refInput Type\n');
        end
    else
        % Specify default conditions if user didn't specify neither
        % Timeseries nor minimal target structure
        target = struct('tas', 90, 'gndtrack', 0, 'stopTime', 10);
    end

end

% once all choices are selected, run setup
SimIn = setup(SimIn, target);  % standard simulation mode
%SimIn = setup(SimIn, target, true);  % release simulation mode

% Setup Parameters
setupParameters(SimIn);

% create variant control objects
setupVariants;

% create buses
setupBuses;

% Clear target after setup (so it won't be accidentally used next run)
clear target