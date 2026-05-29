function [x0,u0,y0,dx0,ix,iu,iy,idx,xulb,xuub] = trimStruct2Array(Trim)
%% Map Trim structure to trimsim inputs
    % Set target/guess for states/psuedo states
    fn  = fieldnames(Trim.States);
    ix  = logical([]);
    x0  = [];
    xlb = [];
    xub = [];
    for i = 1:numel(fn)
        ix  = [ ix;Trim.States.(fn{i}).perturb(:)];
        x0  = [ x0;Trim.States.(fn{i}).x0(:)];
        xlb = [xlb;Trim.States.(fn{i}).min(:)];
        xub = [xub;Trim.States.(fn{i}).max(:)];
    end
    ix  = find(ix);

    % Set state/psuedo state derivatives to constrain
    fn  = fieldnames(Trim.Derivatives);
    idx  = logical([]);
    dx0  = [];
    for i = 1:numel(fn)
        idx  = [ idx;Trim.Derivatives.(fn{i}).constrain(:)];
        dx0  = [ dx0;Trim.Derivatives.(fn{i}).dx0(:)];
    end
    idx = find(idx);

    
    % Target/Guess for inputs from top level input port.  These typicaly
    % require plumbing down into the diagram followed by a switch controlled by
    % the trim flag.
    fn = fieldnames(Trim.Inputs);
    iu  = logical([]);
    u0 = [];
    ulb = [];
    uub = [];
    for i = 1:numel(fn)
        iu  = [ iu;Trim.Inputs.(fn{i}).perturb(:)];
        u0  = [ u0;Trim.Inputs.(fn{i}).u0(:)];
        ulb = [ulb;Trim.Inputs.(fn{i}).min(:)];
        uub = [uub;Trim.Inputs.(fn{i}).max(:)];
    end
    iu = find(iu);
    
    % Target for outputs
    fn = fieldnames(Trim.Outputs);
    iy  = logical([]);
    y0 = [];
    for i = 1:numel(fn)
        iy = [ iy;Trim.Outputs.(fn{i}).target(:)];
        y0 = [ y0;Trim.Outputs.(fn{i}).y0(:)];
    end
    iy = find(iy);
    
    % Combine bounds of psuedo states and inputs
    xulb = [xlb(ix);ulb(iu)];
    xuub = [xub(ix);uub(iu)];