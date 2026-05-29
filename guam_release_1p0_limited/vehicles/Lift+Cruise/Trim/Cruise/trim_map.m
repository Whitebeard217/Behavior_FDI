% Trimming out the Lift+Cruise 

% build the CL-84
build_Lift_plus_Cruise
tiltwing.Tp = [0 0 0 0 0 0 0 0 0];

% environmental conditions
grav = 32.17405;
rho = 0.0023769;

% stepping vairables
step = 5;
dir = -1;
range = [100 300];

TRIM_GAM = [0];

for ii = 1:length(TRIM_GAM)

  %trim point
  V = range(2); 

  % range for fixed wing configuration
  gamma = TRIM_GAM(ii)*pi/180;
  trim_point = [V gamma];

  % initial guess
  T0 = 500;
  dele0 = -1.5*pi/180; 
  alpha0 = 1.981*pi/180;
  U0 = [T0 dele0 alpha0];

  % setup some storage
  VEL    = []; 
  GAM    = [];
  TP     = [];
  DELE   = [];
  ALF    = [];
  SVD    = [];

  while  V >= range(1) & V <= range(2)

    % Elvevator
    

    % Use fsolve to find the equilibrium point
    opt = optimset('Display','on','MaxFunEvals',50,...
                   'TolX',1e-6,'TolFun',1e-6);
    [Ueq Yeq ExitFlag Output Jacobian] = fsolve(@trim_lon, U0, opt,...
      tiltwing, trim_point, grav, rho);

    sing_vals = svd(Jacobian);

    keyboard

    % print the equilibrium conditions
    Tp   = Ueq(1);
    dele = Ueq(2); 
    alf  = Ueq(3); 

    fprintf('\nsvd = [%f %f %f]\n',sing_vals);
    fprintf('Trim Point = [ %f %f ]\n', trim_point.*[1 180/pi]);
    if ExitFlag < 1
      fprintf('Could not find a trim point\n\n')
      break;
    end

    % print the trim point
    fprintf('Tp    = %f\n',Tp);
    fprintf('dele  = %f\n',dele*180/pi);
    fprintf('alpha = %f\n',alf*180/pi);

    if abs(Tp) > 1250 | abs(dele) > 60*pi/180
      fprintf('Trim point beyond actuator limits\n\n')
      %break   
    end

    %% save the trim point
    VEL = [VEL; V];
    GAM = [GAM; trim_point(2)];
    TP  = [TP; Tp];
    DELE = [DELE; dele];
    ALF = [ALF; alf];
    SVD = [SVD; sing_vals'];

    U0 = Ueq;
    V = trim_point(1)+dir*step;
    trim_point(1) = V;

  end

  %%


  trim = [VEL GAM TP DELE ALF];

  filename = sprintf('trim_gam_%0.1f',trim_point(2)*180/pi);
  filename = strrep(filename,'.','p');
  save(sprintf('%s.mat',filename), 'trim');

end


return

%% evaluate the aerodynamics at the equilibrium point
alpha = Ueq(3);
om = [0; 0; 0];
vI = [V*cos(gamma); 0; -V*sin(gamma)];
R = [cos(alpha) 0 -sin(alpha); 0 1 0; sin(alpha) 0 cos(alpha)];
vB = R*vI;
om = [0; 0; 0];
rho = 1.225;

tiltwing.aero(rho, vB, om);


u = V*cos(alpha);
w = V*sin(alpha);
theta = gamma + U0(3);
q = 0;

m = tiltwing.mass;
Jy = tiltwing.I(2,2);

Fx_u = tiltwing.Fx_x(1);
Fx_w = tiltwing.Fx_x(3);
Fx_q = tiltwing.Fx_x(5);

Fz_u = tiltwing.Fz_x(1);
Fz_w = tiltwing.Fz_x(3);
Fz_q = tiltwing.Fz_x(5);

My_u = tiltwing.My_x(1);
My_w = tiltwing.My_x(3);
My_q = tiltwing.My_x(5);

A = [   1/m*Fx_u  -q+1/m*Fx_w  -g*cos(theta)  -w+1/m*Fx_q;
      q+1/m*Fz_u     1/m*Fz_w  -g*sin(theta)   u+1/m*Fz_q;
          0            0           0               1     ;
       1/Jy*My_u    1/Jy*My_w      0            1/Jy*My_q];

Fx_Tw = 2*tiltwing.Fx_u(1);
Fx_dele = tiltwing.Fx_u(6);
 
Fz_Tw = 2*tiltwing.Fz_u(1);
Fz_dele = tiltwing.Fz_u(6);

My_Tw = 2*tiltwing.My_u(1);
My_dele = tiltwing.My_u(6);

B = [  1/m*Fx_Tw   1/m*Fx_dele;
       1/m*Fz_Tw   1/m*Fz_dele;
         0           0        ;
      1/Jy*My_Tw  1/Jy*My_dele];

