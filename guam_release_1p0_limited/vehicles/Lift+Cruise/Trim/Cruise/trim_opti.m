% using optimization to solve for trim conditions with constraints
global TRIM_POINT
global LPC
global GRAV
global RHO

build_Lift_plus_Cruise
tiltwing.Tp = [0 0 0 0 0 0 0 0 0];

%trim point
V = 1:1:200; 
gamma = 0*pi/180;
LPC = tiltwing;
GRAV = 32.17405;
RHO = 0.0023769;


% initial guess
Tp0    =  0;
dele0  =  0*pi/180; 
alpha0 =  -1*pi/180;
Tl0    =  800.0;
Tt0    =  700.0;

% set the initial guess 
x0 = [Tp0; dele0; alpha0; Tl0; Tt0];

% weighted quadratic cost on the thrust
fcost = @(x) x'*diag([1.1 0 0 1 1])*x;

XEQ = nan(length(V), 5);
ft2kts = 0.592484;
exitcount = 0;
for ii = 1:length(V)

  TRIM_POINT = [V(ii) gamma];

  sig = 0.5*(tanh((V(ii)-70)/20)+1);
  %sig = 1;

  % set actuator bounds
  lb = [   0 -35*sig*pi/180 -10*pi/180    0    0];
  ub = [1500  35*sig*pi/180  10*pi/180 1500 1500];

  % use fmincon to solve a constrained optimization propblem
  opts = optimoptions(@fmincon,'Algorithm','interior-point','Display','off');
  [ x, fval, exitflag ] = fmincon(fcost,x0,[],[],[],[],lb,ub,@nlinCon,opts);

  fprintf('----------------------\n');
  fprintf('trim point: V = %0.2f kts, gam = %0.2f deg\n\n',...
          TRIM_POINT.*[ft2kts 180/pi]);
  fprintf('  Tp    = % 0.4f lbf\n', x(1));
  fprintf('  dele  = % 0.4f deg\n', x(2)*180/pi);
  fprintf('  alpha = % 0.4f deg\n', x(3)*180/pi);
  fprintf('  Tl    = % 0.4f lbf\n', x(4));
  fprintf('  Tt    = % 0.4f lbf\n', x(5));
  fprintf('\n')

  if exitflag > 0
    fprintf('  exitflag = %u\n',exitflag);
    fprintf('-- trim point saved --\n');
    fprintf('----------------------\n');
    XEQ(ii,:) = x;
    x0 = x;
    exitcount = 0;
  else 
    exitcount = exitcount+1;
  end

  if exitcount >10 
    break
  end

end

XEQ = [ V' repmat(gamma,size(V')) XEQ];

%% Make some nice plots

figure
plot(V*ft2kts, [ XEQ(:,3) XEQ(:,6:7)],'o-')
grid on 
zoom on
xlabel('Velocity [kts]', 'fontsize',20)
ylabel('Thrust [lbf]', 'fontsize',20)
legend({'$T_{pusher}$','$T_{lead}$','$T_{trail}$'},'fontsize',15)

figure
plot(V*ft2kts, [XEQ(:,5) XEQ(:,4)]*180/pi,'o-')
grid on 
zoom on
xlabel('Velocity [kts]', 'fontsize',20)
ylabel('Angle [deg]', 'fontsize',20)
legend({'$\alpha$','$\delta_e$'},'fontsize',15)

% figure
% plot(V*ft2kts, XEQ(:,2)*180/pi,'o-')
% grid on 
% zoom on
% xlabel('Velocity [kts]', 'fontsize',20)
% ylabel('Elevator Deflection [deg]', 'fontsize',20)


