

GAM = [0 2.5 5 7.5 10];

TRIM_DATA = nan(200,length(GAM));
V         = nan(200,length(GAM));
gam       = nan(200,length(GAM));
Tp        = nan(200,length(GAM));
dele      = nan(200,length(GAM));
alfb      = nan(200,length(GAM));

kk = 1;
for ii = 1:length(GAM);

 filename = sprintf('trim_gam_%0.1f',GAM(ii));
 filename = strrep(filename,'.','p');
 load(filename);

 [rows cols] = size(trim);

 V(1:rows,ii)    = trim(:,1);
 gam(1:rows,ii)  = trim(:,2);
 Tp(1:rows,ii)   = trim(:,3);
 dele(1:rows,ii) = trim(:,4);
 alfb(1:rows,ii) = trim(:,5);

 legendtext{ii} = sprintf('\\gamma = %0.2f',GAM(ii));

end

cmap = num2cell(parula(length(GAM)),2);

figure
h = plot(V, Tp);
set(h, {'color'}, cmap);
grid on
zoom on
xlabel('Velocity [m/s]','fontsize',20)
ylabel('Pusher Propeller Thrust [lbf]','fontsize',20)
legend(legendtext);

figure
h = plot(V, dele*180/pi);
set(h, {'color'}, cmap);
grid on
zoom on
xlabel('Velocity [m/s]','fontsize',20)
ylabel('Elevator Deflection Angle [deg]','fontsize',20)
legend(legendtext);

figure
h = plot(V, alfb*180/pi);
set(h, {'color'}, cmap);
grid on
zoom on
xlabel('Velocity [m/s]','fontsize',20)
ylabel('Angle of Attack [deg]','fontsize',20)
legend(legendtext);
