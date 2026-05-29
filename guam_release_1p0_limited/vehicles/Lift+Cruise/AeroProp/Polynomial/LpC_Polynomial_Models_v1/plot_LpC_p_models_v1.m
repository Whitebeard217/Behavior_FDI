% plot_LpC_models - plot Lift+Cruise surface plots
%
% DESCRIPTION: 
%   This script creates surface plots for the Lift+Cruise vehicle. This is
%   used to validated that the polynomial functions and blending methods
%   are working properly. This program is not used for the simulation.
%
% INPUT: 
%   range of flight conditions specified below
%
% OUTPUT:
%   plots
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   NASA Langley Research Center
%   Email: benjamin.m.simmons@nasa.gov
%
% HISTORY:
%   May 6, 2020 - Created and debugged, BMS
%

% extract constants
c=SimIn.Units;

% blending method
BlendM=3;


% propeller and rotor settings
Rotor_RPM=1050;
Prop_RPM=1150;


% specify the first variable to loop through
fac1=-5:95;
fac1_var='u_kts';
fac1_name=[strrep(fac1_var,'_',' ['),']'];

% specify the second variable to loop through
% fac1=-2:0.25:2;
% fac1_var='v_kts';
% fac1_name=[strrep(fac1_var,'_',' ['),']'];


% specify the second variable to loop through
fac2=-2:0.25:2;
fac2_var='v_kts';
fac2_name=[strrep(fac2_var,'_',' ['),']'];

% specify the second variable to loop through
% fac2=0:0.25:5;
% fac2_var='w_kts';
% fac2_name=[strrep(fac2_var,'_',' ['),']'];

% specify nominal state and control variables
u_kts=50;
v_kts=0;
w_kts=0;

p_dps=0;
q_dps=0;
r_dps=0;

daL_deg=0;
daR_deg=0;
deL_deg=0;
deR_deg=0;
dr_deg=0;



n1_rpm=Rotor_RPM;
n2_rpm=Rotor_RPM;
n3_rpm=Rotor_RPM;
n4_rpm=Rotor_RPM;
n5_rpm=Rotor_RPM;
n6_rpm=Rotor_RPM;
n7_rpm=Rotor_RPM;
n8_rpm=Rotor_RPM;
n9_rpm=Prop_RPM;




% conversion constants
r2d=1/c.deg;
fps2kts=1/c.knot;
radps2rpm=60/(2*pi);

% initialize force and moment vectors
X=zeros(length(fac1),length(fac2));
Y=zeros(length(fac1),length(fac2));
Z=zeros(length(fac1),length(fac2));
L=zeros(length(fac1),length(fac2));
M=zeros(length(fac1),length(fac2));
N=zeros(length(fac1),length(fac2));

% loop through each flight condition
for ii=1:length(fac1)
    eval([fac1_var,'=fac1(ii);']);
    for jj=1:length(fac2)
        eval([fac2_var,'=fac2(jj);']);
        x=[[u_kts,v_kts,w_kts]/fps2kts,[p_dps,q_dps,r_dps]/r2d];
        n=[n1_rpm,n2_rpm,n3_rpm,n4_rpm,n5_rpm,n6_rpm,n7_rpm,n8_rpm,n9_rpm]*2*pi/60;
        d=[daL_deg,daR_deg,deL_deg,deR_deg,dr_deg]/r2d;
        [X(ii,jj),Y(ii,jj),Z(ii,jj),L(ii,jj),M(ii,jj),N(ii,jj)] = LpC_aero_p_v1(x,n,d,BlendM,c);
    end
end

% create surface plots
C_label={'X','Y','Z','L','M','N'};
C_name={'X [lbf]','Y [lbf]','Z [lbf]','L [ft-lbf]','M [ft-lbf]','N [ft-lbf]'};
for ii=1:length(C_label)
    figure
    surf(fac1,fac2,transpose(eval(C_label{ii})));hold on
    xlabel(fac1_name)
    ylabel(fac2_name)
    zlabel(C_name{ii})
    set(gca,'fontsize',12)
end

