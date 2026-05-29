% plot_LpC_rotor_models - plot Lift+Cruise rotor model surface plots
%
% DESCRIPTION:
%   This script creates surface plots for the Lift+Cruise vehicle
%   rotor/propeller models. This is used to validated that the model
%   functions  are working properly. This program is not used for the
%   simulation.
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
%   February 26, 2020 - Created and debugged, BMS
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
fac2=-10:0.25:10;
fac2_var='w_kts';
fac2_name=[strrep(fac2_var,'_',' ['),']'];

% specify the second variable to loop through
% fac2=-5:0.25:5;
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
d2r=c.deg;
fps2kts=1/c.knot;
kts2fps=c.knot;
radps2rpm=60/(2*pi);

% density input
rho=0.00198698;% density at 6000 ft [slug/ft^3]
a=1093.18; %speed of sound at 6000 ft [ft/s]

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
        
        u_fps=u_kts*kts2fps;
        v_fps=v_kts*kts2fps;
        w_fps=w_kts*kts2fps;
        
        p_rps=p_dps*d2r;
        q_rps=q_dps*d2r;
        r_rps=r_dps*d2r;
        
        n_revps=[n1_rpm,n2_rpm,n3_rpm,n4_rpm,n5_rpm,n6_rpm,n7_rpm,n8_rpm,n9_rpm]/60;
        
        [X(ii,jj),Y(ii,jj),Z(ii,jj),L(ii,jj),M(ii,jj),N(ii,jj)]=LpC_Total_PropFM(u_fps,v_fps,w_fps,p_rps,q_rps,r_rps,n_revps,rho,a,SimIn.Model);
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

