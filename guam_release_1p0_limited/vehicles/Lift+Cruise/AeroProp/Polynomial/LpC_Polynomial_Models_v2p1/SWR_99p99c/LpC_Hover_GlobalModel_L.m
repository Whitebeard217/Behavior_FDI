function [C,Metrics]=LpC_Hover_GlobalModel_L(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8)
% LpC_Hover_GlobalModel_L - Blends partitioned aerodynamics model for the LpC Hover
%
% DESCRIPTION: 
%   This function blends local aerodynamic models for LpC Hover to create 
%   a global model. The script was automatically generated using
%   "GenGlobalModel_fixed0.m" on 14-Jan-2021 08:00:56
%
% INPUTS:
%   List of column vectors containing the model explanatory variables 
%       The variables are in the following order:
%       [u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8]
%       Units: deg  kts  rpm 
%
% OUTPUTS:
%   C - Matrix containing the model response variables in each column 
%       The variables are in the following order:
%       [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag]
%       Units: ft-lbf  lbf 
%
% CALLS:
%   LpC_Hover_R7t10_PCp2FI_SWR_cv  LpC_Hover_R7t15_PCp2FI_SWR_cv  LpC_Hover_R12t15_PCp2FI_SWR_cv  
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   NASA Langley Research Center
%   Email: benjamin.m.simmons@nasa.gov
%
% HISTORY:
%   July 15, 2020 - created and debugged, BMS
%
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
% IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
% CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
% TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
% SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
%
% ***FOR INTERNAL NASA LARC USE ONLY***
%
%
%   Rapid Aero Modeling (RAM) References:
%   [1] Murphy, P. C., Simmons, B. M., Hatke, D. B., Busan, R. C., "Rapid
%       Aero Modeling for Urban Air Mobility Aircraft in Wind-Tunnel
%       Tests," AIAA SciTech 2021 Forum, AIAA Paper 2021-1644.
%
%   [2] Murphy, P. C., Buning, P. G., and Simmons, B. M., "Rapid Aero
%       Modeling for Urban Air Mobility Aircraft in Computational
%       Experiments," AIAA SciTech 2021 Forum, AIAA Paper 2021-1002.
%
% Warning: this script is automatically generated. Do not manually modify.
%
%
%   Factor ranges used to develop model:
%             u: [      -5,     +45] kts
%             v: [     -10,     +10] kts
%             w: [     -10,     +10] kts
%            LA: [     -30,     +30] deg
%            RA: [     -30,     +30] deg
%            LE: [     -30,     +30] deg
%            RE: [     -30,     +30] deg
%           RUD: [     -30,     +30] deg
%            N1: [    +550,   +1550] rpm
%            N2: [    +550,   +1550] rpm
%            N3: [    +550,   +1550] rpm
%            N4: [    +550,   +1550] rpm
%            N5: [    +550,   +1550] rpm
%            N6: [    +550,   +1550] rpm
%            N7: [    +550,   +1550] rpm
%            N8: [    +550,   +1550] rpm
%

% initialize variables for Simulink implementation
UpperBound=0;
LowerBound=0;
C=zeros(1,8);
C1=zeros(1,8);
C2=zeros(1,8);
Blend=0;

switch true
   case u<(8 + 1/(-2))
      [C]=LpC_Hover_R7t10_PCp2FI_SWR_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      Blend=0; % no model blending is required
   case u>=(8 + 1/(-2)) && u<(20)
      [C1]=LpC_Hover_R7t10_PCp2FI_SWR_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      [C2]=LpC_Hover_R7t15_PCp2FI_SWR_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      Blend = 1; % model blending is required
      UpperBound = 20; % upper limit of modeling interval
      LowerBound = 8 + 1/(-2); % lower limit of modeling interval
   case u>=(20) && u<(33 + 1/(-2))
      [C1]=LpC_Hover_R7t15_PCp2FI_SWR_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      [C2]=LpC_Hover_R12t15_PCp2FI_SWR_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      Blend = 1; % model blending is required
      UpperBound = 33 + 1/(-2); % upper limit of modeling interval
      LowerBound = 20; % lower limit of modeling interval
   case u>=(33 + 1/(-2))
      [C]=LpC_Hover_R12t15_PCp2FI_SWR_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8);
      Blend=0; % no model blending is required
end

if Blend==1
   % smooth blending between two regions using a quintic polynomial
   % weighting function, which is continuous and twice differentiable at
   % boundaries (Credit: Dan Moerder, NASA)
   wx= (UpperBound-u)/(UpperBound-LowerBound);
   w1=6*wx.^5-15*wx.^4+10*wx.^3;
   w2=1-w1;

   % combine weighted results
   C = w1*C1 + w2*C2;
end
Metrics=[]; % Metrics is not used for the global model

return