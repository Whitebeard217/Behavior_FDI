function [C]=LpC_CCW_RotorModel_J040914(Jx,Jz,Mtip)
% LpC_CCW_RotorModel_J14 - Propeller aerodynamics model for the L+C aircraft
%
% DESCRIPTION: 
%   This script contains the propeller aerodynamics model for L+C. The script was
%   automatically generated using "run_LpC_rotor_modeling_2.m" on 19-Feb-2021 21:50:01
%
%   This function has been manually modified to combine results from three
%   different modeling approaches.
%   - Tx was found to need an extra spline term
%     variable in Jz to adequately characterize low Jz behavior. 
%   - All non-axial responses were modeled using reflected data. 
%   - Tx and Qx were too nonlinear to be able to develop an adequate model 
%     using reflected data.
%
%   Programs from the software toolbox called System IDentification
%   Programs for AirCraft, or SIDPAC, were used to determine the model 
%   structure and parameter estimates shown below.
%
%      Model code: 9 & 14
%      Modeling Algorithm: mof with up to 4th order regressors
%      Explanatory Variables: [Jx,Jz,Mtip_0p3,Jz_0p2] for code 9
%      Explanatory Variables: [Jx,Jz,Mtip_0p3] for code 4 and 14
% 
% INPUTS:
%   List of column vectors containing the model explanatory variables 
%       The variables are in the following order:
%       [Jx,Jz,Mtip]
%
% OUTPUTS:
%   C - Matrix containing the model response variables in each column 
%       The variables are in the following order:
%       [C_Tx,C_Ty,C_Tz,C_Qx,C_Qy,C_Qz]
%
% CALLS:
%   None
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   NASA Langley Research Center
%   Email: benjamin.m.simmons@nasa.gov
%
% HISTORY:
%   February 20, 2021 - Created and debugged, BMS
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
% ***FOR INTERNAL NASA USE ONLY***
%

% spline variable for tip Mach number
Mtip_0p3 = Mtip-0.3;
Mtip_0p3(Mtip_0p3<0) = 0;

% spline variable for Jz
Jz_0p2 = Jz-0.2;
Jz_0p2(Jz_0p2<0) = 0;

% initialize the output
C = zeros(1,6);

%Force and moment polynomials

% FROM: LpC_CCW_RotorModel_J9.m
% C_Tx, R^2=99.88, PSE=3.679E-06, 17 terms
C_Tx = ...
   +1.468025331590495E-01  .*                     1  + ... % SE: 6.885e-05, PE:   0.05%, 95% CI: [  +0.1467,  +0.1469]
   -1.047428782702512E-01  .*                    Jx  + ... % SE: 0.0004671, PE:   0.45%, 95% CI: [  -0.1057,  -0.1038]
   +7.965812396761629E-02  .*                    Jz  + ... % SE: 0.0003478, PE:   0.44%, 95% CI: [ +0.07896, +0.08035]
   -1.488834785361099E-01  .*                 Jz.^2  + ... % SE: 0.0003780, PE:   0.25%, 95% CI: [  -0.1496,  -0.1481]
   -1.526258474381517E-01  .*     Jz .* Mtip_0p3.^2  + ... % SE: 0.0004192, PE:   0.27%, 95% CI: [  -0.1535,  -0.1518]
   -2.144142218139758E-01  .*              Jx .* Jz  + ... % SE:  0.002963, PE:   1.38%, 95% CI: [  -0.2203,  -0.2085]
   +3.330327072970288E-01  .*           Jx .* Jz.^2  + ... % SE:  0.003332, PE:   1.00%, 95% CI: [  +0.3264,  +0.3397]
   +1.669368561728097E-01  .*                Jz_0p2  + ... % SE: 0.0005575, PE:   0.33%, 95% CI: [  +0.1658,  +0.1681]
   +2.164828542840951E-02  .*                 Jz.^3  + ... % SE: 0.0001212, PE:   0.56%, 95% CI: [ +0.02141, +0.02189]
   -1.330817360826850E-01  .*  Jx .* Jz .* Mtip_0p3.^2  + ... % SE:  0.001485, PE:   1.12%, 95% CI: [  -0.1361,  -0.1301]
   -6.120919446078687E-03  .*  Mtip_0p3 .* Jz_0p2.^3  + ... % SE: 8.718e-05, PE:   1.42%, 95% CI: [-0.006295,-0.005947]
   -3.570576428055945E-01  .*          Jx .* Jz_0p2  + ... % SE:  0.004935, PE:   1.38%, 95% CI: [  -0.3669,  -0.3472]
   -5.374865969197672E-02  .*           Jx .* Jz.^3  + ... % SE:  0.001091, PE:   2.03%, 95% CI: [ -0.05593, -0.05157]
   +6.854348590608577E-03  .*              Mtip_0p3  + ... % SE: 0.0001336, PE:   1.95%, 95% CI: [+0.006587,+0.007121]
   -1.162224572882008E-01  .*        Jx.^2 .* Jz.^2  + ... % SE:  0.003642, PE:   3.13%, 95% CI: [  -0.1235,  -0.1089]
   -2.118093652146521E-01  .*                 Jx.^2  + ... % SE:  0.002735, PE:   1.29%, 95% CI: [  -0.2173,  -0.2063]
   +3.632010735473220E-01  .*           Jx.^2 .* Jz ;      % SE:  0.006883, PE:   1.90%, 95% CI: [  +0.3494,  +0.3770]

% FROM: LpC_CCW_RotorModel_J14.m
% C_Ty, R^2=95.58, PSE=2.475E-06, 12 terms
C_Ty = zeros(size(Jx)); %...
%    -1.798864645379544E-02  .*        Jz .* Mtip_0p3  + ... % SE: 0.0004380, PE:   2.44%, 95% CI: [ -0.01886, -0.01711]
%    -1.012847718304478E-02  .*           Jx .* Jz.^3  + ... % SE: 7.858e-05, PE:   0.78%, 95% CI: [ -0.01029,-0.009971]
%    -4.218154164987606E-03  .*                 Jz.^3  + ... % SE: 1.344e-05, PE:   0.32%, 95% CI: [-0.004245,-0.004191]
%    +1.742624372980549E-02  .*     Jz.^3 .* Mtip_0p3  + ... % SE: 5.960e-05, PE:   0.34%, 95% CI: [ +0.01731, +0.01755]
%    +6.761250971164910E-03  .*                    Jz  + ... % SE: 3.648e-05, PE:   0.54%, 95% CI: [+0.006688,+0.006834]
%    -6.366755802698862E-02  .*  Jx .* Jz .* Mtip_0p3  + ... % SE:  0.001633, PE:   2.57%, 95% CI: [ -0.06693, -0.06040]
%    +4.880976868493601E-02  .*     Jz .* Mtip_0p3.^2  + ... % SE:  0.002402, PE:   4.92%, 95% CI: [ +0.04401, +0.05361]
%    +9.505073715739394E-02  .*  Jx .* Jz .* Mtip_0p3.^2  + ... % SE:  0.003828, PE:   4.03%, 95% CI: [ +0.08739,  +0.1027]
%    +4.738715418200865E-03  .*              Jx .* Jz  + ... % SE: 0.0002157, PE:   4.55%, 95% CI: [+0.004307,+0.005170]
%    +1.748378028297267E-02  .*           Jx.^2 .* Jz  + ... % SE:  0.001105, PE:   6.32%, 95% CI: [ +0.01527, +0.01969]
%    -5.544556105657573E-02  .*     Jz .* Mtip_0p3.^3  + ... % SE:  0.003651, PE:   6.59%, 95% CI: [ -0.06275, -0.04814]
%    -3.479867022094399E-02  .*  Jx.^2 .* Jz .* Mtip_0p3 ;      % SE:  0.004670, PE:  13.42%, 95% CI: [ -0.04414, -0.02546]

% FROM: LpC_CCW_RotorModel_J14.m
% C_Tz, R^2=99.59, PSE=4.013E-06, 8 terms
C_Tz = ...
   -2.298965856263280E-02  .*                    Jz  + ... % SE: 4.043e-05, PE:   0.18%, 95% CI: [ -0.02307, -0.02291]
   -3.894258631246416E-01  .*     Jz .* Mtip_0p3.^3  + ... % SE: 0.0008121, PE:   0.21%, 95% CI: [  -0.3911,  -0.3878]
   +2.503974238547664E-01  .*  Jx .* Jz .* Mtip_0p3.^2  + ... % SE:  0.003946, PE:   1.58%, 95% CI: [  +0.2425,  +0.2583]
   +2.841497096854941E-03  .*                 Jz.^3  + ... % SE: 1.672e-05, PE:   0.59%, 95% CI: [+0.002808,+0.002875]
   -7.181938737861267E-03  .*     Jz.^3 .* Mtip_0p3  + ... % SE: 7.335e-05, PE:   1.02%, 95% CI: [-0.007329,-0.007035]
   +1.398390357128658E-02  .*        Jz .* Mtip_0p3  + ... % SE: 0.0002257, PE:   1.61%, 95% CI: [ +0.01353, +0.01444]
   -1.410046524018437E-01  .*  Jx.^2 .* Jz .* Mtip_0p3  + ... % SE:  0.003618, PE:   2.57%, 95% CI: [  -0.1482,  -0.1338]
   +3.839212119242863E-02  .*  Jx .* Jz .* Mtip_0p3 ;      % SE:  0.001405, PE:   3.66%, 95% CI: [ +0.03558, +0.04120]

% FROM: LpC_CCW_RotorModel_J4.m
% C_Qx, R^2=99.98, PSE=1.209E-06, 15 terms
C_Qx = ...
   +1.045430918802704E-02  .*                     1  + ... % SE: 1.120e-05, PE:   0.11%, 95% CI: [ +0.01043, +0.01048]
   +3.592429540771470E-01  .*  Jz.^2 .* Mtip_0p3.^2  + ... % SE: 0.0007116, PE:   0.20%, 95% CI: [  +0.3578,  +0.3607]
   -4.329368054834152E-01  .*           Mtip_0p3.^4  + ... % SE:   0.01231, PE:   2.84%, 95% CI: [  -0.4576,  -0.4083]
   -1.117662832625882E-01  .*     Jz.^2 .* Mtip_0p3  + ... % SE: 0.0004701, PE:   0.42%, 95% CI: [  -0.1127,  -0.1108]
   +6.678338552648375E-01  .*           Mtip_0p3.^3  + ... % SE:  0.009339, PE:   1.40%, 95% CI: [  +0.6492,  +0.6865]
   +1.306573176821386E-01  .*     Jx .* Mtip_0p3.^3  + ... % SE:  0.001872, PE:   1.43%, 95% CI: [  +0.1269,  +0.1344]
   +3.481050880406903E-02  .*     Jz.^3 .* Mtip_0p3  + ... % SE: 0.0001643, PE:   0.47%, 95% CI: [ +0.03448, +0.03514]
   +6.032089437241136E-01  .*     Jz .* Mtip_0p3.^3  + ... % SE:  0.002687, PE:   0.45%, 95% CI: [  +0.5978,  +0.6086]
   -3.575432656880349E-01  .*     Jz .* Mtip_0p3.^2  + ... % SE:  0.001942, PE:   0.54%, 95% CI: [  -0.3614,  -0.3537]
   +8.699544286162707E-02  .*        Jz .* Mtip_0p3  + ... % SE: 0.0005292, PE:   0.61%, 95% CI: [ +0.08594, +0.08805]
   -1.467390110044703E-01  .*           Mtip_0p3.^2  + ... % SE:  0.001835, PE:   1.25%, 95% CI: [  -0.1504,  -0.1431]
   -5.280476267748680E-03  .*                    Jx  + ... % SE: 4.387e-05, PE:   0.83%, 95% CI: [-0.005368,-0.005193]
   -2.241617297575048E-03  .*                 Jz.^4  + ... % SE: 1.563e-05, PE:   0.70%, 95% CI: [-0.002273,-0.002210]
   +3.673721680985544E-03  .*                 Jz.^3  + ... % SE: 2.793e-05, PE:   0.76%, 95% CI: [+0.003618,+0.003730]
   +6.037683379233425E-02  .*  Jx .* Jz .* Mtip_0p3.^2 ;      % SE: 0.0007613, PE:   1.26%, 95% CI: [ +0.05885, +0.06190]

% FROM: LpC_CCW_RotorModel_J14.m
% C_Qy, R^2=99.85, PSE=1.764E-06, 6 terms
C_Qy = ...
   +3.222852942764558E-02  .*                    Jz  + ... % SE: 1.719e-05, PE:   0.05%, 95% CI: [ +0.03219, +0.03226]
   -6.826676220368251E-02  .*              Jx .* Jz  + ... % SE: 0.0001903, PE:   0.28%, 95% CI: [ -0.06865, -0.06789]
   -1.397843934137373E-03  .*                 Jz.^3  + ... % SE: 6.827e-06, PE:   0.49%, 95% CI: [-0.001411,-0.001384]
   +6.507928217060512E-03  .*           Jx .* Jz.^3  + ... % SE: 6.146e-05, PE:   0.94%, 95% CI: [+0.006385,+0.006631]
   +3.428994495606562E-01  .*           Jx.^3 .* Jz  + ... % SE:  0.005593, PE:   1.63%, 95% CI: [  +0.3317,  +0.3541]
   +1.962805046642213E-02  .*           Jx.^2 .* Jz ;      % SE: 0.0005575, PE:   2.84%, 95% CI: [ +0.01851, +0.02074]

% FROM: LpC_CCW_RotorModel_J14.m
% C_Qz, R^2=99.81, PSE=2.686E-06, 6 terms
C_Qz = ...
   +2.827444735196227E-02  .*                    Jz  + ... % SE: 2.144e-05, PE:   0.08%, 95% CI: [ +0.02823, +0.02832]
   +2.395276353744833E-03  .*                 Jz.^3  + ... % SE: 8.514e-06, PE:   0.36%, 95% CI: [+0.002378,+0.002412]
   -1.737254102033171E-02  .*              Jx .* Jz  + ... % SE: 0.0002373, PE:   1.37%, 95% CI: [ -0.01785, -0.01690]
   +2.598992488506573E-03  .*           Jx .* Jz.^3  + ... % SE: 7.665e-05, PE:   2.95%, 95% CI: [+0.002446,+0.002752]
   -1.773660455383638E-01  .*           Jx.^3 .* Jz  + ... % SE:  0.006975, PE:   3.93%, 95% CI: [  -0.1913,  -0.1634]
   +1.647300208738315E-02  .*           Jx.^2 .* Jz ;      % SE: 0.0006952, PE:   4.22%, 95% CI: [ +0.01508, +0.01786]

% C_Qx and C_Qz are both negative for the CCW rotating rotor, as viewed
% from the top
C_Qx=-1*C_Qx;
C_Qz=-1*C_Qz;

% output the propeller force and moment coefficients
C = [C_Tx,C_Ty,C_Tz,C_Qx,C_Qy,C_Qz];

return