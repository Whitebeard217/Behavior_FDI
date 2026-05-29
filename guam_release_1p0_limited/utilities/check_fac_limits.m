function [Factor_output]=check_fac_limits(Factor,Lower_Limit,Upper_Limit,Hold_Percent)
% check_fac_limits - check if the aerodynamic database has been violated.
%
% DESCRIPTION: 
%   This function checks variable limits to determine if the aerodynamic
%   database has been violated. If the factor limits have been violated,
%   the boundary value for the factor will be held. If the factor exceeds a
%   certain percentage of the modeling range an error message will be
%   returned.
%
% INPUTS: 
%   Factor - Current factor value
%   Lower_Limit - Lower factor limit in the aerodynamic database
%   Upper_Limit - Upper factor limit in the aerodynamic database
%   Hold_percent - percentage of the factor range to allow violation before
%                  displaying an error message
%
% OUTPUTS:
%   Factor_output - Returned factor value (either the original value or 
%                   boundary value)
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   NASA Langley Research Center
%   Email: benjamin.m.simmons@nasa.gov
%
% HISTORY:
%   September 14, 2020 - Created and debugged, BMS
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
% NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
% DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
% OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
% USE OR OTHER DEALINGS IN THE SOFTWARE.
%
% ***FOR INTERNAL NASA USE ONLY***
%

coder.inline('never');

% range of model validity for the current factor
Range = Upper_Limit - Lower_Limit;

limit_tolerance = Range * Hold_Percent / 100;

% Check factor limits
%  - if model limits are exceeded hold the boundary value.
%  - return an error message if the limits are violated by more than a
%    certain percentage.

if Factor < Lower_Limit
  Factor_output = Lower_Limit;
elseif Factor > Upper_Limit
  Factor_output = Upper_Limit;
else
  Factor_output = Factor;
end

if ((Factor < (Lower_Limit - limit_tolerance)) || ...
    (Factor > (Upper_Limit + limit_tolerance)))
  error(['The aerodynamic database has been exceeded by more than ' num2str(Hold_Percent) '%'])
end

return