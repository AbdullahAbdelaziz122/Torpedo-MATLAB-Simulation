function guidance_los_sfcn(block)
%% guidance_los_sfcn.m  —  Level-2 S-Function: Guidance Module (M2) — LOS
%
% BLOCK PORTS:
%   Inputs:
%     u1 : nu_hat    [6×1]  — body velocities from Navigation
%     u2 : eta_hat   [6×1]  — NED position/attitude from Navigation
%     u3 : pt        [3×1]  — virtual path point position
%     u4 : pt_vel    [3×1]  — virtual path point velocity
%     u5 : desire_spd[3×1]  — nominal desired speed [u;v;w]
%
%   Outputs:
%     y1 : chi_d      [1×1] — desired course angle (rad) → Control
%     y2 : upsilon_d  [1×1] — desired flight-path angle (rad) → Control
%     y3 : ud         [1×1] — desired surge speed (m/s) → Control
%     y4 : chi_v      [1×1] — vehicle course angle (rad) — logging
%     y5 : upsilon_v  [1×1] — vehicle flight-path angle (rad) — logging
%     y6 : los_errors [3×1] — [x_e; y_e; z_e] cross-track errors — logging
%
% DIRECT FEEDTHROUGH: true — outputs depend on current inputs.
%
% SWAPPABILITY:
%   To replace LOS with another guidance law: create a new S-Function
%   with the same output port layout (y1-y3 must match GuidanceBus).
%
% AUTHOR: AUV Simulation Project — Phase 6

setup(block);

% =========================================================================
function setup(block)

block.NumInputPorts  = 5;
block.NumOutputPorts = 6;

% u1: nu_hat [6×1]
block.InputPort(1).Dimensions  = 6;
block.InputPort(1).DatatypeID  = 0;
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% u2: eta_hat [6×1]
block.InputPort(2).Dimensions  = 6;
block.InputPort(2).DatatypeID  = 0;
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;

% u3: pt [3×1]
block.InputPort(3).Dimensions  = 3;
block.InputPort(3).DatatypeID  = 0;
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = true;

% u4: pt_vel [3×1]
block.InputPort(4).Dimensions  = 3;
block.InputPort(4).DatatypeID  = 0;
block.InputPort(4).Complexity  = 'Real';
block.InputPort(4).DirectFeedthrough = true;

% u5: desire_spd [3×1]
block.InputPort(5).Dimensions  = 3;
block.InputPort(5).DatatypeID  = 0;
block.InputPort(5).Complexity  = 'Real';
block.InputPort(5).DirectFeedthrough = true;

% y1: chi_d
block.OutputPort(1).Dimensions = 1;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';

% y2: upsilon_d
block.OutputPort(2).Dimensions = 1;
block.OutputPort(2).DatatypeID = 0;
block.OutputPort(2).Complexity = 'Real';

% y3: ud
block.OutputPort(3).Dimensions = 1;
block.OutputPort(3).DatatypeID = 0;
block.OutputPort(3).Complexity = 'Real';

% y4: chi_v
block.OutputPort(4).Dimensions = 1;
block.OutputPort(4).DatatypeID = 0;
block.OutputPort(4).Complexity = 'Real';

% y5: upsilon_v
block.OutputPort(5).Dimensions = 1;
block.OutputPort(5).DatatypeID = 0;
block.OutputPort(5).Complexity = 'Real';

% y6: los_errors [3×1]
block.OutputPort(6).Dimensions = 3;
block.OutputPort(6).DatatypeID = 0;
block.OutputPort(6).Complexity = 'Real';

block.NumContStates      = 0;
block.SampleTimes        = [-1 0];   % inherited
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('InitializeConditions', @InitConditions);
block.RegBlockMethod('Outputs',              @Outputs);
block.RegBlockMethod('Terminate',            @Terminate);

% =========================================================================
function InitConditions(block) %#ok<INUSD>

% =========================================================================
function Outputs(block)

nu_hat    = block.InputPort(1).Data;
eta_hat   = block.InputPort(2).Data;
pt        = block.InputPort(3).Data;
pt_vel    = block.InputPort(4).Data;
desire_spd= block.InputPort(5).Data;

try
    auv = evalin('base','auv');
catch
    error('guidance_los_sfcn: auv struct not found. Run auv_params.m.');
end

los = los_init(auv);

[chi_d, upsilon_d, ud, chi_v, upsilon_v, errs] = ...
    los_step(nu_hat, eta_hat, pt, pt_vel, desire_spd, los);

block.OutputPort(1).Data = chi_d;
block.OutputPort(2).Data = upsilon_d;
block.OutputPort(3).Data = ud;
block.OutputPort(4).Data = chi_v;
block.OutputPort(5).Data = upsilon_v;
block.OutputPort(6).Data = [errs.x_e; errs.y_e; errs.z_e];

% =========================================================================
function Terminate(block) %#ok<INUSD>

% =========================================================================
% Embedded copies — canonical source is guidance_los_lib.m
% =========================================================================

function los = los_init(auv)
los.ky=auv.guid.k_y; los.kz=auv.guid.k_z;
los.delta_y=auv.guid.delta_y; los.delta_z=auv.guid.delta_z;
los.U_max=auv.guid.U_max; los.U_min=auv.guid.U_min; los.U_nom=1.5;

function [chi_d,upsilon_d,ud,chi_v,upsilon_v,errs] = los_step(nu,eta,pt,pv,ds,los)
psi_v=eta(6);
Ry=[cos(psi_v) -sin(psi_v) 0; sin(psi_v) cos(psi_v) 0; 0 0 1];
vel_ned=Ry*nu(1:3);
[chi_v,upsilon_v]=vehicle_angles(vel_ned);
[psi_p,theta_p]=path_angles(pv);
[x_e,y_e,z_e]=los_errors(eta(1:3),pt,psi_p,theta_p);
[psi_r,theta_r]=los_corrections(x_e,y_e,z_e,los);
[chi_d,upsilon_d]=los_compose(psi_p,theta_p,psi_r,theta_r);
ud=los_speed(pv,x_e,psi_r,theta_r,ds,los);
errs.x_e=x_e; errs.y_e=y_e; errs.z_e=z_e;

function [cv,uv]=vehicle_angles(v)
if norm(v(1:2))<1e-4, cv=0; else, cv=atan2(v(2),v(1)); end
h=sqrt(v(1)^2+v(2)^2);
if h<1e-4, uv=0; else, uv=atan(-v(3)/h); end

function [pp,tp]=path_angles(pv)
if norm(pv(1:2))<1e-6, pp=0; else, pp=atan2(pv(2),pv(1)); end
h=sqrt(pv(1)^2+pv(2)^2);
if h<1e-6, tp=0; else, tp=atan(-pv(3)/h); end

function [xe,ye,ze]=los_errors(pos,pt,pp,tp)
dx=pos(1)-pt(1); dy=pos(2)-pt(2); dz=pos(3)-pt(3);
cp=cos(pp); sp=sin(pp); ct=cos(tp); st=sin(tp);
xe= cp*ct*dx+sp*ct*dy-st*dz;
ye=-sp*dx+cp*dy;
ze= cp*st*dx+sp*st*dy+ct*dz;

function [pr,tr]=los_corrections(xe,ye,ze,los) %#ok<INUSL>
pr=tanh(-los.ky*ye/los.delta_y);
tr=tanh( los.kz*ze/los.delta_z);

function [cd,ud_ang]=los_compose(pp,tp,pr,tr)
arg=sin(tp)*cos(tr)*cos(pr)+cos(tp)*sin(tr);
arg=max(-1+1e-12,min(1-1e-12,arg));
ud_ang=asin(arg);
cy=cos(pp)*sin(pr)*cos(tr)-sin(tp)*sin(tr)*sin(pp)+sin(pp)*cos(pr)*cos(tp)*cos(tr);
cx=-sin(pp)*sin(pr)*cos(tr)-sin(tp)*sin(tr)*cos(pp)+cos(pp)*cos(pr)*cos(tp)*cos(tr);
cd=atan2(cy,cx);

function ud=los_speed(pv,xe,pr,tr,ds,los)
spd=norm(pv);
denom=max(cos(pr)*cos(tr),0.1);
raw=(spd-0.001*xe)/denom;
u=ds(1); v=ds(2); w=ds(3); mag=sqrt(u^2+v^2+w^2);
if mag<1e-6, ud=los.U_nom; else, ud=raw*u/mag; end
ud=max(los.U_min,min(los.U_max,ud));
