%{
    Author: Divine Nduka
    
    CCBlade by Andrew Ning (Brigham young University) was used as reference
    for this code. Theoretical backing was taken from Ning, Computational
    Aerodynamics, and Manwell, Wind Energy Explained
%}

clc, clearvars

% Constants
rhoAir = 1.184; % At 25C
muAir = 0.00001837; % At 25C

% Define rotor parameters
% arguments:
% - Rhub:
% - Rtip (for F, need to find physical description)
% - B: Number of blades
% - reCorr: Whether to apply Re correction
% - rotCorr: Whether to apply rotational correction

function rotor = initRotor(Rhub, Rtip, B, tipPitch, reCorr, rotCorr)
rotor.Rhub = Rhub;
rotor.Rtip = Rtip;
rotor.B = B;
rotor.reCorr = reCorr;
rotor.rotCorr = rotCorr;
end

% Define rotor section geometry
% Arguments:
% - r: Radius at each station
% - c: Chord " " "
% - theta: Pitch angle " " "
% - af: File name of txt file containing airfoil data
% - Re: Reynolds number for provided airfoil data (for Re corrections)
function section = initSection(r, c, theta, af, Re)
section.r = r;
section.c = c;
section.theta = theta;
section.af = af;
section.Re = Re;
end

% Initialize a station based on location on blade section
% Arguments:
% - section: Blade section to use
% - n: Discrete location of station on blade section

function station = initStation(section, n)
station.r = section.r(n);
station.c = section.c(n);
station.theta = section.theta(n);
station.af = section.af(n);
end

% Define the rotor operational point:
% Arguments:
% - Uinf: Incoming wind velocity
% - rho: Density of fluid
% - mu: Dynamic viscosity of fluid
% - tsrL: Tip speed ratio lower bound
% - tsrU: tsr upper bound
% - tsr: Nominal tsr

function op = initOp(Uinf, rho, mu, tsrL, tsrU, tsr)
op.Uinf = Uinf;
op.rho = rho;
op.mu = mu;
op.tsrL = tsrL;
op.tsrU = tsrU;
op.tsr = tsr;
end

%{
  Define the output from the residual function:
    Arguments:
      - Tp: Thrust per unit length 
      - Qp: Torque per unit length
      - a: Axial induction factor
      - ap: Angular induction factor
      - u: Axial induced velocity
      - v: Angular induced velocity
      - phi: Inflow angle
      - alpha: AoA
      - Urel: Inflow velocity
      - Cl: Lift coefficient
      - Cd: Drag coefficient
      - Cn: Normal force coefficient
      - Cax: Axial force coefficient
      - F: hub/tip loss factor
%}

function outputs = initOutput(Tp, Qp, a, ap, u, v, phi, alpha, Urel, Cl, Cd, Cn, Cax, F)
outputs.Tp = Tp;
outputs.Qp = Qp;
outputs.a = a;
outputs.ap = ap;
outputs.u = u;
outputs.v = v;
outputs.phi = phi;
outputs.alpha = alpha;
outputs.Urel = Urel;
outputs.Cl = Cl;
outputs.Cd = Cd;
outputs.Cn = Cn;
outputs.Cax = Cax;
outputs.F = F;
end

% Compute the tip/hub loss factor by Glauert's method
function F = prandtlLoss(B, Rhub, Rtip, r, phi)
ftip = (B/2)*(Rtip - r)/(r*abs(sin(phi)));
Ftip = (2/pi)*acos(exp(-ftip));
fhub = (B/2)*(r-Rhub)/(Rhub*abs(sin(phi)));
Fhub = (2/pi)*acos(exp(-fhub));
F = Ftip*Fhub;
end

% function reCorr

% function rotCorr

% function for airfoil polar extrapolation

% BEM core algorithm
% Arguments:
% - phi: initial phi
% - rotor: rotor properties
% - station: station properties
% - op: operating point
% Returns:
% - R: residual function
% - outputs: BEM output data

function [R, outputs] = residualAndOutputs(phi, rotor, station, op)

% Compute constants: sigma_p (local solidity), F (hub/tip loss factor)
sigma_p = (rotor.B*station.c)/(2*pi*station.r);
F = prandtlLoss(rotor.B, rotor.Rhub, rotor.Rtip, station.r, phi);
sphi = sin(phi);
cphi = cos(phi);
V = (op.tsr*op.Uinf);

% Determine alpha
alpha = phi - (station.theta);

% Compute Re(neglecting induction)
Urel = sqrt((op.Uinf)^2 + (V)^2);
Re_c = (op.rho*Urel*station.c)/op.mu; % To be used for Re corrections

% Look up Cl and Cd (f(alpha, Re)
Cl = 2*pi*alpha; % currently placeholders
Cd = 0.5;

% Apply rotational and Re correction

% Compute Cn and Cax (Coefficient of Axial force)
Cn = Cl*cphi + Cd*sphi;
Cax = Cl*sphi - Cd*cphi;

% Compute k and kp (Arbitrary constants to help with calculating a and
% ap)
k = (sigma_p*Cax)/(4*F*sphi^2);
kp = (sigma_p*Cax)/(4*F*sphi*cphi);

% Compute a and ap (neglecting phi<0 case)

% State corresopnds to Uinf=0 or omega*R = 0, return any nonzero
% residual
if isapprox(k, 1.0, "tight") || isapprox(kp, 1.0, "tight")
    R = 1.0; outputs = initOutput(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    return
end

if k <= 2.0/3  % momentum region
    a = k/(1 + k);

else
    g1 = 2*k - 1.0/9;
    g2 = 2*k - 1.0/3;
    g3 = 2*k - 7.0/9;
    a = (g1 - sqrt(g2)) / g3;
end

ap = kp/(1-kp);

u = op.Uinf*a; v = V*ap;

% Obtain a residual function R(phi)
R = sin(phi)/(1-a) - cos(phi)/(op.tsr*(1+ap));

% Compute Urel (considering induction)
Urel = sqrt((op.Uinf - u)^2 + (V + v)^2);

% Compute Tp (Thrust per unit length)
Tp = rotor.B*Cn*0.5*op.rho*Urel^2*station.c;
Qp = rotor.B*Cax*0.5*op.rho*Urel^2*station.c;

% Compute Qp (Torque per  unit length)
outputs = initOutput(Tp, Qp, a, ap, u, v, phi, alpha, Urel, Cl, Cd, Cn, Cax, F);

end

% function firstBracket: Find a bracket for the root closest to xmin by
% subdividing interval (xmin, xmax) into n intervals.

function [success, xu, xl] = firstBracket(f, xmin, xmax, n)

% Form n intervals between xmin and xmax, and assign initial parameter
% values
xvec = linspace(xmin, xmax, n);
fprev = f(xmin);
success = false;

for i = 2:n
    fcurr = f(xvec(i));

    if fcurr*fprev < 0 % Sign changed across interval
        xu = xvec(i); xl = xvec(i-1); success = true;
        return
    end

    fprev = fcurr;
end

xu = 0; xl = 0;

end

% function solveStation: solves BEM for a station
% Arguments:
% -   rotor: rotor properties
% -   station: station properties
% -   op: operating point
% Returns:
% - outputs: BEM output data
function outputs = solveStation(rotor, station, op, npts)

% Check if hub/tip is being evaluated
if isapprox(station.r, rotor.Rtip, "tight") || isapprox(station.r, rotor.Rhub, "tight")
    outputs = initOutput(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    return
end

% Initialize phimin and phimax, limits for finding bracket in residual
phimin = 0; phimax = pi/2;

% Pull out residual from residualAndOutputs
    function R = residual(phi)
        [R,~] = residualAndOutputs(phi, rotor, station, op);
    end

% Find bracket where R changes sign
[success, phiU, phiL] = firstBracket(@(phi)residual(phi), phimin, phimax, npts);

% Find phistar (phi that satisfies R = 0) with Brent's method (only if
% success)
if success
    phistar = fzero(@(phi)residual(phi), [phiL, phiU]);
else
    disp("First bracket failed, changing initial condition for brent's to pi/2");
    phistar = fzero(@(phi)residual(phi), pi/2);
end

residual(phistar) % test phistar by evaluating R at phistar

% Find correct outputs with phistar
[~, outputs] = residualAndOutputs(phistar, rotor, station, op);

end


% function nondim: Non-dminesionalise inputs and return coefficients
% Arguments:
% - T: Thrust
% - Q: Torque
% - op: Rotor operational point
% - rotor: rotor object
% Returns:
% - Ct: Thurst Coefficient
% - Cq: Torque Coefficient
% - Cp: Power Coefficient
function [Ct, Cq, Cp] = nondim(T,Q,op,rotor)
% Compute power and constants
omega = op.tsr*op.Uinf/rotor.Rtip;
P = Q*omega;
q = 0.5*op.rho*op.Uinf^2;
A = pi*rotor.Rtip^2;

% Non-dimensionalise
Ct = T/(q*A);
Cq = Q/(q*A*rotor.Rtip);
Cp = P/(q*op.Uinf*A);
end

% function solveRotor: runs solveStation for the entire blade
% Arguments:
% - rotor: rotor properties
% - section: section properties
% - op: operating point
% Returns:
% - Ct: Rotor thurst coefficient
% - Cq: Rotor torque coefficient
% - Cp: Rotor Power coefficient

function [Ct, Cq, Cp] = solveRotor(rotor, section, op)

npts = 10;
% Initialize arrays for Tp and Qp that are the length of section.r
Tp = zeros(1, length(section.r)); Qp = zeros(1, length(section.r));

% Compute Tp and Qp for all blade stations
for i = 1:length(section.r)
    msg = ['iter = ', num2str(i)];
    disp(msg);
    Tp(i) = solveStation(rotor, initStation(section, i), op, npts).Tp;
    Qp(i) = solveStation(rotor, initStation(section, i), op, npts).Qp;
end

% Initialize rfull, Tpfull and Qpfull
rfull = [rotor.Rhub section.r rotor.Rtip];
Tpfull = [0.0 Tp 0.0];
Qpfull = [0.0 Qp 0.0];

% Integrate Tpfull and Qpfull over rfull
T = trapz(rfull, Tpfull);
Q = trapz(rfull, Qpfull);

% non-dimensionalise the results
[Ct, Cq, Cp] = nondim(T, Q, op, rotor);
end

% function performancePlot: Generate performance plots and report maximum
% values and coefficients at those values
% Arguments:
% - tsrL: TSR lower bound
% - tsrU: TSR upper bound
% - rotor: Rotor object
% - section: Blade section
% - op: Operating condition
% Returns:
% - [Cpmax, tsr1, Ct1, Cq1]: Max power coefficient (other results reported
%                            at corresponding tsr)
% - [Ctmax, tsr2, Cp2, Cq2]: Max Ct '' '' ''
% - [Cqmax, tsr3, Cp3, Ct3]: Max Cq '' '' ''

function [Cpmax, tsr_p, Ct_p, Cq_p, Ctmax, tsr_t, Cp_t, Cq_t, Cqmax, tsr_q, Cp_q, Ct_q] = performancePlot(rotor, section, op)

% Generate tsrVec
tsrVec = linspace(op.tsrL, op.tsrU, 20);

% Initialize Cpvec, Ctvec, Cqvec
Cp = zeros(1, length(tsrVec));
Ct = zeros(1, length(tsrVec));
Cq = zeros(1, length(tsrVec));

% solveRotor for all tsr in tsrVec
for i = 1:length(tsrVec)
    msg = ['tsrVec = ', num2str(tsrVec(i))];
    disp(msg);
    opCurr = initOp(op.Uinf, op.rho, op.mu, op.tsrL, op.tsrU, tsrVec(i));
    [Ct(i), Cq(i), Cp(i)] = solveRotor(rotor, section, opCurr);
end

% Plot all coefficients vs tsrVec
figure
plot(tsrVec, Cp)
title('Rotor Performance Plot')
xlabel('Tip Speed Ratio (\lambda)')
ylabel('C_{p}, C_{t}, C_{q}')
hold on

plot (tsrVec, Ct)
hold on

plot(tsrVec, Cq)
hold on
legend('Cp', 'Ct', 'Cq')

% Find max for all coefficients and location in array
Cpmax = max(Cp); np = Cp == Cpmax;
tsr_p = tsrVec(np); Ct_p = Ct(np); Cq_p = Cq(np);

Ctmax = max(Ct); nt = Ct == Ctmax;
tsr_t = tsrVec(nt); Cp_t = Cp(nt); Cq_t = Cq(nt);

Cqmax = max(Cq); nq = Cq == Cqmax;
tsr_q = tsrVec(nq); Cp_q = Cq(nq); Ct_q = Ct(nq);

end

% [~,~] = textAf(-5.3, 'xf-naca4415-il-50000.txt');

% rotor params
Rhub = 1.5; Rtip = 63; B = 3;
testRotor = initRotor(Rhub, Rtip, B, deg2rad(0), 'none', 'none');

% section params
r = [2.8667, 5.6000, 8.3333, 11.7500, 15.8500, 19.9500, 24.0500, 28.1500, 32.2500, 36.3500, 40.4500, 44.5500, 48.6500, 52.7500, 56.1667, 58.9000, 61.6333];
chord = [3.542, 3.854, 4.167, 4.557, 4.652, 4.458, 4.249, 4.007, 3.748, 3.502, 3.256, 3.010, 2.764, 2.518, 2.313, 2.086, 1.419];
theta = deg2rad([13.308, 13.308, 13.308, 13.308, 11.480, 10.162, 9.011, 7.795, 6.544, 5.361, 4.188, 3.125, 2.319, 1.526, 0.863, 0.370, 0.106]);

afData = ["data/Cylinder1.dat", "data/Cylinder2.dat", "data/DU40_A17.dat", "data/DU35_A17.dat", "data/DU30_A17.dat", "data/DU25_A17.dat", "data/DU21_A17.dat", "data/NACA64_A17.dat"];
afIndex = [1, 1, 2, 3, 4, 4, 5, 6, 6, 7, 7, 8, 8, 8, 8, 8, 8];
airfoils = afData(afIndex);

testSection = initSection(r, chord, theta, airfoils, 50000);

% op params
Uinf = 10; tsrL = 2; tsrU = 15; tsr = [];
op = initOp(Uinf, rhoAir, muAir, tsrL, tsrU, tsr);

% [Ct, Cq, Cp] = solveRotor(testRotor, testSection, op)

% Max conditions for all coefficients
CpMaxCondition = zeros(1, 4); CtMaxCondition = zeros(1,4); CqMaxCondition = zeros(1,4);

[CpMaxCondition(1), CpMaxCondition(2), CpMaxCondition(3), CpMaxCondition(4), CtMaxCondition(1), CtMaxCondition(2), CtMaxCondition(3), CtMaxCondition(4), CqMaxCondition(1), CqMaxCondition(2), CqMaxCondition(3), CqMaxCondition(4)] = performancePlot(testRotor, testSection, op);
CpMaxCondition, CtMaxCondition, CqMaxCondition