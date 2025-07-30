clc, clearvars

% Define rotor geometry
    % arguments: 
    % - Rhub:
    % - Rtip (for F, need to find physical description)
    % - B: Number of blades
    % - tip_pitch: Pitch angle of blade tip

function rotor = initRotor(Rhub, Rtip, B, tipPitch)
    rotor.Rhub = Rhub;
    rotor.Rtip = Rtip;
    rotor.B = B;
    rotor.tipPitch = tipPitch;
end

% Define rotor section geometry
    % Arguments:
    % - r: Radius at each station
    % - c: Chord " " "
    % - twist: Twist " " "
    % - af: Airfoil " " "
function section = initSection(r, c, twist, af)
    section.r = r;
    section.c = c;
    section.twist = twist;
    section.af = af;
end

% Initialize a station based on location on blade section
    % Arguments:
    % - section: Blade section to use
    % - n: Discrete location of station on blade section

function station = initStation(section, n)
    station.r = section.r(n);
    station.c = section.c(n);
    station.twist = section.twist(n);
    station.af = section.af(n);
end

% Define the rotor operational point: 
    % Arguments:
    % - Uinf: Incoming wind velocity
    % - rho: Density of fluid
    % - mu: Dynamic viscosity of fluid
    % - TSR: Tip speed ratio (to calculate omega)

function op = initOp(Uinf, rho, mu, tsr)
    op.Uinf = Uinf;
    op.rho = rho;
    op.mu = mu;
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
function F = findF(B, Rhub, Rtip, r, phi)
    ftip = (B/2)*(Rtip - r)/(r*abs(sin(phi)));
        Ftip = (2/pi)*acos(exp(-ftip));
        fhub = (B/2)*(r-Rhub)/(Rhub*abs(sin(phi)));
        Fhub = (2/pi)*acos(exp(-fhub));
        F = Ftip*Fhub;
end

% BEM core algorithm

function [R, outputs] = residualAndOutputs(phi, rotor, station, op)

    % Compute constants: sigma_p (local solidity), F (hub/tip loss factor)
    sigma_p = (rotor.B*rotor.c)/(2*pi*station.r);
    F = findF(rotor.B, rotor.Rhub, rotor.Rtip, station.r, phi);

    % Determine alpha
    alpha = phi - (station.twist + rotor.tipPitch);

    % Compute Re(neglecting induction) = (Urel*Rho*c_mean)/mu. (W = sqrt(Ux^2
    % + U_y^2)

    % Look up Cl and Cd (f(alpha, Re)

    % Apply rotational and Re correction

    % Compute Cn and Cax (Coefficient of Axial force)

    % Compute k and kp (Arbitrary constants to help with calculating a and
    % ap)

    % Compute a and ap (a is computed for Algo 1)

    % Obtain a residual function R(phi)

    % Compute Urel

    % Compute T' (Thrust per unit length)

    % Compute Q' (Torque per  unit length)
end

 % function firstBracket: finds a bracket where the residual changes sign

 % function solveStation: solves BEM for a station

 % function solveBlade: runs solveStation for the entire blade

