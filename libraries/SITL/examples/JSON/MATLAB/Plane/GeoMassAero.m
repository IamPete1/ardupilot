%   GeoMassAero.m
%   Supporting Calculations for Geometric, Inertial, and Aerodynamic 
%   Properties of BizJet B for Use in AeroModelAlpha.m
%   ================================================================
%   November 11, 2018
%	Copyright 2006-2018 by ROBERT F. STENGEL.  All rights reserved.

    clear
    disp('==============================================')
    disp('Geometric and Inertial Properties for BizJet B')
    disp('==============================================')
    disp(' ')
    disp(['Date and Time are ', num2str(datestr(now))]);
    disp('===============================')
    
%   Data saved to:
%       InerGeo.mat
%       DataTable.mat
%       RotCont.mat

    D2R =   pi/180;
    R2D =   180/pi;
    
%   Comparable Bizjet Weight Distribution, lb, based on empty weight less engine weight. 
%   (from Stanford AA241 Notes, Ilan Kroo)   
    WingSys         =   1020;
    TailSys         =   288;
    BodySys         =   930;
    GearSys         =   425;
    NacelleSys      =   241;
    PropSys         =   340;
    ControlSys      =   196;
    InstrSys        =   76;
    HydrPneuSys     =   94;
    ElecSys         =   361;
    AvionSys        =   321;
    FurnEquipSys    =   794;
    ACSys           =   188;
    AntiIceSys      =   101;
    LoadHandSys     =   2;
    EmptyStruc      =   (WingSys+TailSys+BodySys+GearSys+PropSys ...
                        +ControlSys+InstrSys+HydrPneuSys+ElecSys+AvionSys ...
                        +FurnEquipSys+ACSys+AntiIceSys+LoadHandSys);   
                        % Less nacelles and engines
    EngWgt          =   1002;
    EmptyWgt        =   (EmptyStruc+EngWgt);    % Less nacelles
    EmptyStrucWgt   =   EmptyStruc/EmptyWgt;
    
    WingRatio       =   (WingSys+GearSys+HydrPneuSys+AntiIceSys)/EmptyStruc;
    HTRatio         =   0.75*(TailSys)/EmptyStruc;
    VTRatio         =   0.25*(TailSys)/EmptyStruc;
    FusRatio        =   (BodySys+PropSys+ControlSys+InstrSys+ElecSys ...
                        +AvionSys+FurnEquipSys+ACSys+LoadHandSys)/EmptyStruc;
    Total           =   (WingRatio+HTRatio+VTRatio+FusRatio);
    NacelleRatio    =   (NacelleSys)/EngWgt;    
                        % Related to engine weight rather than empty structure 
    
%	BizJet B Geometric Properties
%   x measurements from nose along centerline, negative aft 
%   y & z measurements from centerline, positive right and down

	S		=	19.51			% Reference Area, m^2
 	taperw	=	0.5333			% Wing Taper Ratio
	cBar	=	1.56			% Mean Aerodynamic Chord, m
    sweep   =   11*D2R    % Wing L.E. sweep angle, rad
    xcp     =   -5.7473         % Wing center of pressure, m
    GamWing =   3*D2R     % Dihedral angle of the wing, rad

%	BizJet B Mass and Inertial Properties
	m		=	3000                    % Total mass for simulation (USER-specified), kg
    mEmpty  =   2522                    % Gross empty mass, kg
    mEng    =   (1+NacelleRatio)*240    % Mass of engines + nacelles, kg
    mStruc  =   mEmpty - mEng           % Empty structural mass (less engines + nacelles), kg
    mWing   =   WingRatio*mStruc        % Wing mass, kg
    mHT     =   HTRatio*mStruc          % Horizontal tail mass, kg
    mVT     =   VTRatio*mStruc          % Vertical tail mass, kg
    mFus    =   FusRatio*mStruc         % Empty fuselage mass, kg
    mPay    =   0.5*(m - mEmpty)        % Payload mass, kg
    mFuel   =   0.5*(m - mEmpty)        % Fuel mass, kg

    xcm     =   xcp - 0.4*cBar % Center of mass from nose (USER-specified), m
    
    lWing   =   xcm - xcp   % Horizontal distance between c.m and wing c.p., m
    zWing   =   -0.557      % Vertical distance between c.m and wing c.p., m
    b		=	13.16       % Wing Span, m
    
    lenFus  =   10.72           % Fuselage length, m
    xcpFus  =   -0.25*lenFus    % Linear-regime fuselage center of pressure, m
    xcpFusN =   -0.5*lenFus     % Newtonian-regime fuselage center of pressure, m
    lFus    =   xcm - xcpFus;   % Linear fuselage lift cp offset,m
    lFusN   =   xcm - xcpFusN;  % Newtonian fuselage lift cp offset, m
    
    dFus    =   1.555              % Fuselage diameter, m
    Sfus    =   (pi/4)*lenFus*dFus % Plan or side area of fuselage, m
    Sbase   =   (pi/4)*dFus^2      % Fuselage cross-sectional area, m^2
    
    bHT     =   5.3;            % Horizontal tail span, m
    cHT     =   1.1;            % Mean horizontal tail chord, m
    swpHT   =   38*0.0174533;   % Horizontal tail sweep, rad
    SHT     =   bHT*cHT         % Horizontal tail area, m^2
    xHT     =   -11.3426;       % Linear xcp of horizontal tail 
    lHT     =   xcm - xHT;      % Horizontal tail length, m
    zHT     =   1.5;            % zcp of horizontal tail, m
    
    xVT     =   -10.044;        % Linear xcp of vertical tail, m
 	lVT		=	xcm - xVT;      % Vertical tail length, m
    bVT     =   2.409;          % Vertical tail span, m
    cVT     =   1.88;           % Mean vertical tail chord, m
    swpVT   =   50*0.0174533;   % Vertical tail sweep, rad
    SVT     =   bVT*cVT;        % Vertical tail area, m^2
    zVT     =   1.5;            % zcp of vertical tail, m
    
    xEng    =   -7.735;         % xcm of engine, m
    lEng    =   xcm - xEng;     % Engine length, m
    yEng    =   1.1325;         % ycm of engine, m
    zEng    =   0.4038;         % zcm of engine, m
    
    xNac        =   -7.7252;            % xcp of engine, m
    lNac        =   xcm - xNac;         % Nacelle length, m
    bNac        =   2.5;                % Nacelle span, m
    cNac        =   1.82;               % Nacellechord, m
    dNac        =   0.73;               % Nacelle diameter, m
    SbaseNac    =   0.25*pi*dNac^2;     % Nacelle base area, m^2
    Snac        =   bNac*cNac;          % Nacelle plan area, m^2
    
    xVent   =   -9.94;          % xcp of ventral fin, m
    lVent   =   xcm - xVent;    % Ventral fin length, m
    zVent   =   0;              % zcp of ventral fin, m
    bVent   =   1;              % Ventral fin span, m
    cVent   =   0.85;           % Ventral fin chord, m
    Svent   =   bVent*cVent;    % Ventral fin area, m^2
    swpVent =   60*0.0174533;   % Ventral sweep angle, rad
    
    Splan   =   S + Sfus + Snac + SHT + Svent   % Plan area of airplane
    Swet    =   2*(Splan + Sfus + SVT)          % Wetted area of airplane

    
    ARwing  =   (b^2) / S           % Wing aspect ratio
    ARHT    =   (bHT^2) / SHT       % Horizontal tail aspect ratio
    ARnac   =   (bNac^2) / Snac     % Engine nacelle aspect ratio
    ARvent  =   (bVent^2)/ Svent    % Ventral fin aspect ratio
    ARVT    =   (bVT^2) / SVT       % Vertical tail aspect ratio

%   Moments and Product of Inertia
    Ixx     =   (1/12)*((mWing+mFuel)*b^2 + mHT*bHT^2 + mVT*bVT^2) + (0.25*(mFus+mPay)*dFus^2 + mEng*yEng^2 + mVT*zVT^2)
    Iyy     =   (1/12)*((mFus+mPay)*lenFus^2 + (mWing+mFuel)*cBar^2 + mVT*cHT^2) + (mEng*lEng^2 + mHT*lHT^2 + mVT*lHT^2)
    Izz     =   (1/12)*((mFus+mPay)*lenFus^2 + (mWing+mFuel)*b^2 + mHT*bHT^2) + (mEng*lEng^2 + mHT*lHT^2 + mVT*lVT^2)
    Ixz     =   mHT*lHT*zHT + mVT*lVT*zVT + mEng*lEng*zEng
    
    dEmax   =   20 * D2R %   Maximum Elevator Deflection is ±20 deg
	dAmax   =   35 * D2R %   Maximum Aileron Deflection is ±35 deg
	dRmax   =   35 * D2R %   Maximum Rudder Deflection is ±35 deg
    
    save('InerGeo.mat','m','xcm','Ixx','Iyy','Izz','Ixz','cBar','b','S', ...
        'Splan','taperw','ARwing','sweep','xcp','lWing','GamWing','lHT','lVT')
    
    disp('===================================')
    disp('Aerodynamic Properties for BizJet B')
    disp('===================================')

%   BizJet B Aero Properties    
    AlphaTable	=	[-10 -8 -6 -4 -2 0 2 4 6 8 10 11 12 13 14 15 16 17 18 19 20 ...
					21 22 23 24 25 30 35 40 45 50 55 60 65 70 75 80 85 90];
    Points      =   length(AlphaTable);
    AlphaRad    =   0.0174533*AlphaTable;
    SinAlpha    =   sin(AlphaRad);
    CosAlpha    =   cos(AlphaRad);
    
    %   Newtonian Coefficients
    CN          =   2*(Splan/S)*SinAlpha.*SinAlpha;
    CN          =   CN.*sign(AlphaRad);
    CDNewt      =   2*(Splan/S)*abs(SinAlpha.*SinAlpha.*SinAlpha);
    CLNewt      =   CN.*CosAlpha;
    cpNewt      =   (S*lWing + Sfus*lFusN + SHT*lHT + Svent*lVent + Snac*lNac)...
                    / (S + Sfus + SHT + Svent + Snac);

%   Longitudinal Aerodynamics
%   =========================
%   Lift
    CLaWing =   pi*ARwing / (1 + sqrt(1 + (0.5*ARwing/cos(sweep))^2));
    deda    =   0; %   T-tail
    CLaHT   =   (1 - deda)*(pi*ARHT / (1 + sqrt(1 + (0.5*ARHT/cos(swpHT))^2)))*SHT / S;
    CLaFus  =   2*Sbase / S;
    CLaNac  =   2*SbaseNac / S;
    CLaVent =   (pi*ARvent / (1 + sqrt(1 + (0.5*ARvent/cos(swpVent))^2)))*Svent / S;
    CLaTot  =   CLaWing + CLaHT + CLaFus + CLaNac + CLaVent;
    %   Assume CL is linear to Alpha = 10 deg = 0.1745 rad
    CL10    =   CLaTot*10*D2R;
    
    %   Assume CL is symmetrically quadratic about CLmax
    CLmax   =   1.35;
    delAlph =   4*D2R;    % Stall occurs at 14 deg

    CLstatic        =   zeros(1,39);
    CLstatic(1:11)  =   CLaTot*AlphaRad(1:11);
    kStall          =   (CLmax - CL10)/delAlph^2;
    CLstatic(12:21) =   CLmax - kStall*(AlphaRad(15) - AlphaRad(12:21)).*(AlphaRad(15) - AlphaRad(12:21));
    CLstatic(22)    =   0.73;
    CLstatic(23)    =   0.73;
    CLstatic(24)    =   0.74;
    CLstatic(25)    =   0.76;
    CLstatic(26)    =   0.78;
    CLstatic(27:39) =   CN(27:39).*CosAlpha(27:39);
    figure
    CLTable         =   CLstatic;
    plot(AlphaTable,CLTable),grid, title('CLTable')
    
%   Drag
    Re      =   1.225*100*lenFus / 1.725e-5;    %   Reynolds number at 100 m/s
    Cf      =   0.46*(log10(Re))^(-2.58);       %   Flat plate friction coefficient
    CDf     =   Cf*Swet / S;
    CDbase  =   0.12*Sbase / S;                 %   Base pressure drag
    CDo     =   CDf + CDbase;
    OEF     =   1.78*(1 - 0.045*ARwing^0.68) - 0.64; 
                % Oswald Efficiency factor, Raymer (straight wing)
    CDstatic        =   zeros(1,39);
    CDstatic(1:10)  =   CDo + CLstatic(1:10).*CLstatic(1:10) / (OEF*pi*ARwing);
    CDstatic(11)    =   CDstatic(10)*1.15;
    CDstatic(12)    =   CDstatic(11)*1.15;
    CDstatic(13)    =   CDstatic(12)*1.15;
    CDstatic(14)    =   CDstatic(13)*1.15;
    CDstatic(15)    =   CDstatic(14)*1.1;
    CDstatic(16)    =   CDstatic(15)*1.1;
    CDstatic(17)    =   CDstatic(16)*1.1;
    CDstatic(18)    =   CDstatic(17)*1.1;
    CDstatic(19)    =   CDstatic(18)*1.1;
    CDstatic(20)    =   CDstatic(19)*1.1;
    CDstatic(21)    =   CDstatic(20)*1.1;
    CDstatic(22)    =   CDstatic(21)*1.1;
    CDstatic(23)    =   CDstatic(22)*1.1;
    CDstatic(24)    =   CDstatic(23)*1.1;
    CDstatic(25)    =   CDstatic(24)*1.1;
    CDstatic(26)    =   CDstatic(25)*1.1;
    CDstatic(26:39) =   2*(Splan/S)*abs(SinAlpha(26:39).*SinAlpha(26:39).*SinAlpha(26:39));
    CDNewt          =   2*(Splan/S)*abs(SinAlpha.*SinAlpha.*SinAlpha);
    figure
    CDTable         =   CDstatic;
    plot(AlphaTable,CDTable), grid, title('CDTable')
    CDTable         =   CDstatic;
    
%   Pitching Moment (c.m. @ wing c.p.)
    CmStatic        =   zeros(1,39);
    SM              =   (CLaWing*lWing + CLaFus*lFus + CLaHT*lHT + CLaNac*lNac + CLaVent*lVent) / ...
                        (cBar*(CLaWing + CLaFus + CLaHT + CLaNac + CLaVent));   
                        % Static Margin at Alpha = 0 deg
    CmStatic(1:21)  =   -(CLstatic(1:21).*cos(AlphaRad(1:21)) + CDstatic(1:21).*sin(AlphaRad(1:21)))*SM;
    CmStatic(22)    =   CmStatic(21)*0.9;
    CmStatic(23)    =   CmStatic(22)*0.9;
    CmStatic(24)    =   CmStatic(23)*0.9;
    CmStatic(25)    =   CmStatic(24)*0.9;
    CmStatic(26)    =   0.9*CmStatic(25) - 0.1*CN(26).*sign(AlphaRad(26))*cpNewt/cBar;
    CmStatic(27)    =   0.4*CmStatic(26) - 0.6*CN(27).*sign(AlphaRad(27))*cpNewt/cBar;
    CmStatic(28)    =   0.1*CmStatic(27) - 0.9*CN(28).*sign(AlphaRad(28))*cpNewt/cBar;     
    CN              =   2*(Splan/S)*SinAlpha.*SinAlpha;
    CmStatic(29:39) =   -CN(29:39)*cpNewt/cBar;
    figure
    CmTable         =   CmStatic;
    plot(AlphaTable,CmTable), grid, title('CmTable')

    
%   CmdETable & CLdEo, Elevator Effect
    tauDE           =   0.32;   % Geometric elevator chord/horizontal tail chord
    tauCO           =   0.68;   % Elevator Carryover effect
    Sigmoid         =   [];
    Sigmoid(1:5)    =   tauCO;
    Sigmoid(6:39)   =   tauDE + (tauCO - tauDE) ./ (1 + exp(-15.*(AlphaRad(26) - AlphaRad(6:39))));    
    CmdETable       =   zeros(1,39);
    CLdEo           =   tauCO*CLaHT;                % CLdE at Alpha = 0
    CmdEo           =   -tauCO*(lHT/cBar)*CLaHT;    % CmdE at Alpha = 0
    CmdETable(1:39) =   (CmdEo*Sigmoid).*cos(AlphaRad(1:39));
                                       % Elevator effect on moment, per rad
    CLdETable(1:39) =   (CLdEo*Sigmoid).*cos(AlphaRad(1:39));
                                       % Elevator effect on lift, per rad
    figure
    plot(AlphaTable,CLdETable), grid, title('CLdETable')
    figure
    plot(AlphaTable,CmdETable), grid, title('CmdETable')
    
%   Longitudinal Rotary & Unsteady Derivatives
    CLqHat      =   2*CLaHT*lHT/cBar;
    CmqHat      =   -CLqHat*lHT/cBar;

%   Lateral-Directional Aerodynamics
%   ================================
%   CYBetaTable, Side Force Sensitivity to Sideslip Angle
    EndPlate    =   1.1;    % End-plate effect of T-tail
    CYBetaVT    =   -EndPlate*(pi*ARVT / (1 + sqrt(1 + (0.5*ARVT/cos(swpVT))^2)))*SVT / S;
    CYBetaFus   =   -2*Sbase / S;
    CDoWing     =   0.005;
    CYBetaWing  =   -CDoWing - (GamWing^2)*pi*ARwing / (1 + sqrt(1 + ARwing^2));
    CYBetaVent  =   -0.4*CLaVent;
    CYBetao     =   CYBetaVT + CYBetaFus + CYBetaWing + CYBetaVent;
    CYBetaTable =   CYBetao*cos(AlphaRad);
    figure
    plot(AlphaTable, CYBetaTable), grid, title('CYBetaTable')
    
%   ClBetaTable, Roll Moment Sensitivity to Sideslip Angle
    ClBetaWing  =   -((1 + 2*taperw)/(6*(1 + taperw)))*(GamWing*CLaWing + (CLTable.*tan(sweep)));
    ClBetaWF    =   1.2*sqrt(ARwing)*(2*zWing*dFus/b^2);
    ClBetaVT    =   -zVT*CYBetaVT/b;
    ClBetao     =   (ClBetaWing + ClBetaWF + ClBetaVT);
    ClBetaTable =   ClBetao.*cos(AlphaRad);
    figure
    plot(AlphaTable, ClBetaTable), grid, title('ClBetaTable')

%   CnBetaTable, Yaw Moment Sensitivity to Sideslip Angle
    CnBetaWing  =   0.075*CLTable*GamWing;
    CnBetaFus   =   CLaFus*lFusN / b;
    CnBetaVT    =   -CYBetaVT*lVT / b;
    CnBetaVent  =   -CYBetaVent*lVent / b;
    CnBetaTable =   (CnBetaWing + CnBetaFus + CnBetaVT).*cos(AlphaRad);
    figure
    plot(AlphaTable, CnBetaTable), grid, title('CnBetaTable')
    
%   CldATable, Roll Moment Sensitivity to Aileron Deflection
    tauDA       =   0.25;
    kDA         =   0.38;
    CldAo       =   tauDA*(CLaWing/(1 + taperw))*((1 - kDA^2)/3 - (1 - kDA^3)*(1 - taperw)/3);
    CldATable   =   CldAo*cos(AlphaRad);
    figure
    plot(AlphaTable, CldATable), grid, title('CldATable')
    
    CYdAo       =   0;  %   Side force due to aileron, rad

%   CndATable, Yaw Moment Sensitivity to Aileron Deflection
%   Cessna 510 has an Aileron-Rudder Interconnect; assume CndA = 0
    CndATable   =   zeros(1,39);
    figure
    plot(AlphaTable, CndATable), grid, title('CndATable')
   
%   CldRTable, Roll Moment Sensitivity to Rudder Deflection
    tauDR       =   0.5;    % Geometric Rudder chord/horizontal tail chord
    tauCOR      =   0.8;    % Rudder Carryover effect
    CldRo       =   tauCOR*zVT*CYBetaVT / b;
    CldRTable   =   CldRo*cos(AlphaRad);
    figure
    plot(AlphaTable,CldRTable), grid, title('CldRTable')

%   CndRTable, Yaw Moment Sensitivity to Rudder Deflection
    CndRo       =   -tauCOR*CnBetaVT;
    CndRTable   =   CndRo*cos(AlphaRad);
    figure
    plot(AlphaTable, CndRTable), grid, title('CndRTable')
    
%   Lateral-Directional Rotary & Unsteady Derivatives
    CYrHat      =   -2*CYBetaVT*lVT/b;
    ClpHato     =   -(CLaWing + CLaHT*(SHT/S) - CYBetaVT*(SVT/S))* ...
                    ((1 + 3*taperw)/(1 + taperw))/12;
    ClpHatTable =   ClpHato*cos(AlphaRad);
    figure
    plot(AlphaTable, ClpHatTable), grid, title('ClpHat')
    ClrHato     =   -(CLaWing + CLaHT*(SHT/S) - CYBetaVT*(SVT/S))* ...
                    (1 + 3 * taperw)/(12 * (1 + taperw));    
    ClrHatTable =   ClrHato*cos(AlphaRad);    
    figure
    plot(AlphaTable, ClrHatTable), grid, title('ClrHatTable')
    CnpHatTable =   (- CLTable*((1 + 3*taperw)/(1 + taperw))/12) .* cos(AlphaRad);
    figure
    plot(AlphaTable, CnpHatTable), grid, title('CnpHatTable')
    CnrHatVT    =   -2*CnBetaVT*(lVT/b);
    CnrHatWing  =   -0.103*CLTable.*CLTable - 0.4*CDoWing; %   (from Seckel)
    CnrHato     =   CnrHatVT + CnrHatWing;
    CnrHatTable =   CnrHato .* cos(AlphaRad);
    figure
    plot(AlphaTable, CnrHatTable), grid, title('CnrHatTable')
     
    save('DataTable.mat','AlphaTable','CLTable','CDTable','CmTable','CmdETable',...
        'CYBetaTable','ClBetaTable','CnBetaTable','CldATable','CndATable','CldRTable','CndRTable')
    save('RotCont.mat','CLdEo','CYdAo','CLdETable','CLqHat','CmqHat','CYrHat','ClpHatTable', ...
        'ClrHatTable','CnpHatTable','CnrHatTable')
                                        