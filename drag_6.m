function [aero,etude, geo] = drag_6(vitesse,aero,etude,geo)

%**************************************************************************

%-> COMMENTAIRE : Calcul du drag total et par composantes de l'avion en fonction de différents
%                 geoètres. Calcule aussi les surfaces des
%                 stabilisateurs.

%-> AUTEUR  : William Désilets.
%-> CRÉATION : Juillet 2017.
%-> DERNIÈRE MODIFICATION : 1 octobre 2017.
%-> UNITÉS : Système international.
%
%MODIFICATIONS: (16/09/2019) Par Laurent Emond-Brunet
%               Ajustement aux règles de 2019-2020

%**************************************************************************
%% PRESSION DYNAMIQUE ET CONSTANTES (SECTION 2)

%Géométrie du fuselage
[A_rectangle,A_ellipse,F_rectangle,F_ellipse,longueur,longueur_fairing,longueur_nose_cone,P]=configFuselage(etude.nb_ballons,etude.angle_ballons); %Dimensions en pouces [in^2] %Dimensions en pouces [in^2]

%Conversion en métrique
s_wet = F_ellipse*0.00064516;    %[m^2]
s_frontale= A_ellipse*0.00064516;    %[m^2]
aire_totale = s_wet+s_frontale;

 geo.masse.m_surf_fuselage = 0.002309468 * 703.08;           % kg/m^2    Masse du fuselage par unité de surface, déduit de l'ancien matlab
 etude.masse.m_fuselage = geo.masse.m_surf_fuselage * aire_totale*1.15; %surestimation de 25 % de la masse du fuselage;

aero.wing.chord = geo.s_aile / sqrt(aero.wing.aspect_ratio*geo.s_aile) ;
moment_arm = 0.55 * geo.longueur_totale;   % raymer p.160 %modifié pour queue 35"

%% Trainée de l'aile
%Trainée de la forme:
cd0 = aero.wing.Cd0_0AOA ;
drag_aile_f = cd0 * geo.s_aile * aero.q ;

%Trainée induite:
cdi = aero.wing.CL_runway_3D / (pi*aero.wing.aspect_ratio*aero.wing.oswald_efficiency) ;
drag_aile_i = cdi * geo.s_aile * aero.q ;

%Skin friction:
    grain_size=20*10^-6;            %[microns] surface de peinture d'avion en mass production Hoerner p.5-3
    Cf_skin = 0.032*grain_size^(1/5);    %Doûteux, à revoir
    drag_aile_s = Cf_skin*aero.q*2*geo.s_aile;   %Trainée de la friction de l'aile (dessus et dessous)

%Composantes additionnelles:
    %Trous servos
    Cd_trous_s = 0.04;               	%Hoerner p.5-11
    surf_trou_s = 2*0.06*0.04;          %[m^2] pour 2 servos (un sur chaque aile)
    drag_servos = Cd_trous_s*aero.q*surf_trou_s;    
    
    %Hinges des ailerons
    Cd_hinges = 0.5;                    %Hoerner p. 14-4
    surf_hinges = 2*0.03*0.005;          %Surface frontale [m^2] pour 2 hinges (un sur chaque aile)
    drag_hinges = Cd_hinges*aero.q*surf_hinges; 
    
    %Gaps devant les ailerons
    Cd_gap_f = 0.025;                   %Hoerner p. 14-4
    span_ailerons = 0.45;                %[m] REVOIR
    fente_ailerons = 0.01;                        %[m] largeur de la fente REVOIR
    surf_gap_f = 2*span_ailerons*fente_ailerons;    %[m^2] 2 ailerons
    drag_gap_f = Cd_gap_f*aero.q*surf_gap_f;
    
    %Gap côté des ailerons
    Cd_gap_s = 0.5;                     %Hoerner p. 14-4
    chord_ailerons = 0.15;              %[m] REVOIR
    fente_ailerons = 0.01;              %[m] largeur de la fente REVOIR
    surf_gap_s = 4*span_ailerons*fente_ailerons;   %[m^2] 4 rebords latéraux d'ailerons
    drag_gap_s = Cd_gap_s*aero.q*surf_gap_s;
    
    drag_aile = drag_aile_f + drag_aile_i + drag_aile_s + drag_servos + drag_hinges + drag_gap_f + drag_gap_s;
    
%% Trainée de fuselage

%Corps principal:
    %Face, considéré comme une section cylindrique
    Cd_corps = 0.7 ;             %p. 3-12 et ce qui s'est fait avant, Pointe plus ou moins aiguisée non arrondi
    drag_corps = s_frontale*aero.q*Cd_corps;

%Skin friction:
    grain_size=20*10^-6;            %[microns] surface de peinture d'avion en mass production Hoerner -> monokote? p.5-3
    Cf_skin = 0.032*grain_size^(1/5);    %Doûteux, à revoir
    Cd_wet = 1.07*Cf_skin;               %1.07 facteur "streamline" (à revoir)
    drag_fuse_s = Cf_skin*aero.q*2*s_wet;   %Trainée de la friction de l'aile (dessus et dessous)
    
%Appendices
    %Soute à cargo ?
%     if soute=='Oui'
%         Cd_soute = 0.21;                 %p.13-2 Représenté par un canopé aux rebords carrés (si arrondi, 0.1)
%         surf_soute = largeur_soute*longueur_soute;  %Aire frontale de la soute
%         drag_soute = Cd_soute*q*surf_soute;  
%     else 
        drag_soute=0;
%     end
    
%Interférence aile-fuselage 
    %Si biplan, est-ce que les deux ailes coupent le fuselage?
    Cd_interf = 0.004;                      %p.8 -15 REVOIR
    largeur_fuse = 11*0.0254;
    drag_interf_fuse_aile = Cd_interf*aero.q*(largeur_fuse/aero.wing.wingspan)*aero.wing.chord;
    
%Moteur
    Cd_moteur = 0.49; %p. 13 - 4 uncowled motor
    surf_moteur = 0.035^2*pi;       %REVOIR
    drag_moteur = Cd_moteur*aero.q*surf_moteur;
    
%Train d'atterrissage
    %Structure
    Cd_struct = 0.25;            %p.13 -14
    surf_struct = 0.14*0.005;    %REVOIR
    drag_struct = Cd_struct*aero.q*surf_struct;

    %Roue
    Cd_roue = 0.25;              %p. 13 -14
    surf_roue = 0.02*0.06;       %REVOIR
    drag_roue = Cd_roue*aero.q*surf_roue;
    
drag_fuse_0 = drag_corps +  drag_fuse_s + drag_soute + drag_interf_fuse_aile + drag_moteur + drag_struct + drag_roue;
drag_fuse = 1.1*drag_fuse_0;         %Facteur de 10% à cause de l'effet de la propulsion qui augmente la pression sur le fuselage (p. 14-5)

%% Stabilisateurs

%trainee de l'élévateur et du rudder (pris de l'ancien Matlab)   
    aero.tail.c_ht = 0.45 ;                           % raymer p.160
    aero.tail.c_vt = 0.022 ;                          % raymer p.160
    aero.wing.chord = geo.s_aile / sqrt(aero.wing.aspect_ratio*geo.s_aile) ;
    moment_arm = 0.55 * geo.longueur_totale;   % raymer p.160 %modifié pour queue 35"
%dimensions des stabilisateurs
    aero.tail.s_ht = aero.tail.c_ht * aero.wing.chord * geo.s_aile / moment_arm ;
    aero.tail.s_vt = aero.tail.c_vt * (geo.s_aile/aero.wing.chord) * geo.s_aile / moment_arm ; 

%masse des stabilisateurs
    geo.masse.m_surf_rudder         = 1.009450475; %((105.3-28)/1000)/(14.15*7.13*0.00064516);         % kg/m^2    Masse du rudder par unite de surface   % Basé sur regular 2017-2018    %Ancienne estimation:(0.00133/2) * 703.08
    geo.masse.m_surf_elevateur      = 0.794304706; %((319.5-53)/1000)/(51.4*8.6*0.00064516);              % kg/m^2    Masse de l'elevateur par unite de surface % Basé sur regular 2017-2018    %Ancienne estimation: 0.0015 * 703.08; 
    etude.masse.m_ht = geo.masse.m_surf_elevateur * aero.tail.s_ht;
    etude.masse.m_vt = geo.masse.m_surf_rudder * aero.tail.s_vt;

%Pas de drag induit car techniquement les surfaces stabilisatrices ne
%devraient pas générer de forces portantes en vol, seulement en virages et
%en rotation.
    drag_hstab = aero.tail.s_ht * aero.tail.cd_min_hstab * aero.q ;
    drag_vstab = aero.tail.s_vt * aero.tail.cd_min_vstab * aero.q ;

%Skin friction:
    grain_size=20*10^-6;            %[microns] surface de peinture d'avion en mass production Hoerner p.5-3
    Cf_skin = 0.032*grain_size^(1/5);    %Doûteux, à revoir
    drag_stab_s = Cf_skin*aero.q*2*(aero.tail.s_vt+aero.tail.s_ht);   %Trainée de la friction des stabs (dessus et dessous)

%Gap devant elevator
    Cd_gap_f = 0.025;                   %Hoerner p. 14-4
    span_elevator = 0.75;                %[m] REVOIR
    fente_elevator = 0.01;                        %[m] largeur de la fente REVOIR
    surf_gap_f_elev = 2*span_elevator*fente_elevator;    %[m^2] 2 ailerons
    drag_gap_f_elev = Cd_gap_f*aero.q*surf_gap_f_elev;

%Gap devant rudder
    Cd_gap_f = 0.025;                   %Hoerner p. 14-4
    span_rudder = 0.30;                %[m] REVOIR
    fente_rudder = 0.01;                        %[m] largeur de la fente REVOIR
    surf_gap_f_rudd = 2*span_rudder*fente_rudder;    %[m^2] 2 ailerons
    drag_gap_f_rudd = Cd_gap_f*aero.q*surf_gap_f_rudd;

%Trous servos
    Cd_trou_s = 0.04;               	%Hoerner p.5-11
    surf_trou_s = 2*0.06*0.04;          %[m^2] pour 2 servos (un sur chaque aile)
    drag_servos_stab = Cd_trou_s*aero.q*surf_trou_s;    
    
%Hinges des ailerons
    Cd_hinges = 0.5;                    %Hoerner p. 14-4
    surf_hinges = 2*0.03*0.005;          %Surface frontale [m^2] pour 2 hinges (un sur chaque aile)
    drag_hinges_stab = Cd_hinges*aero.q*surf_hinges; 
    
drag_stabs = drag_hstab + drag_vstab + drag_stab_s +  drag_gap_f_elev + drag_gap_f_rudd + drag_servos_stab + drag_hinges_stab;

%% Total
drag_total = drag_aile + drag_fuse + drag_stabs;

%% STOCKAGE DES RESULTATS FINAUX (SECTION 9)

drags = {};

aero.drag.drag_fuse = drag_fuse; 
aero.drag.null = 0;                %retrait d'un type de drag, 0 pour éviter d'avoir trop de changements d'indices à faire
aero.drag.drag_wing = drag_aile;
aero.drag.drag_stabs = drag_stabs;
aero.drag.drag_hstab = drag_hstab; 
aero.drag.drag_vstab = drag_vstab; 
aero.drag.drag_gear = drag_roue; 
aero.drag.drag_total = drag_total; 
aero.tail.s_ht = aero.tail.s_ht;
aero.tail.s_vt = aero.tail.s_vt;
etude.masse.m_ht = etude.masse.m_ht;
etude.masse.m_vt = etude.masse.m_vt;
etude.masse.m_fuselage = etude.masse.m_fuselage;
etude.geo.moment_arm = moment_arm;

