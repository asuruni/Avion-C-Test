%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%NOM:           MAIN_nouvelles_regles_iteratif.m (Dérivé de
%               MAIN_conditions_posees_avec_air.m de Frédéric Larocque?
%
%DESCRIPTION:   Programme de test pour estimer les performances des modèles 
%               itérés
%
%AUTEUR:        Laurent Emond-Brunet 
%
%DATE DE CRÉATION:  17-09-2019 (Dérivé de MAIN_nouvelles_regles.m)
%
%MODIFICATIONS:
%Super long à executer; verifier pourquoi

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
clc;

%% Itérations:
%Wingspan; en in
iter.b.min = 70;
iter.b.max = 120;
iter.b.delta = 20;
iter.b.nb = (iter.b.max - iter.b.min)/iter.b.delta;

%Payload; en lbs
iter.pl.min = 0;
iter.pl.max = 3; 
iter.pl.delta = 1;
iter.pl.nb = (iter.b.max - iter.b.min)/iter.b.delta;

%Ballons
iter.ball.min = 7;
iter.ball.max = 9;
iter.ball.delta = 1;
iter.ball.nb = (iter.b.max - iter.b.min)/iter.b.delta;

iter.total = iter.ball.nb * iter.pl.nb * iter.b.nb; %nombre total de combinaisons testées


%% initialisation variables de départ
aero.wing.profil = 'E423';
aero.vitesse = 12; %m/s
aero.wing.gap = 0.6;                                %espace entre les deux ailes (mètres)
aero.wing.stagger = 0;                              %Angle du décallage entre les deux ailes (rad)
aero.wing.aspect_ratio = 7.45;                      % fixed

%% Tests
for i = 1:iter.total
    for b = iter.b.min:iter.b.delta:iter.b.max
aero.wing.wingspan = b * 0.0254; % m

aero.tail.tail_type = 'normal';
geo.fuselage.hauteur_gear = ((4.5+0.5) * 2.54)/100; %m   %provient de Présentation DDR+ 0.5" de jeu
geo.fuselage.profil_fuselage = 'panneaux';       %utilisé pour calcul de drag du fuselage
geo.fuselage.arrondi_fuselage = 'demi';          %utilisé pour le calcul de drag du fuselage
aero.tail.cd_min_hstab = 0.012;               
aero.tail.cd_min_vstab = 0.008;
aero.Alpha_Runway = 0; %EN DEGRÉS
geo.hauteur_gear = 0.5;

aero.wing.oswald_efficiency = 1.78*(1-0.045*(aero.wing.aspect_ratio^0.68))-0.64;    %Formule semi empirique, RAYMER 5eme p.456 -- n'agit plus comme un input modifiable
geo.fuselage.aire_gear = (2*geo.hauteur_gear + (7*2.54)/100)*0.01; %m^2

%% Choix altitude avion et application facteurs de correction
load('ThrustCurves.mat');

%choix altitude
aero.altitude = 1300; %ft

aero.density=density_altitude(aero.altitude/3.28084);
etude.thrust = ThrustCurves{(aero.altitude/100)+1,2};
%thrust = lbf/ m/s

%facteur correction pour static thrust réduit à cause de l'interface
%fuselage-moteur
facteur_corr = 0.9;
etude.thrust = etude.thrust*facteur_corr;
        
        %Itération nb ballons
        for ball=iter.ball.min:iter.ball.delta:iter.ball.max
        etude.nb_ballons=ball;

        %Itération payload
        for p_l=iter.pl.min:iter.pl.delta:iter.pl.max
        etude.payload=p_l;      %[lbs]
        geo.longueur_queue=(27*2.54/100) ;  %posée comme conditions posées

        %Calcul de la forme du fuselage en fonction de la position des
        %ballons (design 2019-2020)
        etude.angle_ballons = 7;
        [A_rectangle,A_ellipse,F_rectangle,F_ellipse,longueur_totale,longueur_fairing,longueur_nose_cone,P]=configFuselage(etude.nb_ballons,etude.angle_ballons); %Dimensions en pouces [in^2]
        
        %Conversion en métrique
        aero.fuselage.s_wet = F_ellipse*0.00064516;    %[m^2]
        aero.fuselage.s_frontale= A_ellipse*0.00064516;    %[m^2]
        aero.fuselage.aire_totale = aero.fuselage.s_wet+aero.fuselage.s_frontale;
        aero.fuselage.longueur_totale = longueur_totale*0.0254;  %[m]
        
        geo.longueur_moteur = 3 * (2.54/100);       %vérifier si c'est cette valeur
        geo.longueur_electro = 3 * (2.54/100);      %vérifier valeur
        geo.longueur_cargo = longueur_totale-(longueur_fairing+longueur_nose_cone);
        geo.longueur_totale = longueur_totale;
        geo.s_aile = (((aero.wing.wingspan)^2)/aero.wing.aspect_ratio); 
               
        
        %% CALCUL DE LA MASSE A VIDE

            % Masses surfaciques 
            geo.masse.m_surf_aile          = 0.000752 * 703.08;               % kg/m^2   Masse de l'aile par unite de surface    %basé sur ancien Matlab
            geo.masse.m_lin_longeron       = 0.019 * 17.858;                 % kg/m     Masse du Longeron par unite de longueur     %basé sur valeur réelle 2018-2019
            geo.masse.m_lin_tail           = (362/1000)/(28*0.0254)*0.75; %ancienne valeur avec diminution de 75% visée          % kg/m     Masse de la queue par unite de longueur %basé sur estimation 2018-2019    %basé sur ancien matlab: (0.01 * 17.858; )
            geo.masse.m_lin_landing        =(1.1*0.453592)/(6.74*0.0254) ;     % kg/m     Masse du train d'atterissage en fonction de la hauteur  %basé sur Regular 2017-2018
            % Masses des equipements
            etude.masse.masse_moteur      = 1.30 / 2.2;             % kg  %masse réelle moteur+hélice          Masse du moteur +hélice (0.8 lb pour S 4025, 1.38 lb pour HK 5025, 0.8377 SII-4035-380kv, 0.266lb hélice))
            etude.masse.masse_batterie    = (0.921 + 0.251) / 2.2;          % kg           Masse de la batterie + speed controler
            etude.masse.masse_noseAssy    = 0 / 2.2;                  % kg           Masse du nose avec le engine mount
            etude.masse.masse_servo       = 0.4 / 2.2;%0.275;           % kg          Masse de l'electronique (cf. fichier Bilan_Masses_Red_Eagle)
            etude.masse.masse_autre       = (0.6 / 2.2)+ 0.5/2.2;       % kg  %masse de 0.5kg fixation aile         Masse monokote + masse fixation aile (mesure 19/01/19 Département structure)
            etude.masse.masse_equipement = etude.masse.masse_moteur + etude.masse.masse_batterie + etude.masse.masse_noseAssy + etude.masse.masse_servo + etude.masse.masse_autre;

            %masses 
            %masse_aile = 2.4; %donnees reelles 19/01/19 
            etude.masse.masse_aile = geo.masse.m_surf_aile * geo.s_aile;
            %masse_longeron =0; 
            etude.masse.masse_longeron = geo.masse.m_lin_longeron * (96*2.54/100);    %longeron est 8" de long
            %masse_queue = 0.723/2.2; %mesure masse finale sur avion 2018-2019
            etude.masse.masse_queue = geo.masse.m_lin_tail * geo.longueur_queue;
            etude.masse.masse_gear = 0.802/2.2; %prise mesure avion final 2018-2019

            etude.masse.masse_avion_vide = etude.masse.masse_aile + etude.masse.masse_longeron + etude.masse.masse_queue + etude.masse.masse_gear + etude.masse.masse_equipement-0.5; %kg  %manque masses du stabh, stab v et du fuselage (normal)
            etude.masse.masse_totale = etude.masse.masse_avion_vide + etude.payload * 0.453592 + etude.nb_ballons * 0.45; %kg
            
            
            
%% Performance
resultats(i,:) = analyse (etude, aero, geo);

        end
        end
end
end
%% Identification de la meilleure

[score, i] = max([resultats{:,1}]);%.*[resultats{:,5}]);
fprintf('Meilleure combinaison à i = %d\nScore de %d points\nDécolle : %d\n',i, score, resultats{i,5});

