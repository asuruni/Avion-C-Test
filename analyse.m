function resultats = analyse(etude, aero, geo)

%% Recup

%variables pour folders
folder_profils = 'profils';         %folder ou sont les donnees des profils
folder_mat = 'mat files';           %folder ou sont les fichiers mat
%crée le path pour aller sauvegarder/get les fichier .mat
%path_mat=fullfile(folder_mat,['Moy_',aero.wing.profil,'_',num2str(aero.wing.aspect_ratio),'_',num2str(aero.wing.oswald_efficiency),'_',num2str(aero.Alpha_Runway),'.mat']);
path_mat=fullfile(folder_mat,'Moy_AH79100b_E423_80 20_7.45_0.82617_0.mat');

%loader variables du fichier info.mat
load(path_mat);

aero.wing.AoA_Cl_max = donnees_profil{1}(2) * ((2*pi)/360);                %passage en radians
aero.wing.AoA_Cd_min = donnees_profil{1}(4) * ((2*pi)/360);
aero.wing.Pente_Cl_AoA = donnees_profil{1}(10) * (360/(2*pi));
aero.wing.ZeroLiftAoA= donnees_profil{1}(11)* ((2*pi)/360);

%donnees_profil{1}(1)=Cl_max;
%donnees_profil{1}(2)=min(AoA_Cl_max);
%donnees_profil{1}(3)=Cd_min;
%donnees_profil{1}(4)=max(AoA_Cd_min);
%donnees_profil{1}(5)=Cm_L_D_max;
%donnees_profil{1}(6)=delta_AoA;
%donnees_profil{1}(7)=LDB;
%donnees_profil{1}(8)=Cl_runway_2D;
%donnees_profil{1}(9)=Cd0_0AOA;
%donnees_profil{1}(10)=Pente_Cl_AoA;
%donnees_profil{1}(11)=ZeroLiftAoA;
%donnees_profil{1}(12)=Cl_0AoA;

%facteur de sécurité de 10% sur le lift
corr_lift = 0.9;
aero.wing.Cl_max = donnees_profil{1}(1) * corr_lift;
aero.wing.Cl_runway_2D = donnees_profil{1}(8) * corr_lift;
aero.wing.Pente_Cl_AoA = donnees_profil{1}(10) * corr_lift;
aero.wing.Cl_0AoA = donnees_profil{1}(12) * corr_lift;

%pour tenir compte des erreurs de XFLR5, 20% sur la trainee
corr_drag = 1.2;
aero.wing.Cd0_0AOA= donnees_profil{1}(9) * corr_drag;
aero.wing.Cd_min= donnees_profil{1}(3) * corr_drag;

%crée le path pour aller sauvegarder/get les fichier .mat
path_mat=fullfile(folder_mat,'CmMoyen.mat');
load(path_mat); 
aero.wing.cm_average = cm_average;

%% PRESSION DYNAMIQUE ET CONSTANTES (SECTION 2)
%Initialisation des constantes atmosphériques, en unités du SI. Ces données
%seront réutilisées plus tard dans les calculs.

aero.kin_viscosity = 1.46*(10^(-5)) ;

%density = 1.2074 ;
aero.mach = aero.vitesse / 340 ;
aero.q = 0.5 * aero.density * aero.vitesse^2 ;

%% ÉTABLISSEMENT DU LIFT 3D (SECTION 4)
%Approximation des coefficients de portance 3D à partir des données 2D et
%des caractéristiques de l'aile.

rendement_profil = aero.wing.Pente_Cl_AoA / (2*pi);                                                                     %pente_cl_alpha / (2*pi)
pente_CL_3D = (2*pi*aero.wing.aspect_ratio) / (2 + sqrt(4+(((aero.wing.aspect_ratio)^2 * (1-aero.mach^2)^2)/(rendement_profil)^2))) ;           %basé sur Raymer p.412

%Angle d'attaque 0 lift 2D
alpha_zero = aero.wing.ZeroLiftAoA ;

%Angle 'Attaque au Cd minimum 2D
alpha = aero.wing.AoA_Cd_min;

%Boucle qui permet de créer une courbe de points du coefficient de portance
%3D. La courbe est définie jusqu'à l'angle de CL_max 2D.
i = 1;
equation_lift_3D=zeros(1,(floor((alpha-alpha_zero)*1000)));
while alpha < aero.wing.AoA_Cl_max 
    equation_lift_3D(i) = (alpha-(-aero.wing.Cl_0AoA/aero.wing.Pente_Cl_AoA))*pente_CL_3D ;   %LE +500 EST POUR ÉLIMINER LES INDEXATIONS À UNE POSITION NÉGATIVE
    alpha = alpha + 0.001;
    i = i+1;
end

if aero.Alpha_Runway*2*pi/360<aero.wing.AoA_Cd_min
    error('Angle d''incidence trop petit')
end

%Établissements des CL 3D importants à partir des valeurs 2D entrées en geoètres dans la fonction
aero.wing.CL_runway_3D = equation_lift_3D(round(((aero.Alpha_Runway*2*pi/360)-aero.wing.AoA_Cd_min)/0.001)) ;      %CL à Alpha_Runway
aero.wing.CL_takeoff_3D = equation_lift_3D(end) ;    


%% CALCUL DE TRAINEE TOTALE ET DIMENSIONNEMENT STABS (SECTION 5)
%Bloc permettant de calculer et de stocker toutes les données relatives à
%la traînée de l'avion. Sort aussi les caractéristiques dimensionnelles des
%surfaces des stabilisateurs.

%Fonction qui s'occupe de calculer les traînées. Voir fonction drag.m pour
%précisions supplémentaires.
[aero,etude,geo] = drag_6(aero.vitesse,aero,etude,geo);

%Dimensions du stabilisateur horizontal
% s_ht = 0.275;     %s_ht = drags{9};
% m_ht = 0.672/2.2;    %masse mesurée sur vrai avion %m_ht = drags{11};
aero.tail.AR_ht = aero.wing.aspect_ratio * 0.75;
aero.tail.span_ht = sqrt(aero.tail.AR_ht*aero.tail.s_ht);
aero.tail.chord_ht = aero.tail.s_ht / aero.tail.span_ht;

%Dimensions du stabilisateur vertical
% s_vt = 0.1012; %s_vt = drags{10};
% m_vt = 0.163/2.2; %masse mesuree sur vrai avion %m_vt = drags{12};
aero.tail.AR_vt = aero.wing.aspect_ratio * 0.25;
aero.tail.span_vt = sqrt(aero.tail.AR_vt*aero.tail.s_vt);
aero.tail.chord_vt = aero.tail.s_vt / aero.tail.span_vt;

%masse du fuselage
% m_fuselage = 4.057/2.2; %prise mesure avion final REVOIR POUR 2019-2020

%Légende des sorties de la fonction drag_3.m pour aider à s'y retrouver !
%drags{1} = drag_fuselage; 
%drags{2} = drag_base;
%drags{3} = drag_wing;
%drags{4} = drag_induced;
%drags{5} = drag_hstab; 
%drags{6} = drag_vstab; 
%drags{7} = drag_gear; 
%drags{8} = drag_total; 
%drags{9} = s_ht;
%drags{10} = s_vt;
%drags{11} = m_ht;
%drags{12} = m_vt;
%drags{13} = m_fuselage;


%% CALCUL DU DRAG BUDGET (SECTION 6)
%Bloc très simple qui attribue un pourcentage de la trainée totale de
%l'avion à chaque source de trainée. Le résultat est un "budget de
%trainee".

budget = struct() ;
budget(1).type = 'Fuselage' ;
budget(1).pourcentage = (aero.drag.drag_fuse / aero.drag.drag_total) * 100;
budget(2).type = 'Wing' ;
budget(2).pourcentage = (aero.drag.drag_wing / aero.drag.drag_total) * 100;
budget(3).type = 'Wing' ;
budget(3).pourcentage = (aero.drag.drag_wing / aero.drag.drag_total) * 100;
budget(4).type = 'H-Stab' ;
budget(4).pourcentage = (aero.drag.drag_hstab / aero.drag.drag_total) * 100;
budget(5).type = 'V-Stab' ;
budget(5).pourcentage = (aero.drag.drag_vstab/ aero.drag.drag_total) * 100;
budget(6).type = 'Gear' ;
budget(6).pourcentage = (aero.drag.drag_gear / aero.drag.drag_total) * 100;


%% COURBE DE THRUST (SECTION 7)
%Integration de la courbe de thrust obtenue expérimentalement au programme.
%***Possibilité d'être intégrée comme geoètre d'entrée.

% Polynome d'interpolation du thrust en fonction de la vitesse
% - Courbe basee sur les résultats des tests moteurs de 2013 avec l'helice 
%   {} ==> courbe parabolique obtenue
poly_thrust       = etude.thrust*4.44822;%[ 0.0005   -0.0203    0.0550   10.7915 ]*4.44822; %N / m/s

%% LIFT TOTAL ET VITESSE EN BOUT DE PISTE (SECTION 8)

%COPIÉ (tres fortement inspiré) DE LA FONCTION Analyse_Decollage.m; le 21
%juin 2017.


%valeurs importantes pour le decollage
downlift_elevateur = aero.q * 0.7 * aero.tail.s_ht;                        %cl du H-Stab estimé à 0.7
cd_max_hstab = 0.025;                                       %fixé approximativement
aero.stall_speed = sqrt((etude.masse.masse_totale*9.8 + downlift_elevateur)/ (0.5 * aero.density * geo.s_aile * aero.wing.CL_takeoff_3D))*1.1 ;   % vitesse minimale de controle


%temps rotation
t_rot = 0.4; %sec

dt = 0.01;                     %possibilité d'être augmenté pour améliorer le cpu time
i = 1;

%conditions initiales
time(i)     = 0;    %s
position(i) = 0;
position_x(i) = 0;    %m
position_y(i) = 0;
speed_x(i)    = 0;%15/3.6;    %m/s
speed_y(i)    = 0;%15/3.6;    %m/s
speed(i) = sqrt(speed_x(i).^2+speed_y(i).^2);
thrust(i)   = Thrust_Curve(0, poly_thrust);  %N
lift(i)     = 0;        %N
trainee(i)     = 0;     %N
rolling_resistance(i) = 0.05 * etude.masse.masse_totale;
accelx(i)    = thrust(i) / etude.masse.masse_totale ;
accely(i)    = 0;
theta(i) = 0; %rad
gamma(i) = atan(speed_y(i)/speed_x(i)); %rad
alpha(i) = gamma(i)-theta(i); %rad

while (speed(i) < 1.1*aero.stall_speed) && (position(i)< (30 - speed(i)*t_rot))

    %vitesses et angles
    speed_x(i+1)    = speed_x(i)    + accelx(i) * dt;
    speed_y(i+1)    = speed_y(i)    + max([0 accely(i)]) * dt;
    speed(i+1) = sqrt(speed_x(i).^2+speed_y(i).^2);
    gamma(i) = atan(speed_y(i)/speed_x(i));
    
    position_x(i+1) = position(i) + speed_x(i) * dt;
    position_y(i+1) = position(i) + speed_y(i) * dt;
    time(i+1,1)   = time(i)     + dt;
    
    thrust(i+1) = Thrust_Curve(speed(i), poly_thrust);
    lift(i+1) = 0.5 * aero.density * (speed(i+1))^2 * geo.s_aile * aero.wing.CL_runway_3D ;
    [aero,etude,geo] = drag_6(speed(i+1),aero,etude,geo);
    trainee(i+1) = aero.drag.drag_total;    
    %trainee_elevateur = ((cd_max_hstab - cd_min_hstab))*q*s_ht;               %trainee supplementaire du h-stab en rotation
    rolling_resistance(i+1) = 0.05 * (etude.masse.masse_totale-lift(i+1)) ;
    accelx(i+1) = (thrust(i+1)- trainee(i+1) - max([0,rolling_resistance(i+1)])) / etude.masse.masse_totale ;
    accely(i+1) = (- 9.8*etude.masse.masse_totale + lift(i+1)) / etude.masse.masse_totale;
    position(i+1) = sqrt(position_x(i).^2+position_y(i).^2);
    
    i=i+1;
end

%Angle de montée
etude.climb_angle = ((thrust(end) - trainee(end))/ (etude.masse.masse_totale*9.8))*(180/pi);

%Portance totale au décollage
etude.v_takeoff = speed(end) ;
L = lift(end) ;

%% VERIFICATION DES ERREURS POTENTIELLES (SECTION 9)

%teste si la vitesse entrée comme geoètre à la fonction est réaliste
if etude.v_takeoff < 0.8 * aero.vitesse || etude.v_takeoff > 1.3 * aero.vitesse ;         %pour s'assurer que la vitesse supposee est realiste OU faire une boucle !
%     warning('La vitesse d"entree de la fonction est irréaliste, la vitesse de décollage calculée est de %.2f m/s. La vitesse normale est %.2f m/s. \n', etude.v_takeoff,vitesse);
end

%teste si l'avion décolle réellement dans la distance impartie et stocke
%cette distance comme étant la distance nécessaire pour décoller.
decollage = 0;
distance_takeoff=0;
if etude.climb_angle >= 2 && etude.v_takeoff>= 1.1*aero.stall_speed && (position(end)+speed(end)*t_rot <60)
    
    decollage = 1 ;
    distance_takeoff = position(end)+speed(end)*t_rot;
    decollage_distance = sprintf('L''avion decolle apres %.2f m avec un angle de %.1f degres', distance_takeoff, etude.climb_angle);
else
    etude.v_takeoff = 0 ;
    decollage_distance = sprintf('L''avion decolle apres %.2f m avec un angle de %.1f degres', distance_takeoff, etude.climb_angle);
    decollage = 0 ;
    etude.climb_angle=0;  %code pour savoir qu'il n'y a pas de décollage
end


score = 120*(2*etude.nb_ballons+etude.payload)/(aero.wing.wingspan*100/2.54+geo.longueur_cargo*100/2.54);

resultats = {score, etude.payload, etude.nb_ballons, aero.wing.wingspan*100/2.54, decollage, geo, aero, etude, aero.drag};