
function [resultats,score_normal,score_biplan] = analyse_9(aero,etude,param,a_r,n_b,p_l,ang,w_s)

%**************************************************************************

%-> COMMENTAIRE : Fonction qui calcule toutes les caractéristiques
%importantes de l'avion de manière préliminaire en fonction des paramètres
%entrés. Les sorties comportent, entre autres, un budget de traînée, la
%distance de décollage, la vitesse de décollage, les caractéristiques
%géométriques et aérodynamiques de l'aile, etc.

%-> AUTEURS  : William Désilets et Cédric Dionne
%-> CRÉATION : Juin 2017.
%-> DERNIÈRE MODIFICATION : 6/11/19.
%-> UNITÉS : Système international.

%
%       Modifié le 6/11/19 par Laurent Emond-Brunet
%       Change notification: Fusion des variables en format struct avec le
%       calcul d'influence du biplan

%**************************************************************************

%Mettre à 'on' si on teste pour une seule configuration. Si la fonction
%analyse.m est utilisée dans une boucle, s'assurer de laisser ces deux
%variables à 'off'.

%graphiques = 'off';
%spam = 'on';

%% DONNEES DE BASE DU PROFIL (SECTION 1) 
%Section qui permet d'aller chercher toutes les données relatives au profil
%étudié. Les données angulaires sont passées en radians pour les calculs à suivre.

%variables pour folders
folder_profils = 'profils';         %folder ou sont les donnees des profils
folder_mat = 'mat files';           %folder ou sont les fichiers mat
%crée le path pour aller sauvegarder/get les fichier .mat
path_mat=fullfile(folder_mat,['Moy_',aero.wing.profil,'_',num2str(aero.wing.aspect_ratio),'_',num2str(aero.wing.oswald_efficiency),'_',num2str(aero.Alpha_Runway),'.mat']);

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
aero.wing.Cl_max = donnees_profil{1}(1) * 0.90;
aero.wing.Cl_runway_2D = donnees_profil{1}(8) * 0.90;
aero.wing.Pente_Cl_AoA = donnees_profil{1}(10) * 0.90;
aero.wing.Cl_0AoA = donnees_profil{1}(12) * 0.90;

%pour tenir compte des erreurs de XFLR5, 20% sur la trainee
aero.wing.Cd0_0AOA= donnees_profil{1}(9) *1.2;
aero.wing.Cd_min= donnees_profil{1}(3) *1.2;

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


%% DIMENSIONNEMENT DE L'AVION (SECTION 3)                     
%Bloc permettant de dimensionner l'avion en fonction de la configuration
%choisie et du nombre de balles. Les données géométriques de l'aile, du
%fuselage ainsi que les masses préliminaires sont définies. [À TRAVAILLER. PAYLOAD À INCLURE]

etude.geo.longueur_moteur = 3 * (2.54/100);       %vérifier si c'est cette valeur
etude.geo.longueur_electro = 3 * (2.54/100);      %vérifier valeur

%config_balles='tri';
%if config_balles =='tri'
%    longueur_cabine_balles = (3*sqrt(3)*6.5/(2*100))+((6.5 * ((nb_balles-6)/largeur_balles))/100) ; %configuration 1-2-3-4x...
%else
%    longueur_cabine_balles = (6.5 * (nb_balles/largeur_balles)) / 100 ;      % configuration 4x...
%end

%nb_balles_temp=54;
%longueur_cabine_balles = 1.17 %tiré du CAD 19/01/19 %2*(3*sqrt(3)*6.5/(2*100))+((6.5 * ((nb_balles_temp-12)/largeur_balles))/100);    %on pose configuration 54 balles avec triangle en avant et en arrière

%La longueur du cargo est maintenant définie dans MAIN_nouvelles_regles

etude.geo.longueur_totale = etude.geo.longueur_cargo + etude.geo.longueur_moteur + etude.geo.longueur_electro + etude.geo.longueur_queue; %longueur supplementaire ou il n<y a pas de balles

%Calculs des propriétés importantes de l'aile, en SI et en pouces.
if aero.wing.aile=='normal'
    if etude.nb_ballons~=0
        aero.wing.span = sqrt(aero.wing.aspect_ratio*(aero.wing.s_aile+(etude.geo.largeur_ballons*22.6*10^-2*24.89*2.54*10^-2)));
    else
        aero.wing.span = sqrt(aero.wing.aspect_ratio*(aero.wing.s_aile+(etude.geo.largeur_payload*22.6*10^-2*24.89*2.54*10^-2))); %Ajout 2019-2020 au cas où aucun ballon
    end
elseif aero.wing.aile=='biplan'
    if etude.nb_ballons~=0
        aero.wing.span = sqrt(aero.wing.aspect_ratio*(aero.wing.s_aile+(etude.geo.largeur_ballons*22.6*10^-2*24.89*2.54*10^-2)))/2;
    else
        aero.wing.span = sqrt(aero.wing.aspect_ratio*(aero.wing.s_aile+(etude.geo.largeur_payload*22.6*10^-2*24.89*2.54*10^-2))/2); %Ajout 2019-2020 au cas où aucun ballon
    end
end

aero.wing.span_in = aero.wing.span*100 / 2.54;

if aero.wing.aile=='normal'
    aero.wing.chord = aero.wing.s_aile / aero.wing.span;
elseif aero.wing.aile=='biplan'
    aero.wing.chord = (aero.wing.s_aile/2) / aero.wing.span;
end
aero.wing.chord_in = aero.wing.chord*100 /2.54;


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

%Établissements des CL 3D importants à partir des valeurs 2D entrées en paramètres dans la fonction
aero.wing.CL_runway_3D = equation_lift_3D(round(((aero.Alpha_Runway*2*pi/360)-aero.wing.AoA_Cd_min)/0.001)) ;      %CL à Alpha_Runway
aero.wing.CL_takeoff_3D = equation_lift_3D(end) ;    


%% CALCUL DE TRAINEE TOTALE ET DIMENSIONNEMENT STABS (SECTION 5)
%Bloc permettant de calculer et de stocker toutes les données relatives à
%la traînée de l'avion. Sort aussi les caractéristiques dimensionnelles des
%surfaces des stabilisateurs.

%Fonction qui s'occupe de calculer les traînées. Voir fonction drag.m pour
%précisions supplémentaires.
[aero,etude,param] = drag_6(aero.vitesse,aero,etude,param);

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

%% CALCUL DES MASSES

etude.masse.masse_ballons = 0.408233 * etude.nb_ballons ;                          % [lbs] poids des ballons
etude.masse.masse_avion_vide = etude.masse.masse_avion_vide + etude.masse.m_ht + etude.masse.m_vt + etude.masse.m_fuselage ;
etude.masse.masse_totale = etude.payload+etude.masse.masse_ballons + etude.masse.masse_avion_vide ;

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
%***Possibilité d'être intégrée comme paramètre d'entrée.

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
aero.stall_speed = sqrt((etude.masse.masse_totale*9.8 + downlift_elevateur)/ (0.5 * aero.density * aero.wing.s_aile * aero.wing.CL_takeoff_3D))*1.1 ;   % vitesse minimale de controle


%temps rotation
t_rot = 0.4; %sec

dt = 0.01;                     %possibilité d'être augmenté pour améliorer le cpu time
i = 1;

%conditions initiales
time(i)     = 0;    %s
position(i) = 0;    %m
speed(i)    = 0;%15/3.6;    %m/s
thrust(i)   = Thrust_Curve(0, poly_thrust);  %N
lift(i)     = 0;        %N
trainee(i)     = 0;     %N
rolling_resistance(i) = 0.05 * etude.masse.masse_totale;
accelx(i)    = thrust(i) / etude.masse.masse_totale ;
accely(i)    = 0;

while (speed(i) < 1.1*aero.stall_speed) && (position(i)< (30 - speed(i)*t_rot))

    speed(i+1)    = speed(i)    + accelx(i) * dt;
    position(i+1) = position(i) + speed(i) * dt;
    time(i+1,1)   = time(i)     + dt;
    
    thrust(i+1) = Thrust_Curve(speed(i+1), poly_thrust);
    lift(i+1) = 0.5 * aero.density * (speed(i+1))^2 * aero.wing.s_aile * aero.wing.CL_runway_3D ;
    [aero,etude,param] = drag_6(speed(i+1),aero,etude,param);
    trainee(i+1) = aero.drag.drag_total;    
    %trainee_elevateur = ((cd_max_hstab - cd_min_hstab))*q*s_ht;               %trainee supplementaire du h-stab en rotation
    rolling_resistance(i+1) = 0.05 * (etude.masse.masse_totale-lift(i+1)) ;
    accelx(i+1) = (thrust(i+1)- trainee(i+1) - max([0,rolling_resistance(i+1)])) / etude.masse.masse_totale ;
    accely(i+1) = (- 9.8*etude.masse.masse_totale + lift(i+1)) / etude.masse.masse_totale;
    
    i=i+1;
end

%Angle de montée
etude.climb_angle = ((thrust(end) - trainee(end))/ (etude.masse.masse_totale*9.8))*(180/pi);

%Portance totale au décollage
etude.v_takeoff = speed(end) ;
L = lift(end) ;

%% VERIFICATION DES ERREURS POTENTIELLES (SECTION 9)

%teste si la vitesse entrée comme paramètre à la fonction est réaliste
if etude.v_takeoff < 0.8 * aero.vitesse || etude.v_takeoff > 1.3 * aero.vitesse ;         %pour s'assurer que la vitesse supposee est realiste OU faire une boucle !
%     warning('La vitesse d"entree de la fonction est irréaliste, la vitesse de décollage calculée est de %.2f m/s. La vitesse normale est %.2f m/s. \n', etude.v_takeoff,vitesse);
end

%teste si l'avion décolle réellement dans la distance impartie et stocke
%cette distance comme étant la distance nécessaire pour décoller.
decollage = 0;
distance_takeoff=0;
if etude.climb_angle >= 2 && etude.v_takeoff>= 1.1*aero.stall_speed && (position(end)+speed(end)*t_rot <60)
    
    decollage = 'Oui' ;
    distance_takeoff = position(end)+speed(end)*t_rot;
    decollage_distance = sprintf('L''avion decolle apres %.2f m avec un angle de %.1f degres', distance_takeoff, etude.climb_angle);
else
    etude.v_takeoff = 0 ;
    decollage_distance = sprintf('L''avion decolle apres %.2f m avec un angle de %.1f degres', distance_takeoff, etude.climb_angle);
    decollage = 'Non' ;
    etude.climb_angle=0;  %code pour savoir qu'il n'y a pas de décollage
end

%% STOCKAGE DES RESULTATS FINAUX (SECTION 10)

%Ensemble de cellules avec les résultats les plus importants
resultats = {};
resultats{1} = budget ; 
resultats{2} = aero.drag.drag_total ;
resultats{3} = L ;
resultats{4} = etude.masse.masse_totale ;
resultats{5} = etude.v_takeoff ; 
resultats{6} = aero.wing.CL_runway_3D ;
resultats{7} = aero.wing.CL_takeoff_3D ;
resultats{8} = decollage ;
resultats{9} = distance_takeoff ;
resultats{10} = etude.climb_angle ;

%Enregistrement des paramètres
 if aero.wing.aile=='normal'
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{2}=aero.wing.AoA_Cl_max* (360/(2*pi));    %Angle de lift maximal
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{3}=aero.wing.AoA_Cd_min* (360/(2*pi));    %Angle de trainée minimale
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{4}=aero.stall_speed;                           %Vitesse de décrochage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{5}=aero.wing.span_in;                               %Span
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{6}=aero.wing.chord_in;                              %Chord
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{7}=resultats{2};                          %Drag décollage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{8}=resultats{3};                          %Lift décollage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{9}=resultats{4};                           %Masse totale
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{10}=resultats{7};                          %CLmax au décollage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{11}=aero.drag.drag_fuselage;                           %Trainée fuselage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{12}=aero.drag.drag_wing;                           %Trainée forme aile
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{13}=aero.drag.drag_induced;                           %Trainée induite aile
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{14}=decollage;                          %Décollage?
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{15}=etude.climb_angle;                          %Angle décollage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{16}=distance_takeoff;                     %Distance décollage
            score_biplan=[];
 elseif aero.wing.aile=='biplan'
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{2}=aero.wing.AoA_Cl_max* (360/(2*pi));    %Angle de lift maximal
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{3}=aero.wing.AoA_Cd_min* (360/(2*pi));    %Angle de trainée minimale
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{4}=aero.stall_speed;                           %Vitesse de décrochage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{5}=aero.wing.span_in;                               %Span
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{6}=aero.wing.chord_in;                              %Chord
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{7}=resultats{2};                          %Drag décollage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{8}=resultats{3};                          %Lift décollage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{9}=resultats{4};                           %Masse totale
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{10}=resultats{7};                          %CLmax au décollage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{11}=aero.drag.drag_fuse;                           %Trainée fuselage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{12}=aero.drag.drag_wing;                           %Trainée forme aile
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{13}=aero.drag.drag_wing;                           %Trainée induite aile
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{14}=decollage;                          %Décollage?
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{15}=etude.climb_angle;                          %Angle décollage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{16}=distance_takeoff;                     %Distance décollage
            score_normal=[];
 end
        

%affiche tous les résultats importants qui ont été calculés dans la
%fonction analyse.m
% if strcmp(spam,'on')
% 
%     fprintf('\nL''avion décolle-t-il ? Réponse: %s \n', decollage);
%     if strcmpi(decollage,'Oui')
%         disp(decollage_distance);
%     end
%     fprintf('Le span du H-Stab est de %.2f m\n', span_ht)
%     fprintf('La corde du H-Stab est de %.2f m\n', chord_ht)
%     fprintf('La surface du V-Stab est de %.2f m2\n', trainees{10})
%     fprintf('Le span du V-Stab est de %.2f m\n', span_vt)
%     fprintf('La corde du V-Stab est de %.2f m\n', chord_vt)
%     fprintf('L''efficacité du profil est de %.2f\n', rendement_profil)
%     fprintf('L''angle de trainee minimale est de %.2f degrés\n', aero.wing.AoA_Cd_min* (360/(2*pi)))
%     fprintf('L''angle de lift maximal est de %.2f degrés\n', aero.wing.AoA_Cl_max* (360/(2*pi)))
%     fprintf('La vitesse de décrochage est de %.2f m/s\n', aero.stall_speed)
%     fprintf('L''envergure est de %.2f m (%.0f in)\n', aero.wing.span, aero.wing.span_in)
%     fprintf('La corde est de %.2f m (%.1f in)\n', aero.wing.chord, aero.wing.chord_in)
%     fprintf('Le drag total est %.2f N au décollage\n', resultats{2});
%     fprintf('Le lift total est %.2f N au décollage\n', resultats{3});
%     fprintf('La masse totale de l''avion est de %.2f kg (%.0f N)\n', resultats{4}, resultats{4}*9.8);
%     fprintf('La masse de l''avion à vide est de %.2f kg\n', etude.masse.masse_avion_vide);
%     fprintf('La masse du fuselage est de %.2f kg\n', m_fuselage);
%     fprintf('La masse de l''élévateur est de %.2f kg\n', m_ht);
%     fprintf('La masse du rudder est de %.2f kg\n', m_vt);
%     fprintf('La longueur totale de l''avion est de %.2f m\n', etude.geo.longueur_totale);
%     fprintf('Le vitesse est %.2f m/s au décollage\n', resultats{5});
%     fprintf('Le CL sur la piste est de %.2f\n', resultats{6});
%     fprintf('Le CL maximal au décollage est de %.2f\n', resultats{7});
%     fprintf('Trainee fuselage: %.1f\n', trainees{1});
%     fprintf('Trainee de forme aile: %.1f\n', trainees{3});
%     fprintf('Trainee induite aile: %.1f\n', trainees{4});
%     fprintf('Trainee hstab: %.1f\n', trainees{5});
%     fprintf('Trainee vstab: %.1f\n', trainees{6});
%     fprintf('Trainee gear: %.1f\n', trainees{7});
%     disp(budget);
% end
%% GRAPHIQUES DE PERFORMANCE (SECTION 11)

%Sortie graphiques de performances du programme pour une configuration
%donnée.
% if strcmpi(graphiques,'on')  
%     figure1=figure;
%     subplot(2,4,1);
%     plot([0:dt:((length(position)-1)*dt)], position./0.3048);
%     xlabel('Time (s)');
%     ylabel('Position(ft)');
%     grid on
%     subplot(2,4,2);
%     plot([0:dt:((length(speed)-1)*dt)], speed/0.3048);
%     xlabel('Time (s)');
%     ylabel('Speed (ft/s)');
%     axis([0 inf 0 inf])
%     grid on
%     
%     hold on
%     plot([0;10],[aero.stall_speed/0.3048;aero.stall_speed/0.3048]);
%     plot([0;10],[1.1*aero.stall_speed/0.3048;1.1*aero.stall_speed/0.3048]);
%     hold off
%     
%     subplot(2,4,3);
%     plot([0:dt:((length(accelx)-1)*dt)], accelx/0.3048);
%     xlabel('Time (s)');
%     ylabel('Acceleration en x (ft/s^2)');
%     grid on
%     subplot(2,4,4);
%     plot([0:dt:((length(accely)-1)*dt)], accely/0.3048);
%     xlabel('Time (s)');
%     ylabel('Acceleration en y (ft/s^2)');
%     grid on
%     subplot(2,4,5);
%     plot([0:dt:((length(thrust)-1)*dt)], thrust/4.45);
%     xlabel('Time (s)');
%     ylabel('Thrust (lbf)');
%     grid on
%     subplot(2,4,6);
%     plot([0:dt:((length(lift)-1)*dt)], lift/4.45);
%     xlabel('Time (s)');
%     ylabel('Lift (lbf)');
%     grid on
%     subplot(2,4,7);
%     plot([0:dt:((length(trainee)-1)*dt)], trainee/4.45);
%     xlabel('Time (s)');
%     ylabel('Drag (lbf)');
%     grid on
%     subplot(2,4,8);
%     hold on
%     plot([aero.wing.AoA_Cd_min* (360/(2*pi)):0.001* (360/(2*pi)):aero.wing.AoA_Cl_max* (360/(2*pi))], equation_lift_3D);
%     xlabel('Alpha (deg)');
%     ylabel('CL');
%     plot(donnees_profil{2}(:,1),donnees_profil{2}(:,2));
%     xlabel('Alpha (deg)');
%     ylabel('CL');
%     grid on
%     plot(donnees_profil{2}(:,1),donnees_profil{2}(:,3));
%     xlabel('Alpha (deg)');
%     ylabel('CL');
%     grid on
%     hold off
%     %saveas(figure1,['0_',profil,'.jpg']);
%     
%     figure(2)
%     plot([0:dt:((length(thrust)-1)*dt)], thrust/4.45);
%     xlabel('Time (s)');
%     ylabel('Thrust (lbf)');
end

% trainees{14}



