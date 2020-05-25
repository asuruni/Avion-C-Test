
function [resultats,score_normal,score_biplan] = analyse_9(aero,etude,param,a_r,n_b,p_l,ang,w_s)

%**************************************************************************

%-> COMMENTAIRE : Fonction qui calcule toutes les caract�ristiques
%importantes de l'avion de mani�re pr�liminaire en fonction des param�tres
%entr�s. Les sorties comportent, entre autres, un budget de tra�n�e, la
%distance de d�collage, la vitesse de d�collage, les caract�ristiques
%g�om�triques et a�rodynamiques de l'aile, etc.

%-> AUTEURS  : William D�silets et C�dric Dionne
%-> CR�ATION : Juin 2017.
%-> DERNI�RE MODIFICATION : 6/11/19.
%-> UNIT�S : Syst�me international.

%
%       Modifi� le 6/11/19 par Laurent Emond-Brunet
%       Change notification: Fusion des variables en format struct avec le
%       calcul d'influence du biplan

%**************************************************************************

%Mettre � 'on' si on teste pour une seule configuration. Si la fonction
%analyse.m est utilis�e dans une boucle, s'assurer de laisser ces deux
%variables � 'off'.

%graphiques = 'off';
%spam = 'on';

%% DONNEES DE BASE DU PROFIL (SECTION 1) 
%Section qui permet d'aller chercher toutes les donn�es relatives au profil
%�tudi�. Les donn�es angulaires sont pass�es en radians pour les calculs � suivre.

%variables pour folders
folder_profils = 'profils';         %folder ou sont les donnees des profils
folder_mat = 'mat files';           %folder ou sont les fichiers mat
%cr�e le path pour aller sauvegarder/get les fichier .mat
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

%facteur de s�curit� de 10% sur le lift
aero.wing.Cl_max = donnees_profil{1}(1) * 0.90;
aero.wing.Cl_runway_2D = donnees_profil{1}(8) * 0.90;
aero.wing.Pente_Cl_AoA = donnees_profil{1}(10) * 0.90;
aero.wing.Cl_0AoA = donnees_profil{1}(12) * 0.90;

%pour tenir compte des erreurs de XFLR5, 20% sur la trainee
aero.wing.Cd0_0AOA= donnees_profil{1}(9) *1.2;
aero.wing.Cd_min= donnees_profil{1}(3) *1.2;

%cr�e le path pour aller sauvegarder/get les fichier .mat
path_mat=fullfile(folder_mat,'CmMoyen.mat');
load(path_mat); 
aero.wing.cm_average = cm_average;

%% PRESSION DYNAMIQUE ET CONSTANTES (SECTION 2)
%Initialisation des constantes atmosph�riques, en unit�s du SI. Ces donn�es
%seront r�utilis�es plus tard dans les calculs.

aero.kin_viscosity = 1.46*(10^(-5)) ;
%density = 1.2074 ;
aero.mach = aero.vitesse / 340 ;
aero.q = 0.5 * aero.density * aero.vitesse^2 ;


%% DIMENSIONNEMENT DE L'AVION (SECTION 3)                     
%Bloc permettant de dimensionner l'avion en fonction de la configuration
%choisie et du nombre de balles. Les donn�es g�om�triques de l'aile, du
%fuselage ainsi que les masses pr�liminaires sont d�finies. [� TRAVAILLER. PAYLOAD � INCLURE]

etude.geo.longueur_moteur = 3 * (2.54/100);       %v�rifier si c'est cette valeur
etude.geo.longueur_electro = 3 * (2.54/100);      %v�rifier valeur

%config_balles='tri';
%if config_balles =='tri'
%    longueur_cabine_balles = (3*sqrt(3)*6.5/(2*100))+((6.5 * ((nb_balles-6)/largeur_balles))/100) ; %configuration 1-2-3-4x...
%else
%    longueur_cabine_balles = (6.5 * (nb_balles/largeur_balles)) / 100 ;      % configuration 4x...
%end

%nb_balles_temp=54;
%longueur_cabine_balles = 1.17 %tir� du CAD 19/01/19 %2*(3*sqrt(3)*6.5/(2*100))+((6.5 * ((nb_balles_temp-12)/largeur_balles))/100);    %on pose configuration 54 balles avec triangle en avant et en arri�re

%La longueur du cargo est maintenant d�finie dans MAIN_nouvelles_regles

etude.geo.longueur_totale = etude.geo.longueur_cargo + etude.geo.longueur_moteur + etude.geo.longueur_electro + etude.geo.longueur_queue; %longueur supplementaire ou il n<y a pas de balles

%Calculs des propri�t�s importantes de l'aile, en SI et en pouces.
if aero.wing.aile=='normal'
    if etude.nb_ballons~=0
        aero.wing.span = sqrt(aero.wing.aspect_ratio*(aero.wing.s_aile+(etude.geo.largeur_ballons*22.6*10^-2*24.89*2.54*10^-2)));
    else
        aero.wing.span = sqrt(aero.wing.aspect_ratio*(aero.wing.s_aile+(etude.geo.largeur_payload*22.6*10^-2*24.89*2.54*10^-2))); %Ajout 2019-2020 au cas o� aucun ballon
    end
elseif aero.wing.aile=='biplan'
    if etude.nb_ballons~=0
        aero.wing.span = sqrt(aero.wing.aspect_ratio*(aero.wing.s_aile+(etude.geo.largeur_ballons*22.6*10^-2*24.89*2.54*10^-2)))/2;
    else
        aero.wing.span = sqrt(aero.wing.aspect_ratio*(aero.wing.s_aile+(etude.geo.largeur_payload*22.6*10^-2*24.89*2.54*10^-2))/2); %Ajout 2019-2020 au cas o� aucun ballon
    end
end

aero.wing.span_in = aero.wing.span*100 / 2.54;

if aero.wing.aile=='normal'
    aero.wing.chord = aero.wing.s_aile / aero.wing.span;
elseif aero.wing.aile=='biplan'
    aero.wing.chord = (aero.wing.s_aile/2) / aero.wing.span;
end
aero.wing.chord_in = aero.wing.chord*100 /2.54;


%% �TABLISSEMENT DU LIFT 3D (SECTION 4)
%Approximation des coefficients de portance 3D � partir des donn�es 2D et
%des caract�ristiques de l'aile.

rendement_profil = aero.wing.Pente_Cl_AoA / (2*pi);                                                                     %pente_cl_alpha / (2*pi)
pente_CL_3D = (2*pi*aero.wing.aspect_ratio) / (2 + sqrt(4+(((aero.wing.aspect_ratio)^2 * (1-aero.mach^2)^2)/(rendement_profil)^2))) ;           %bas� sur Raymer p.412

%Angle d'attaque 0 lift 2D
alpha_zero = aero.wing.ZeroLiftAoA ;

%Angle 'Attaque au Cd minimum 2D
alpha = aero.wing.AoA_Cd_min;

%Boucle qui permet de cr�er une courbe de points du coefficient de portance
%3D. La courbe est d�finie jusqu'� l'angle de CL_max 2D.
i = 1;
equation_lift_3D=zeros(1,(floor((alpha-alpha_zero)*1000)));
while alpha < aero.wing.AoA_Cl_max 
    equation_lift_3D(i) = (alpha-(-aero.wing.Cl_0AoA/aero.wing.Pente_Cl_AoA))*pente_CL_3D ;   %LE +500 EST POUR �LIMINER LES INDEXATIONS � UNE POSITION N�GATIVE
    alpha = alpha + 0.001;
    i = i+1;
end

if aero.Alpha_Runway*2*pi/360<aero.wing.AoA_Cd_min
    error('Angle d''incidence trop petit')
end

%�tablissements des CL 3D importants � partir des valeurs 2D entr�es en param�tres dans la fonction
aero.wing.CL_runway_3D = equation_lift_3D(round(((aero.Alpha_Runway*2*pi/360)-aero.wing.AoA_Cd_min)/0.001)) ;      %CL � Alpha_Runway
aero.wing.CL_takeoff_3D = equation_lift_3D(end) ;    


%% CALCUL DE TRAINEE TOTALE ET DIMENSIONNEMENT STABS (SECTION 5)
%Bloc permettant de calculer et de stocker toutes les donn�es relatives �
%la tra�n�e de l'avion. Sort aussi les caract�ristiques dimensionnelles des
%surfaces des stabilisateurs.

%Fonction qui s'occupe de calculer les tra�n�es. Voir fonction drag.m pour
%pr�cisions suppl�mentaires.
[aero,etude,param] = drag_6(aero.vitesse,aero,etude,param);

%Dimensions du stabilisateur horizontal
% s_ht = 0.275;     %s_ht = drags{9};
% m_ht = 0.672/2.2;    %masse mesur�e sur vrai avion %m_ht = drags{11};
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

%L�gende des sorties de la fonction drag_3.m pour aider � s'y retrouver !
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
%Bloc tr�s simple qui attribue un pourcentage de la train�e totale de
%l'avion � chaque source de train�e. Le r�sultat est un "budget de
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
%Integration de la courbe de thrust obtenue exp�rimentalement au programme.
%***Possibilit� d'�tre int�gr�e comme param�tre d'entr�e.

% Polynome d'interpolation du thrust en fonction de la vitesse
% - Courbe basee sur les r�sultats des tests moteurs de 2013 avec l'helice 
%   {} ==> courbe parabolique obtenue
poly_thrust       = etude.thrust*4.44822;%[ 0.0005   -0.0203    0.0550   10.7915 ]*4.44822; %N / m/s

%% LIFT TOTAL ET VITESSE EN BOUT DE PISTE (SECTION 8)

%COPI� (tres fortement inspir�) DE LA FONCTION Analyse_Decollage.m; le 21
%juin 2017.


%valeurs importantes pour le decollage
downlift_elevateur = aero.q * 0.7 * aero.tail.s_ht;                        %cl du H-Stab estim� � 0.7
cd_max_hstab = 0.025;                                       %fix� approximativement
aero.stall_speed = sqrt((etude.masse.masse_totale*9.8 + downlift_elevateur)/ (0.5 * aero.density * aero.wing.s_aile * aero.wing.CL_takeoff_3D))*1.1 ;   % vitesse minimale de controle


%temps rotation
t_rot = 0.4; %sec

dt = 0.01;                     %possibilit� d'�tre augment� pour am�liorer le cpu time
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

%Angle de mont�e
etude.climb_angle = ((thrust(end) - trainee(end))/ (etude.masse.masse_totale*9.8))*(180/pi);

%Portance totale au d�collage
etude.v_takeoff = speed(end) ;
L = lift(end) ;

%% VERIFICATION DES ERREURS POTENTIELLES (SECTION 9)

%teste si la vitesse entr�e comme param�tre � la fonction est r�aliste
if etude.v_takeoff < 0.8 * aero.vitesse || etude.v_takeoff > 1.3 * aero.vitesse ;         %pour s'assurer que la vitesse supposee est realiste OU faire une boucle !
%     warning('La vitesse d"entree de la fonction est irr�aliste, la vitesse de d�collage calcul�e est de %.2f m/s. La vitesse normale est %.2f m/s. \n', etude.v_takeoff,vitesse);
end

%teste si l'avion d�colle r�ellement dans la distance impartie et stocke
%cette distance comme �tant la distance n�cessaire pour d�coller.
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
    etude.climb_angle=0;  %code pour savoir qu'il n'y a pas de d�collage
end

%% STOCKAGE DES RESULTATS FINAUX (SECTION 10)

%Ensemble de cellules avec les r�sultats les plus importants
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

%Enregistrement des param�tres
 if aero.wing.aile=='normal'
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{2}=aero.wing.AoA_Cl_max* (360/(2*pi));    %Angle de lift maximal
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{3}=aero.wing.AoA_Cd_min* (360/(2*pi));    %Angle de train�e minimale
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{4}=aero.stall_speed;                           %Vitesse de d�crochage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{5}=aero.wing.span_in;                               %Span
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{6}=aero.wing.chord_in;                              %Chord
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{7}=resultats{2};                          %Drag d�collage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{8}=resultats{3};                          %Lift d�collage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{9}=resultats{4};                           %Masse totale
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{10}=resultats{7};                          %CLmax au d�collage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{11}=aero.drag.drag_fuselage;                           %Train�e fuselage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{12}=aero.drag.drag_wing;                           %Train�e forme aile
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{13}=aero.drag.drag_induced;                           %Train�e induite aile
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{14}=decollage;                          %D�collage?
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{15}=etude.climb_angle;                          %Angle d�collage
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{16}=distance_takeoff;                     %Distance d�collage
            score_biplan=[];
 elseif aero.wing.aile=='biplan'
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{2}=aero.wing.AoA_Cl_max* (360/(2*pi));    %Angle de lift maximal
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{3}=aero.wing.AoA_Cd_min* (360/(2*pi));    %Angle de train�e minimale
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{4}=aero.stall_speed;                           %Vitesse de d�crochage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{5}=aero.wing.span_in;                               %Span
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{6}=aero.wing.chord_in;                              %Chord
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{7}=resultats{2};                          %Drag d�collage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{8}=resultats{3};                          %Lift d�collage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{9}=resultats{4};                           %Masse totale
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{10}=resultats{7};                          %CLmax au d�collage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{11}=aero.drag.drag_fuse;                           %Train�e fuselage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{12}=aero.drag.drag_wing;                           %Train�e forme aile
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{13}=aero.drag.drag_wing;                           %Train�e induite aile
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{14}=decollage;                          %D�collage?
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{15}=etude.climb_angle;                          %Angle d�collage
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{16}=distance_takeoff;                     %Distance d�collage
            score_normal=[];
 end
        

%affiche tous les r�sultats importants qui ont �t� calcul�s dans la
%fonction analyse.m
% if strcmp(spam,'on')
% 
%     fprintf('\nL''avion d�colle-t-il ? R�ponse: %s \n', decollage);
%     if strcmpi(decollage,'Oui')
%         disp(decollage_distance);
%     end
%     fprintf('Le span du H-Stab est de %.2f m\n', span_ht)
%     fprintf('La corde du H-Stab est de %.2f m\n', chord_ht)
%     fprintf('La surface du V-Stab est de %.2f m2\n', trainees{10})
%     fprintf('Le span du V-Stab est de %.2f m\n', span_vt)
%     fprintf('La corde du V-Stab est de %.2f m\n', chord_vt)
%     fprintf('L''efficacit� du profil est de %.2f\n', rendement_profil)
%     fprintf('L''angle de trainee minimale est de %.2f degr�s\n', aero.wing.AoA_Cd_min* (360/(2*pi)))
%     fprintf('L''angle de lift maximal est de %.2f degr�s\n', aero.wing.AoA_Cl_max* (360/(2*pi)))
%     fprintf('La vitesse de d�crochage est de %.2f m/s\n', aero.stall_speed)
%     fprintf('L''envergure est de %.2f m (%.0f in)\n', aero.wing.span, aero.wing.span_in)
%     fprintf('La corde est de %.2f m (%.1f in)\n', aero.wing.chord, aero.wing.chord_in)
%     fprintf('Le drag total est %.2f N au d�collage\n', resultats{2});
%     fprintf('Le lift total est %.2f N au d�collage\n', resultats{3});
%     fprintf('La masse totale de l''avion est de %.2f kg (%.0f N)\n', resultats{4}, resultats{4}*9.8);
%     fprintf('La masse de l''avion � vide est de %.2f kg\n', etude.masse.masse_avion_vide);
%     fprintf('La masse du fuselage est de %.2f kg\n', m_fuselage);
%     fprintf('La masse de l''�l�vateur est de %.2f kg\n', m_ht);
%     fprintf('La masse du rudder est de %.2f kg\n', m_vt);
%     fprintf('La longueur totale de l''avion est de %.2f m\n', etude.geo.longueur_totale);
%     fprintf('Le vitesse est %.2f m/s au d�collage\n', resultats{5});
%     fprintf('Le CL sur la piste est de %.2f\n', resultats{6});
%     fprintf('Le CL maximal au d�collage est de %.2f\n', resultats{7});
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
%donn�e.
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



