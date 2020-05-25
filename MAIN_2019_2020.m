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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
clc;

etude.score.score_normal=[];
etude.score.score_biplan=[];

%% initialisation variables de départ
aero.wing.profil = 'E423';
aero.vitesse = 12; %m/s
%aero.wing.s_aile = (19.3*144)*(2.54^2)/(100^2);%1.78; %m^2       ligne enlevée car
%variable selon largeur du fuselage

%Itération de l'aspect ratio

%Définition de la boucle d'itération (fixer à la même valeur si non itéré)
iter.a_r.initial=2;
iter.a_r.delta=1;
iter.a_r.final=2;
iter.para.a_r = 1;

for a_r = iter.a_r.initial:iter.a_r.delta:iter.a_r.final
    fprintf('Aspect_ratio_%g \n', a_r); 
aero.wing.aspect_ratio = 7+(a_r-5);

%Choix biplan (ajouté le 17-09-2019)
aero.wing.aile = 'biplan'; %normal ou biplan

        if strcmpi(aero.wing.aile,'biplan')
            aero.wing.gap = 0.6;                                %espace entre les deux ailes (mètres)
            aero.wing.stagger = 0;                             %Angle du décallage entre les deux ailes (rad)
        else
            aero.wing.gap = 0;
            aero.wing.stagger = 0;
        end

aero.tail.tail_type = 'normal';
etude.geo.hauteur_gear = ((4.5+0.5) * 2.54)/100; %m   %provient de Présentation DDR+ 0.5" de jeu
etude.geo.profil_fuselage = 'panneaux';       %utilisé pour calcul de drag du fuselage
etude.geo.arrondi_fuselage = 'demi';          %utilisé pour le calcul de drag du fuselage
aero.tail.cd_min_hstab = 0.012;               
aero.tail.cd_min_vstab = 0.008;
aero.Alpha_Runway = 0; %EN DEGRÉS

aero.wing.oswald_efficiency = 1.78*(1-0.045*(aero.wing.aspect_ratio^0.68))-0.64;    %Formule semi empirique, RAYMER 5eme p.456 -- n'agit plus comme un input modifiable
etude.geo.aire_gear = (2*etude.geo.hauteur_gear + (7*2.54)/100)*0.01; %m^2

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

%% Aller chercher les fichiers des profils d'aile
%variables pour folders
folder_profils = 'profils';         %folder ou sont les donnees des profils
folder_mat = 'mat files';           %folder ou sont les fichiers mat
%crée le path pour aller sauvegarder/get les fichier .mat
path_mat=fullfile(folder_mat,'info.mat');
load(path_mat);

tableau_resultats={};

%% Paramètres de boucle (quand il y a boucle)

        %Itération du nombre de ballons

        %Définition de la boucle d'itération (fixer à la même valeur si non itéré)
        iter.ballon.initial=9;
        iter.ballon.delta=1;
        iter.ballon.final=9;
        
        for n_b=iter.ballon.initial:iter.ballon.delta:iter.ballon.final
        etude.nb_ballons=n_b;
        %Itération du payload

        %Définition de la boucle d'itération (fixer à la même valeur si non itéré)
        iter.payload.initial=0;        
        iter.payload.delta=1;         
        iter.payload.final=5;           
        
        %Définir le payload à part et itérer?
        for p_l=iter.payload.initial:iter.payload.delta:iter.payload.final
        etude.payload=p_l-1;      %[lbs]
        etude.geo.longueur_queue=(27*2.54/100) ;  %posée comme conditions posées
        
        %Itération du wingspan

        %Définition de la boucle d'itération (fixer à la même valeur si non itéré)
        iter.wingspan.initial=7;        
        iter.wingspan.delta=1;         
        iter.wingspan.final=7;
        
        %Définition winspan (itérer?)
        for iter_para_w_s=iter.wingspan.initial:iter.wingspan.delta:iter.wingspan.final
        aero.wing.b=w_s*10+20; %[in] 
        
        %Itération de l'angle entre les ballons

        %Définition de la boucle d'itération (fixer à la même valeur si non itéré)
        iter.angle.initial=7;        
        iter.angle.delta=1;         
        iter.angle.final=7;
        
        %Définition de l'angle
        for iter_para_ang = iter.angle.initial:iter.angle.delta:iter.angle.final
        etude.geo.angle = (ang-1)*10; %[deg]

        %Calcul de la forme du fuselage en fonction de la position des
        %ballons (design 2019-2020)
        [A_rectangle,A_ellipse,F_rectangle,F_ellipse,longueur_totale,longueur_fairing,longueur_nose_cone,P]=configFuselage(etude.nb_ballons,etude.geo.angle); %Dimensions en pouces [in^2]
        
        %Conversion en métrique
        aero.fuselage.s_wet = F_ellipse*0.00064516;    %[m^2]
        aero.fuselage.s_frontale= A_ellipse*0.00064516;    %[m^2]
        aero.fuselage.aire_totale = aero.fuselage.s_wet+aero.fuselage.s_frontale;
        aero.fuselage.longueur_totale = longueur_totale*0.0254;  %[m]
        aero.fuselage.longueur_cargo = longueur_totale-(longueur_fairing+longueur_nose_cone);
            
        s_aile = (((b*2.54/100)^2)/aspect_ratio); 
        
        if aero.wing.aile=='biplan'       %Design unmanned aircraft systems p.125 Semble trop idéal, néglige intéraction entre deux ailes
            aero.wing.s_aile=2*aero.wing.s_aile;
        end
        
        
        %% CALCUL DE LA MASSE A VIDE

            % Masses surfaciques 
            param.masse.m_surf_aile          = 0.000752 * 703.08;               % kg/m^2   Masse de l'aile par unite de surface    %basé sur ancien Matlab
            param.masse.m_lin_longeron       = 0.019 * 17.858;                 % kg/m     Masse du Longeron par unite de longueur     %basé sur valeur réelle 2018-2019
            param.masse.m_lin_tail           = (362/1000)/(28*0.0254)*0.75; %ancienne valeur avec diminution de 75% visée          % kg/m     Masse de la queue par unite de longueur %basé sur estimation 2018-2019    %basé sur ancien matlab: (0.01 * 17.858; )
            param.masse.m_lin_landing        =(1.1*0.453592)/(6.74*0.0254) ;     % kg/m     Masse du train d'atterissage en fonction de la hauteur  %basé sur Regular 2017-2018
            % Masses des equipements
            etude.masse.masse_moteur      = 1.30 / 2.2;             % kg  %masse réelle moteur+hélice          Masse du moteur +hélice (0.8 lb pour S 4025, 1.38 lb pour HK 5025, 0.8377 SII-4035-380kv, 0.266lb hélice))
            etude.masse.masse_batterie    = (0.921 + 0.251) / 2.2;          % kg           Masse de la batterie + speed controler
            etude.masse.masse_noseAssy    = 0 / 2.2;                  % kg           Masse du nose avec le engine mount
            etude.masse.masse_servo       = 0.4 / 2.2;%0.275;           % kg          Masse de l'electronique (cf. fichier Bilan_Masses_Red_Eagle)
            etude.masse.masse_autre       = (0.6 / 2.2)+ 0.5/2.2;       % kg  %masse de 0.5kg fixation aile         Masse monokote + masse fixation aile (mesure 19/01/19 Département structure)
            etude.masse.masse_equipement = etude.masse.masse_moteur + etude.masse.masse_batterie + etude.masse.masse_noseAssy + etude.masse.masse_servo + etude.masse.masse_autre;

            %masses 
            %masse_aile = 2.4; %donnees reelles 19/01/19 
            etude.masse.masse_aile = param.masse.m_surf_aile * aero.wing.s_aile;
            %masse_longeron =0; 
            etude.masse.masse_longeron = param.masse.m_lin_longeron * (96*2.54/100);    %longeron est 8" de long
            %masse_queue = 0.723/2.2; %mesure masse finale sur avion 2018-2019
            etude.masse.masse_queue = param.masse.m_lin_tail * etude.geo.longueur_queue;
            etude.masse.masse_gear = 0.802/2.2; %prise mesure avion final 2018-2019

            etude.masse.masse_avion_vide = etude.masse.masse_aile + etude.masse.masse_longeron + etude.masse.masse_queue + etude.masse.masse_gear + etude.masse.masse_equipement-0.5; %kg  %manque masses du stabh, stab v et du fuselage (normal)

            
%% Performances
        
        
        [resultats,etude.score.score_normal,etude.score.score_biplan] = analyse_9(aero,etude,param);
        
        %Pointage
        if aero.wing.aile=='normal'
            score_normal{a_r,n_b}{p_l,w_s}{ang,1}{1}=120*(2*etude.nb_ballons+etude.payload)/(aero.wing.b+etude.geo.longueur_cargo*100/2.54);
        elseif aero.wing.aile=='biplan'
            score_biplan{a_r,n_b}{p_l,w_s}{ang,1}{1}=120*(2*etude.nb_ballons+etude.payload)/(aero.wing.b+etude.geo.longueur_cargo*100/2.54);
        end
        
%         fprintf('Le pointage est %g points\n',score);

        end
        end
        end
        end
end


%% Sauvegarde des résultats dans le fichier score_...mat

if aero.wing.aile=='normal'
    save('score_normal.mat','score_normal');
elseif aero.wing.aile=='biplan'
    save('score_biplan.mat','score_biplan');
end