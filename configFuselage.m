
function [A_rectangle,A_ellipse,F_rectangle,F_ellipse,longueur,longueur_fairing,longueur_nose_cone,P]=configFuselage(nombreBalles,angle) 

%Fonction qui permet de calculer les valeur de la surface frontale de 
%l'avion et de l'aire du fuselage
%function[]=configFusegale(angle,balles) 
%***********************************************************
%Auteurs: Cristiana Deac et Camila Torres
%geoètres: Nombre de balles et angle
%Valeurs de retour: Aire frontale, aire du fuselage
%Date: 20-09-2019
%******************************************************

Diametre= 9;
 %Transformation de l'angle en radians
 angle=(angle*pi)/180;
 %calcul de la hauteur du fuselage
        if nombreBalles == 1 || nombreBalles==2
            hauteur=11;
        else
            HG=9*sin(angle);                                 %Hauteur entre les deux centres de masse (balles supérieur et inf)
            hauteur=11+HG;                                  %Hauteur du fuselage avec un jeu de 1 po en haut et un en bas
        end
    
    %Calcul de la distance entre les balles et de la longueur totale du fuselage
        if nombreBalles == 1
            PG=0;
            P=0;
            longueur=11;
        elseif nombreBalles == 2
            PG=0;
            P=0;
            longueur=20;
        elseif rem(nombreBalles,2)==0
            PG=18*cos(angle);                           %Distance horizaontale entre les centres de masse de deux balles
            P=PG-9;                                    %Distance libre entre chaque extrémité de deux balles
            longueur=PG*((nombreBalles-1)/2)+9+PG/2;
        elseif rem(nombreBalles,2)~=0
            PG=18*cos(angle);                           %Distance horizontale entre les centres de masse de deux balles
            P=PG-9;                                    %Distance libre entre chaque extrémité de deux balles
            longueur=PG*((nombreBalles-1)/2)+9;
        end
    %Ajout nose cone et fairing arrière REVOIR
    longueur_fairing = 15;
    longueur_nose_cone = 10;
    longueur = longueur+longueur_fairing+longueur_nose_cone;
    
    %Calcul de l'aire de la surface frontale
    A_rectangle= 11*hauteur;
    A_ellipse= (hauteur/2)*((Diametre+2)/2)*pi;
    %Calcul de l'aire extérieure du fuselage
    F_rectangle= (2*11*longueur)+(2*hauteur*longueur);
    F_ellipse= (2*pi*(((hauteur/2)^2+((Diametre+2)/2)^2)/2)^(1/2))*longueur;
    