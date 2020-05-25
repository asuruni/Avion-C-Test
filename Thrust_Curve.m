function [thrust] = Thrust_Curve(V, poly_thrust)

%**************************************************************************
%-> COMMENTAIRE : 
% - Fonction qui determine le thrust en fonction d'une vitesse donnee en
%   pieds/secondes.

%-> AUTEUR  : Vincent Dubanchet.
%-> VERSION : Novembre 2013.
%-> DERNIÈRE MODIFICATION: William Désilets, 4 juillet 2017
%**************************************************************************

% Calcul du thrust a la vitesse donnee, d'apres la courbe d'interpolation
% POLYNOMIALE obtenue par les essais dynamiques
thrust =  polyval(poly_thrust,V); 

end