#pragma once

#include <utility>

/**
   @return {pan,tilt}, i.e. les angles de pan et de tilt en degré,
   à appliquer à la caméra axis-ptz pour recentrer un objet
   @param u,v coordonnées de l'objet à recentrer (en pixels)
   @param u0,v0 coordonnées du centre de l'image (en pixels)
   @param pan,tilt angle de pan et de tilt initiaux en degré
   @param zoom paramètre de réglage du zoom sur la caméra axis-ptz (varie entre 1 et 10000)

   Description géométrique : voir pages 27 à 29 dans le poly Traitement des Images
   http://moodle.supelec.fr/moodle/file.php/398/poly/TraitementDesImages.pdf

   @author Jean-Luc Collette
*/
std::pair<double,double> pantiltzoom(double u,   double v,
				     double u0,  double v0,
				     double pan, double tilt, double zoom);
