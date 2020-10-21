//
// Created by Taoufik on 10/03/2020.
//

#include "QuadrampDerivate.h"
#include "iostream"
using namespace std;

QuadrampDerivate::QuadrampDerivate(bool is_distance) {


    m_prevConsigneVitesse = 0;
    m_is_reached = true;

    if(is_distance) {
        m_speed_max_arr = Configuration::instance().getFloat("distance_speed_max");
        m_speed_max_av = m_speed_max_arr;

        m_acceleration_max_av = Configuration::instance().getFloat("distance_acceleration_max"); //Accélération avant
        m_deceleration_max_av = Configuration::instance().getFloat("distance_deceleration_max"); //Décélération avant


        m_acceleration_max_arr = m_acceleration_max_av; //Accélération arrière
        m_deceleration_max_arr = m_deceleration_max_av; //Décélération arrière

        // Gain anticipation
        // Coeff déterminant le début de la rampe de décélération en rotation
        m_gainAnticipation_av = Configuration::instance().getFloat("distance_anticipation_gain"); // avant
        m_gainAnticipation_arr = m_gainAnticipation_av; // arrière

        // window
        m_window_reach = Configuration::instance().getFloat("distance_window");; //mm
    }
    else {

        m_speed_max_arr = Configuration::instance().getFloat("angle_speed_max");
        m_speed_max_av = m_speed_max_arr;

        m_acceleration_max_av = Configuration::instance().getFloat("angle_acceleration_max"); //Accélération avant
        m_deceleration_max_av = Configuration::instance().getFloat("angle_deceleration_max"); //Décélération avant


        m_acceleration_max_arr = m_acceleration_max_av; //Accélération arrière
        m_deceleration_max_arr = m_deceleration_max_av; //Décélération arrière


        // Gain anticipation
        // Coeff déterminant le début de la rampe de décélération en rotation
        m_gainAnticipation_av = Configuration::instance().getFloat("angle_anticipation_gain"); // avant
        m_gainAnticipation_arr = m_gainAnticipation_av; // arrière

        // window
        m_window_reach = MathUtils::deg2rad(2); // rad
    }

}

/**
 * Filtrer la consigne de distance pour effectuer une ramp, pour ensuite le donner filtre PID
 *
 * @param consigne de distance/angle (point d'arrivé)
 * @param position_actuelle (distance/angle parcouru depuis t0)
 * @param vitesse (distance/angle parcouru en dt)
 * @return
 */
float QuadrampDerivate::process(float consigne_init, float position_actuelle, float vitesse) {

    // Reset du flag "arrivee" signalant que la consigne est atteinte
    m_is_reached = false;

    //Calcul de la position du pivot qui sert à déterminer si l'on doit commencer à décélérer ou non
    int sens = (consigne_init - position_actuelle >= 0) ? 1 : -1;
    float position_pivot;

    if (sens == 1) {
        position_pivot = consigne_init + ((vitesse >= 0) ? -1 : 1) * (((vitesse * vitesse) / (2 * m_deceleration_max_av)) + abs(vitesse) * m_gainAnticipation_av);
    } else {
        position_pivot = consigne_init + ((vitesse >= 0) ? -1 : 1) * (((vitesse * vitesse) / (2 * m_deceleration_max_arr)) + abs(vitesse) * m_gainAnticipation_arr);
    }

    //Calcul de la consigne d'accélération qui dépend dans le sens dans lequel on roule et vient de config.h
    float acceleration_consign;

    if (position_pivot >= position_actuelle) {
        acceleration_consign = (sens == 1) ? m_acceleration_max_av : m_deceleration_max_arr;
    } else {
        acceleration_consign = (sens == 1) ? -m_deceleration_max_av : -m_acceleration_max_arr;
    }

    // Calcul de la consigne de vitesse
    float consigneVitesse = m_prevConsigneVitesse + acceleration_consign;


    // On borne la consigne
    consigneVitesse = MathUtils::constrain(consigneVitesse, -m_speed_max_arr, m_speed_max_av);
    // On stocke la nouvelle consigne pour l'itération suivante
    m_prevConsigneVitesse = consigneVitesse;

    // debug
    /*cout << "======= FILTER QUADRAMP DEBUG =======" << endl;
    cout << "[RAMP] consigne init: " << consigne_init << endl;
    cout << "[RAMP] position_actuelle: " << position_actuelle << endl;
    cout << "[RAMP] sens: " << sens << endl;
    cout << "[RAMP] position_pivot: " << position_pivot << endl;
    cout << "[RAMP] vitesse: " << vitesse << endl;
    cout << "[RAMP] consigneVitesse: " << consigneVitesse << endl;
    cout << "[RAMP] acceleration_consign: " << consigneVitesse << endl;
    cout << "[RAMP] return rampedDistance: " << position_actuelle + consigneVitesse << endl;
    cout << " ================================ " << endl;*/


    // On vérifie si on est dans la fenêtre d'arrivée et si oui, on est arrivé à la fin de la rampe
    if (abs(consigne_init - position_actuelle) < m_window_reach) {
        m_prevConsigneVitesse = 0; // On reset la consigne precedente
        m_is_reached = true;
        return consigne_init;
    }

    //On retourne la consigne de position
    return position_actuelle + consigneVitesse;
}


QuadrampDerivate::~QuadrampDerivate() {

}

