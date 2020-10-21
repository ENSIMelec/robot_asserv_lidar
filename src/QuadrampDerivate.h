//
// Created by Taoufik on 10/03/2020.
//

#ifndef RAMP_QUADRAMPDERIVATE_H
#define RAMP_QUADRAMPDERIVATE_H

using namespace std;
#include <cmath>
#include "MathUtils.h"
#include "Configuration.h"

class QuadrampDerivate {


public:
    QuadrampDerivate() {};
    QuadrampDerivate(bool is_distance);
    ~QuadrampDerivate();

    /**
     * Renvoie le booléen arrivee qui passe à frais quand la rampe est terminée
     */
    bool isRampFinished() {return m_is_reached; }

    // Filtre la consigne pour tenir compte des rampes d'accélération et de décélération
    float process(float consigne, float position_actuelle , float vitesse);

private:

    /**
     * Vitesse
     */
    float m_speed_max_av = 0;  // avant
    float m_speed_max_arr = 0; // arrière (négatif)

    /**
     * Accélération / Décélération
     */

    float m_acceleration_max_av = 0; //Accélération avant
    float m_deceleration_max_av = 0; //Décélération avant


    float m_acceleration_max_arr = 0; //Accélération arrière
    float m_deceleration_max_arr = 0; //Décélération arrière

    /**
     * Gain anticipation
     */
    float m_gainAnticipation_av = 0; // avant
    float m_gainAnticipation_arr = 0; // arrière


    // Sauvegarde de la vitesse précédente
    int m_prevConsigneVitesse = 0; // Stocke la vitesse de l'itération précédente

    // Taille de la fenêtre dans laquelle on se considère comme arrivé puisque la consigne exacte est généralement inatteignable
    float m_window_reach = 0;

    // Permet de connaitre l'état de la consigne actuelle ( on est arrivée ou non... )
    bool m_is_reached = false;

};


#endif //RAMP_QUADRAMPDERIVATE_H
