#include "Odometry.h"

using namespace std;

/**
 * Odométrie
 * @param codeurs
 */

Odometry::Odometry(ICodeurManager &codeurs) : m_codeurs(codeurs){


    // nouvelle approche odométrie
    //this->PERIM_ROUE = 31.5*M_PI;
    //this->PERIM_ROUE = 34.8*M_PI; //Diametre * PI
//
//    this->PERIM_ROUE = 31.5*M_PI; //Diametre * PI
//    this->RESOLUTION = 516;
//    this->COEF_CORRECTEUR = m_config.getCoeffCorrecteur();
//    this->ENTRAXE = 290;

    //this->ENTRAXE = 280;

    float diametre = Configuration::instance().getFloat("diametre_roue");
    this->PERIM_ROUE = diametre*M_PI; //Diametre * PI
    this->RESOLUTION = Configuration::instance().getFloat("resolution");
    this->COEF_CORRECTEUR = Configuration::instance().getFloat("coeff_correcteur");
    this->ENTRAXE = Configuration::instance().getFloat("entraxe");
    
    // init
    this->m_pos.theta = 0;

}

/**
* @brief Calcule la nouvelle position et la nouvelle vitesse.
* détermine la nouvelle vitesse instantanée et la nouvelle position par approximation de segment de droite
 * Approximation linéaire
 */
void Odometry::update() {


    // récupérer les tics des codeurs + réinitialisation
    m_codeurs.readAndReset();

    // Récupéreration des tics codeurs
    long int ticksLeft = m_codeurs.getLeftTicks();
    long int ticksRight = m_codeurs.getRightTicks();

    // calculer la distance de effectué par chaque roue
    float distanceLeft = ticksLeft * (PERIM_ROUE/RESOLUTION);
    float distanceRight = ticksRight * ((COEF_CORRECTEUR*PERIM_ROUE)/RESOLUTION);


    // Calculer les variations de position en distance et en angle

    // distance parcourue depuis la position de départ jusqu’à l’instant présent.
    float dDistance = (distanceLeft + distanceRight) / 2;
    // Calcul de la différence du nombre de tic entre chaque roue (appx. gauss)
    float diffCount = distanceRight - distanceLeft;
    float dAngle = diffCount/(ENTRAXE);



    // nouvelle méthode:

//    if (diffCount==0)    // On considère le mouvement comme une ligne droite
//    {
//        // Mise à jour de la position
//        this->m_pos.x    += dDistance * cos(this->m_pos.theta);
//        this->m_pos.y    += dDistance * sin(this->m_pos.theta);
//    }
//    else
//    {
//        //On approxime en considérant que le robot suit un arc de cercle
//        // On calcule le rayon de courbure du cercle
//        double R = dDistance / dAngle;
//
//        //Mise à jour de la position
//        this->m_pos.x    += R * (-sin(this->m_pos.theta) + sin(this->m_pos.theta + dAngle));
//        this->m_pos.y    += R * (cos(this->m_pos.theta) - cos(this->m_pos.theta + dAngle));
//        // Mise à jour du cap
//        this->m_pos.theta += dAngle;
//
//        // On limite le cap à +/- PI afin de ne pouvoir tourner dans les deux sens et pas dans un seul
//        if (this->m_pos.theta > M_PI)
//            this->m_pos.theta -= 2 * M_PI ;
//        else if (this->m_pos.theta <= -M_PI)
//            this->m_pos.theta += 2 * M_PI ;
//    }




    // <!> m_pos.theta l'angle initiale
    // Moyenne des angles pour connaître le cap exact
    float avgTheta = m_pos.theta + dAngle/2;

    //Mise à jour de la position du robot en xy et en angle

    this->m_pos.x       += dDistance * cosf(avgTheta); // dAngle?
    this->m_pos.y       += dDistance * sinf(avgTheta);
    this->m_pos.theta   += dAngle;

     if (this->m_pos.theta > M_PI)
		this->m_pos.theta -= 2 * M_PI ;
     else if (this->m_pos.theta <= -M_PI)
		this->m_pos.theta += 2 * M_PI ;

    // Calcul de la vitesse angulaire et linéaire
    // Actualisation du temps
    this->m_lastTime = m_codeurs.getTime();

    float timestep      = MathUtils::millis2sec(m_lastTime); // micros -> s
    //float timestep      = 0.01; // micros -> s

    float linVel        = 0; // mm / s
    float angVel        = 0; // rad / s

    if(timestep > 0) {
        linVel = dDistance / timestep;
        angVel = dAngle / timestep;

        this->m_leftVel = distanceLeft / timestep;
        this->m_rightVel = distanceRight / timestep;
    }

    // Actualisation de la vitesse linéaire et angulaire du robot
    this->m_linVel = linVel;
    this->m_angVel = angVel;

    // Sauvegarde distance et angle actuelle
    m_dDistance = dDistance;
    m_dTheta = dAngle;

    // Actualisation du total distance parcouru
    distance_total_update(ticksLeft, ticksRight);
}

/**
 * @brief Debug purpose
 */
void Odometry::debug() {
    cout << "===========DEBUG ODOMETRY============" << endl;
    cout << "[DATA CODEUR][TICS] : Gauche:" << m_codeurs.getLeftTicks() << " Droit: " << m_codeurs.getRightTicks() << endl;
    cout << "[DATA CODEUR][TOTAL TICS] : Gauche:" << getTotalTicksL() << " Droit: " << getTotalTicksR() << endl;
    cout << "[DATA CODEUR][LAST TIME] : " << getLastTime() << " (ms)" << endl;
    cout << "[ODOMETRY][POSITION] : X:" << getPosition().x << " Y: " << getPosition().y << " Theta: " <<  MathUtils::rad2deg(getPosition().theta) << " °" << endl;
    cout << "[ODOMETRY][DISTANCE PARCOURU EN LASTTIME (mm)] : " << getDeltaDistance() << endl;
    cout << "[ODOMETRY][ROTATION EFFECTUE EN LASTTIME (rad)] : " << getDeltaOrientation() << endl;
    cout << "[ODOMETRY][VITESSE]: Vitesse angulaire (rad/s) : " << getAngVel() << " Vitesse Linéaire (mm/s) : " << getLinVel() << endl;
    cout << "[ODOMETRY][VITESSE]: Roue gauche (mm/s) : " << getLeftVel() << endl;
    cout << "[ODOMETRY][VITESSE]: Roue droite (mm/s) : " << getLeftVel() << endl;
    cout << "[ODOMETRY][TOTAL DISTANCE] (cm): " << getTotalDistance() / 10 << endl;
    cout << "=======================" << endl;

}

void Odometry::distance_total_update(int long ticksLeft, int long ticksRight) {

    m_totalTicksL += ticksLeft;
    m_totalTicksR += ticksRight;

    m_totalDistance += m_dDistance;
    m_totalAngle += m_dTheta;
}