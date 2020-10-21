#ifndef ROBOT_ODOMETRY_H
#define ROBOT_ODOMETRY_H

#include "ICodeurManager.h"
#include "cmath"
#include "iostream"
#include "MathUtils.h"
#include "Configuration.h"
#include "Config.h"

/**
 * @brief Structure de position.
 * \author Taoufik Tribki
 * Position est une structure de Odometry.h qui permet en une variable d'obtenir la totalité des informations à propos du positionnement du robot.
 */
struct Position
{
    /**
     * @brief Constructeur de Position.
     * Constructeur de Position qui initialise la position au coordonnées (0,0) et à l'angle 0.
     */
    Position() : x(0), y(0), theta(0){}

    /**
     * @brief  Constructeur de Position.
     *
     * Constructeur de Position qui initialise la position au coordonnées indiqués.
     *
     * @param x coordoonée en x initial.
     * @param y coordoonée en y initial.
     * @param theta angle initial.
     */
    Position(float vx, float vy, float vtheta) : x(vx), y(vy), theta(vtheta){}
    // Coordonnée en x
    float x;
    // Cordonnée en y
    float y;
    // Angle
    float theta;
};
/**
 * Calcul la position en temps réél du robot
 */
class Odometry {

public:

    explicit Odometry(ICodeurManager &codeurs);
    void debug();
    /*void calcul_position_segment(float distance, float angle);
    void calcul_position_arc(float distance, float angle);*/
    void distance_total_update(int long ticksLeft, int long ticksRight);
    /**
     * @brief Retourne la position
     * @return
     */
    const Position &getPosition() const { return m_pos; }
    void setPosition(float x, float y, float theta){ m_pos.x = x; m_pos.y = y; m_pos.theta = theta;}

    /**
    * @brief Calcule la nouvelle position et la nouvelle vitesse.
    * détermine la nouvelle vitesse instantanée et la nouvelle position.
    */
    void update();


    float getDeltaOrientation() const { return m_dAvgTheta; }
    float getDeltaTheta() const { return m_dTheta; }
    float getTotalTheta() const { return m_totalAngle; }
    float getDeltaDistance() const { return m_dDistance; }
    float getTotalDistance() const { return m_totalDistance; }
    float getLinVel() const {return m_linVel;} /* Vitesse linéaire */
    float getAngVel() const {return m_angVel;} /* Vitesse angulaire */
    int getLastTime() const { return m_lastTime; }
    float getTotalTicksL() const { return m_totalTicksL; };
    float getTotalTicksR() const { return m_totalTicksR; };
    float getEntraxe() const { return ENTRAXE; };
    float getRightVel() const { return m_rightVel; }
    float getLeftVel() const { return m_leftVel; }


protected:
    Position m_pos; /*!< Structure de position de Odometry. */
    float m_linVel = 0; /*!< Vitesse lineaire en mm/s. */
    float m_angVel = 0; /*!< Vitesse angulaire en rad/s.*/
    float m_rightVel = 0; /*! Vitesse roue droite */
    float m_leftVel = 0; /*! Vitesse roue gauche */

    int m_totalTicksL = 0; /* Totat des tics gauche */
    int m_totalTicksR = 0;/* Totat des tics droite */
    float m_totalDistance = 0; /* Total distance parcouru (odometry test purpose) */
    float m_totalAngle = 0;

    float m_dDistance = 0; /*  Distance en mm parcouru en dt */
    float m_dAvgTheta = 0; /* moyenne angle(t-1) et t en rad en dt */
    float m_dTheta = 0; /* angle rad en dt */
    float m_lastTime = 0; /* dt */


    /**
     * Coefficient pour tics -> mm
     */
    float TICK_RIGHT_TO_MM = 0; /* Coefficient Longeur Gauche */
    float TICK_LEFT_TO_MM = 0; /* Coefficient Longeur Droit */

    /**
     * Coefficient pour tics -> rad
     */
    float TICK_RIGHT_TO_RAD = 0; /* Coefficient Angle Gauche */
    float TICK_LEFT_TO_RAD = 0; /* Coefficient Angle Droit */


    // constantes
    float PERIM_ROUE = 0; /* Perimetre roue en mm  Diametre * PI */
    float RESOLUTION = 0; /* nombre tics par 1 tour du roue */
    float COEF_CORRECTEUR = 0;
    float ENTRAXE = 0; // entraxe entre les deux roues en mm

    ICodeurManager& m_codeurs;

};


#endif //ROBOT_ODOMETRY_H
