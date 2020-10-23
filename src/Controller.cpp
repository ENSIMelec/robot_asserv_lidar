//
// Created by Taoufik on 12/11/2019.
//

/**
 * TODO:
 *  -Asservissement vitesse, ramp
 *  -Gestion du point d'arrivé (remove correction angle when we are in windowed point)
 *  - Add trajectory goto distance+angle
 *  -Création de plusieurs points (stratégie)
 *  -Rajouter des coefs pour corriger les moteurs en ligne droite sans asservissement angle
 *  -Création du module pour détection lidar
 *  -Lidar path planning
 *
 */

#include "Controller.h"

using namespace std;

Controller::Controller(ICodeurManager& codeurs, MoteurManager& motor, Config& config):
        m_odometry(codeurs), m_motor(motor), m_config(config) {
    // Init PID Controllers

    this->m_maxPWM = 150;
    this->m_maxPWMR = 35;


    // Polar position PID
    this->m_distancePID = PID(
            m_config.getPIDkpDep(),
            m_config.getPIDkiDep(),
            m_config.getPIDkdDep(),
            -m_maxPWM,
            m_maxPWM
    );
    this->m_anglePID = PID(
            m_config.getPIDkpA(),
            m_config.getPIDkiA(),
            m_config.getPIDkdA(),
            -m_maxPWMR,
            m_maxPWMR
    );


    this->m_rampfilterDistance = QuadrampDerivate(true);
    this->m_rampfilterAngle = QuadrampDerivate(false);

    // Enable Control System
    this->m_controlDistance = true;
    this->m_controlAngle = true;
}
/**
 * CS Loop
 */
void Controller::update()
{
    //synchronisation à une fréquence régulière!!

    //Mise à jour de la position/orientation et de la vitesse du robot (Odométrie) (via les roues codeuses)
    m_odometry.update();
    m_odometry.debug();

    // update des consignes en fonction du trajectoire choisi

    switch (m_trajectory) {

        case Point::Trajectory::THETA:
            trajectory_theta(m_targetPos.theta);
            break;
        case Point::Trajectory::XY_ABSOLU:
            trajectory_xy(m_targetPos.x, m_targetPos.y);
            break;
        case Point::Trajectory::LOCKED:
            trajectory_stop();
            break;

        case Point::Trajectory::CALIB_X:
            trajectory_calibration_x();
            break;
        case Point::Trajectory::CALIB_Y:
            trajectory_calibration_y();
            break;
        case Point::Trajectory::CALIB_XY:
            trajectory_calibration_xy();
            break;

        case Point::Trajectory::NOTHING:
            //il n'y a plus rien a faire
            break;

    }

    cout << "[CONSIGNE] TARGET ANGLE (°): " << MathUtils::rad2deg(m_consign.angle) << endl;
    cout << "[CONSIGNE] TARGET DISTANCE (mm) : " << m_consign.distance << endl;
    cout << "[CONSIGNE] DIRECTION: " << m_direction << endl;


    // envoye des consignes au PID
    update_speed(m_consign.distance, m_consign.angle);

    // gestion d'arrivé
    if(is_target_reached()) {
        //trajectory_stop();
        //motors_stop();
        cout << " ----- REACHED target --------- " << endl;
    }

}

/**
 * Effectuer un trajectoire en XY
 * update angle and/or distance
 * @param x_voulu en mm
 * @param y_voulu en mm
 */
void Controller::trajectory_xy(float x_voulu, float y_voulu) {

    // récupérer la position actuelle du robot (odométrie)
    Position deltaPos = m_odometry.getPosition();

    // calcul des erreurs ( prendre la position du robot comme zero)
    float x_diff = (x_voulu - deltaPos.x);
    float y_diff = (y_voulu - deltaPos.y);

    // coordonnées cart -> polaire
    // distance entre la position du robot à instant t, et son objectif
    m_consign.distance = sqrt(x_diff * x_diff + y_diff * y_diff);

    // orientation qui doit prendre le robot pour atteindre le point
    m_consign.angle = atan2f(y_diff, x_diff) - deltaPos.theta;

    // Borner la consigne Angle entre [-pi, pi]
    m_consign.angle = MathUtils::inrange(m_consign.angle, -M_PI, M_PI);

    // Direction (cap inférieur à -pi/2 et supérieur à pi/2)
    //gestion de la marche arrière si on dépasse point de consigne
    // TODO: à tester ( cas < -90  pas nécessaire)
    if (m_consign.angle > M_PI_2)
    {
        m_direction = Direction::BACKWARD;
        m_consign.angle -= M_PI;
        m_consign.distance = -m_consign.distance;
    }
    else if (m_consign.angle < -M_PI_2)
    {
        m_direction = Direction::BACKWARD;
        m_consign.angle += M_PI;
        m_consign.distance = -m_consign.distance;
    }
    else {
        m_direction = Direction::FORWARD;
    }
    // Borner l'angle [-pi, pi]
    m_consign.angle = MathUtils::inrange(m_consign.angle, -M_PI, M_PI);

    cout << "[CONSIGNE] X_DIFF = " << x_diff << " | Y_DIFF = " << y_diff  << endl;
}
/**
 * Effectuer un trajectoire theta
 * @param angle_voulu  en rad
 */
void Controller::trajectory_theta(float angle_voulu) {

    // récupérer la position actuelle du robot (odométrie)
    Position deltaPos = m_odometry.getPosition();

    // set consigne
    m_consign.distance = 0;
    m_consign.angle = angle_voulu - deltaPos.theta;
    // Borner l'angle
    m_consign.angle = MathUtils::inrange(m_consign.angle, -M_PI, M_PI);

}
void Controller::trajectory_stop() {
    // set consigne angle et distance en 0
    m_consign.distance = 0;
    m_consign.angle = 0;

    m_point_reached = true;
}

void Controller::trajectory_calibration_x() {
    m_odometry.setPosition(
            m_point.getX(),
            m_odometry.getPosition().y,
            MathUtils::deg2rad(m_point.getTheta())
            );

    trajectory_stop();
}
void Controller::trajectory_calibration_xy() {
    m_odometry.setPosition(
            m_point.getX(),
            m_point.getY(),
            MathUtils::deg2rad(m_point.getTheta())
            );

    trajectory_stop();
}
void Controller::trajectory_calibration_y() {
    m_odometry.setPosition(
            m_odometry.getPosition().x,
            m_point.getY(),
            MathUtils::deg2rad(m_point.getTheta())
            );

    trajectory_stop();
}

/**
 * Calcul PID
 * @param consigne_distance : distance to do in mm (is the distance between the robot and the goal position)
 * @param consigne_theta : angle to do in rad (is the angle to the goal relative to the heading of the robot)
 */
void Controller::update_speed(float consigne_distance, float consigne_theta) {

    cout << "CONSIGNE_DISTANCE " << consigne_distance << " - | CONSIGNE THETA:  " << MathUtils::rad2deg(consigne_theta) << endl;


    // On filtre la consigne à l'aide de la QuadrampDerivate pour savoir si l'on est en phase d'accélération, constante ou de décélération
    float ramped_consign_distance = this->m_rampfilterDistance.process(consigne_distance + m_odometry.getTotalDistance(), m_odometry.getTotalDistance(), m_odometry.getDeltaDistance());
    //float ramped_consign_angle = this->m_rampfilterAngle.process(consigne_theta + m_odometry.getTotalTheta(), m_odometry.getTotalTheta(), m_odometry.getDeltaTheta());

    // un mouvement en distance
    // PID distance activé
    if(m_controlDistance) {
        m_speedDistance = m_distancePID.compute(m_odometry.getTotalDistance(), ramped_consign_distance);
//        m_speedDistance = m_distancePID.compute(m_odometry.getDeltaDistance(), consigne_distance);
        // borner la distance (pas nécessaire, la vitesse est déjà borner dans le PID)
        m_speedDistance = max(-m_maxPWM, min(m_maxPWM, m_speedDistance));
    }

    // un mouvement de rotation
    // PID angle activé
    if(m_controlAngle) {
        //m_speedAngle = m_anglePID.compute(m_odometry.getTotalTheta(), ramped_consign_angle);
        m_speedAngle = m_anglePID.compute(m_odometry.getDeltaTheta(), consigne_theta);
        // Borner (pas nécessaire, la vitesse est déjà borner dans le PID)
        m_speedAngle = max(-m_maxPWMR, min(m_maxPWMR, m_speedAngle));
    }

    // On désactive la correction de l'angle lorsque il s'agit d'un point de dérapage (recalage)
    if(m_point.isSlipping()) {
        m_speedAngle = 0;
    }

    int leftPWM = m_speedDistance - m_speedAngle;
    int rightPWM = m_speedDistance + m_speedAngle;

    //détection de dérapage
    if(m_point.isSlipping()) {

        m_slipping_left = false;
        m_slipping_right = false;

        if(leftPWM <= -50 && abs(m_odometry.getLeftVel())<80){
            leftPWM = 0;
            m_slipping_left = true;
            cout << "Dérapage Gauche" << endl;
        }

        if(rightPWM <= -50 && abs(m_odometry.getRightVel())<80){
            rightPWM = 0;
            m_slipping_right = true;
            cout << "Dérapage Droite" << endl;
        }

        //Dérapage des deux cotés, on est contre la bordure, prêt pour le recalage
        if(m_slipping_left && m_slipping_right){
            // asservissement fini!
            m_trajectory = Point::Trajectory::LOCKED;
        }
    }

    m_motor.setConsigne(leftPWM, rightPWM);


    // debug:
    cout << "[ERROR DISTANCE] Error distance : " << m_distancePID.getError() << endl;
    cout << "[ERROR ANGLE] Error Angle : " << m_anglePID.getError() << endl;
    cout << "[PID DISTANCE] Speed Translation : " << m_speedDistance;
    cout << "[PID ANGLE] Speed Rotation : " << m_speedAngle;
    cout << "[PWM] LEFT : " << leftPWM << " RIGHT: " << rightPWM << endl;
    cout << " ======================== " << endl;
}
/**
 * Mettre en place le point voulu à atteindre dans la table
 * @param x  en mm
 * @param y  en mm
 * @param angle  en degré
 */
void Controller::set_point(int x, int y, int angle) {
    m_targetPos.x = x;
    m_targetPos.y = y;
    m_targetPos.theta = MathUtils::deg2rad(angle);
}
/**
 * Mettre en place le point voulu à atteindre dans la table
 * @param Point point
 */
void Controller::set_point_o(Point p) {
    m_targetPos.x = p.getX();
    m_targetPos.y = p.getY();
    m_targetPos.theta = MathUtils::deg2rad(p.getTheta());

    // set point to reach
    this->m_point = p;
}

/**
 * Stop des moteurs et réinitilisation des PID
 */
void Controller::motors_stop() {

    // arrêt des moteurs
    m_motor.stop();
    // reset des erreurs
    m_distancePID.resetErrors();
    m_anglePID.resetErrors();
}
/** near the target (dist in mm and angle in rad) **/
bool Controller::is_target_reached() {

    int distance_tolerance = 8; // mm
    // get errors from PID
    return abs(m_distancePID.getError()) < distance_tolerance;
    //&& (abs(m_anglePID.getError()) < angle_tolerance);
}

/** return true if traj is nearly finished */
bool Controller::is_trajectory_reached() {

    //int distance_tolerance = 10;
    float angle_tolerance = MathUtils::deg2rad(2);
    bool reached = false;

    switch (m_trajectory) {

        case Point::Trajectory::THETA:
            reached = (abs(m_consign.angle) < angle_tolerance);
            //reached = m_rampfilterAngle.isRampFinished();
            break;
        case Point::Trajectory::XY_ABSOLU:
            //reached = is_target_reached();
            reached = m_rampfilterDistance.isRampFinished();
            break;
        case Point::Trajectory::LOCKED:
            reached = m_point_reached;
            break;
        case Point::Trajectory::CALIB_X:
            reached = m_point_reached;
            break;
        case Point::Trajectory::CALIB_Y:
            reached = m_point_reached;
            break;
        case Point::Trajectory::CALIB_XY:
            reached = m_point_reached;
            break;
        case Point::Trajectory::NOTHING:
            reached = true;
            break;
        default:
            reached = false;
    }

    return reached;
}