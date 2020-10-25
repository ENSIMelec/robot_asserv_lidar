//
// Created by Taoufik on 12/11/2019.
//

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H


#include "PID.h"
#include "ICodeurManager.h"
#include "Odometry.h"
#include "MoteurManager.h"
#include "MathUtils.h"
#include "Config.h"
#include "QuadrampDerivate.h"
#include "Point.h"

class Controller {

public:
    /**
     * Initialisation de l'asservisement
     * @param codeurs
     * @param motor
     * @param config
    */
    Controller(ICodeurManager& codeurs, MoteurManager& motor);
    /** enum Direction
    *  \brief Sens de déplacement pour le robot.
    */
    enum Direction {
        FORWARD     = 1, ///< Le robot avance en marche avant.
        BACKWARD    = -1 ///< Le robot avance en marche arrière.
    };

//    enum Trajectory {
//        THETA,
//        XY_ABSOLU,
//        LOCKED,
//        NOTHING,
//        CALIB_X,
//        CALIB_Y,
//        CALIB_XY
//    };

    /**
     * Method asserv
     */
    void update();
    void set_point(int x, int y, int angle);
    void set_point_o(Point p);
    void set_trajectory(Point::Trajectory trajectory) { m_trajectory = trajectory; }
    void motors_stop();
    bool is_target_reached();
    bool is_target_reached_xy();
    bool is_target_reached_angle();
    bool is_trajectory_reached();
    void setPosition(float x, float y, float theta) { m_odometry.setPosition(x, y, theta); }
    Odometry& getOdometry() { return m_odometry; }
    int getm_direction() { return m_direction; }

    //test
    bool isRampFinished() { return  m_rampfilterDistance.isRampFinished() && m_rampfilterAngle.isRampFinished(); }


private:

    void update_speed(float consigne_distance, float consigne_theta);
    void trajectory_theta(float angle_voulu);
    void trajectory_xy(float x_voulu, float y_voulu);
    void trajectory_stop();
    void trajectory_calibration_x(); // recalage en X
    void trajectory_calibration_y(); // recalage en Y
    void trajectory_calibration_xy(); // recalage en X et Y
    void trajectory_distance_finished();
    void trajectory_angle_finished();
    void trajectory_distance_angle();
    void set_consigne_distance_theta(float new_distance, float new_angle);

    // PID Controller
    PID m_leftSpeedPID;
    PID m_rightSpeedPID;
    PID m_distancePID;
    PID m_anglePID;

    // Target position
    Position m_targetPos;

    //Trajectory actuelle
    Point::Trajectory m_trajectory = Point::Trajectory::NOTHING;

    /**
     * Structure de la consigne à atteindre
     */
    struct
    {
        float distance = 0.0; // mm
        float angle = 0.0; // rad

    } m_consign;

    /**
     * Structure de la consigne initial à atteindre
     */
    struct {
        float distance = 0.0; // mm
        float angle = 0.0; // rad
    } m_consignInitial;

    int m_direction = Direction::FORWARD;

    float m_maxTranslationSpeed = 0; // mm/s
    float m_maxRotationSpeed = 0; // mm/s
    int m_maxPWM = 0;
    int m_maxPWMR = 0;

    //Odometry
    Odometry m_odometry;

    // MoteurManager
    MoteurManager m_motor;
    //Config
    Config m_config;

    // enabled cs system
    bool m_controlDistance = true;
    bool m_controlAngle = true;

    // cs speed
    int m_speedDistance = 0;
    int m_speedAngle = 0;

    // slipping
    bool m_slipping_right = false;
    bool m_slipping_left = false;

    // Point to reach
    Point m_point;

    // Point is reached ?
    bool m_point_reached = false;

    // Quadramp filter distance
    QuadrampDerivate m_rampfilterDistance;
    // Quadramp filter angle
    QuadrampDerivate m_rampfilterAngle;

};


#endif //ROBOT_CONTROLLER_H
