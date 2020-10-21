//
// Created by Taoufik on 20/12/2019.
//

#ifndef ROBOT_POINT_H
#define ROBOT_POINT_H

using namespace std;

class Point {

public:
    // Les types d'un point
    enum Trajectory {
        THETA,
        XY_ABSOLU,
        LOCKED,
        NOTHING,
        CALIB_X,
        CALIB_Y,
        CALIB_XY
    };

    Point();
    Point(float mX, float mY, float mTheta, Point::Trajectory mTrajectory);
    float getX() const;
    float getY() const;
    float getTheta() const;

    float getMDistanceTolerance() const;
    void setMDistanceTolerance(float mDistanceTolerance);
    float getMAngleTolerance() const;
    void setMAngleTolerance(float mAngleTolerance);
    int getMSpeed() const;
    void setMSpeed(int mSpeed);
    const string &getMAction() const;
    void setMAction(const string &mAction);
    int getMTimeout() const;
    void setMTimeout(int mTimeout);
    Point::Trajectory getMTrajectory() const;
    void setMTrajectory(Trajectory mTrajectory);
    Point::Trajectory getTrajectory() const;
    bool isSlipping() const;
    void setSlipping(bool slip);

private:

    float m_x = 0;
    float m_y = 0;
    float m_theta = 0;
    float m_distance_tolerance;
    float m_angle_tolerance;
    int m_speed;
    bool m_slip = false; // dérapage

    string m_action;
    int m_timeout;
    Point::Trajectory m_trajectory = Point::Trajectory::NOTHING;

};


#endif //ROBOT_POINT_H
