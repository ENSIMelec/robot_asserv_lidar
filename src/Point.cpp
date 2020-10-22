//
// Created by Taoufik on 20/12/2019.
//
#include <string>
#include "Point.h"

Point::Point() : m_x(0), m_y(0), m_theta(0),
                 m_trajectory(Point::Trajectory::NOTHING) {

}
Point::Point(float x, float y, float theta, Trajectory mTrajectory) : m_x(x), m_y(y), m_theta(theta),
                                                                                     m_trajectory(mTrajectory) {

}

float Point::getX() const {
    return m_x;
}

float Point::getY() const {
    return m_y;
}

float Point::getTheta() const {
    return m_theta;
}

Point::Trajectory Point::getTrajectory() const {
    return m_trajectory;
}

float Point::getMDistanceTolerance() const {
    return m_distance_tolerance;
}

void Point::setMDistanceTolerance(float mDistanceTolerance) {
    m_distance_tolerance = mDistanceTolerance;
}

float Point::getMAngleTolerance() const {
    return m_angle_tolerance;
}

void Point::setMAngleTolerance(float mAngleTolerance) {
    m_angle_tolerance = mAngleTolerance;
}

int Point::getMSpeed() const {
    return m_speed;
}

void Point::setMSpeed(int mSpeed) {
    m_speed = mSpeed;
}

const string &Point::getMAction() const {
    return m_action;
}

void Point::setMAction(const string &mAction) {
    m_action = mAction;
}

int Point::getMTimeout() const {
    return m_timeout;
}

void Point::setMTimeout(int mTimeout) {
    m_timeout = mTimeout;
}

Point::Trajectory Point::getMTrajectory() const {
    return m_trajectory;
}

void Point::setMTrajectory(Trajectory mTrajectory) {
    m_trajectory = mTrajectory;
}

bool Point::isSlipping() const {
    return m_slip;
}
void Point::setSlipping(bool slip) {
    m_slip = slip;
}

bool Point::getLDetection() const {
    return l_detection;
}

void Point::setLDetection(bool Detection) {
    l_detection = Detection;
}

int Point::getScore() const {
    return p_score;
}

void Point::setScore(int Score){
    p_score = Score;
}



