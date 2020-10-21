#include "PID.h"
using namespace std;
/**
 * Construction PID
 * @param kp
 * @param ki
 * @param kd
 * @param min
 * @param max
 */
PID::PID(float kp, float ki, float kd, float min, float max) {

    this->m_kp = kp;
    this->m_ki = ki;
    this->m_kd = kd;
    this->m_min = min;
    this->m_max = max;
}
/**
 * @brief Calculer le PID
 *
 * @param currentState L'état actuelle
 * @param consigne  Goal
 * @param timestep
 * @return
 */
float PID::compute(float currentState, float consigne) {

    // Ecart entre la consigne et la mesure
    float error = consigne - currentState;

    // Proportional term
    float Pout = m_kp * error;

    //cout << " ERROR : " << error << endl;
    //cout << " ERROR Pout (kp*error): " << Pout << endl;

    // Integral term
    this->m_integral += error;
    float Iout = m_ki * this->m_integral;

    // Derivative term
    this->m_derivative = (error - m_pre_error);
    float Dout = m_kd * this->m_derivative;

    // Compute the PID controller's output
    float output = Pout + Iout + Dout;


    // Restrict to max/min
    if( output > m_max )
        output = m_max;
    else if( output < m_min )
        output = m_min;

    // Save error to previous error
    this->m_pre_error = error;

    // Returns the manipulated variable given a setpoint and current process value
    return output;
}

void PID::setTunings(float kp, float ki, float kd) {
    if (kp < 0 || ki < 0 || kd < 0)
        return;

    this->m_kp = kp;
    this->m_ki = ki;
    this->m_kd = kd;
}

void PID::resetErrors() {
    this->m_pre_error = 0;
    this->m_integral = 0;
    this->m_derivative = 0;
}

float PID::getCurrentGoal() const {
    return m_goal;
}

void PID::setGoal(float goal) {
    this->m_goal = goal;
}

