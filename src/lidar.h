#ifndef LIDAR_H
#define LIDAR_H


#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include <stdio.h>
#include <stdlib.h>
#include <ostream>
#include <iostream>
#include <signal.h>
#include <math.h>
#include "Controller.h"

using namespace std;
using namespace rp::standalone::rplidar;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

// Plages de vitesse
#define DISTANCE_MAX_SPEED 700
#define DISTANCE_AVG_SPEED 600
#define DISTANCE_LOW_SPEED 450

// Etats des obstacles
#define ETAT_OBSTACLE_NONE 3
#define ETAT_OBSTACLE_EXIST 2
#define ETAT_OBSTACLE_CLOSE 1
#define ETAT_OBSTACLE_DANGEROUS 0

// Vitesse
// Angle de détection de 90°
#define ANGLE_KRABS 120

class Lidar
{
public:

    /*enum DetectionDirection { NONE = 0, FORWARD = 1, BACKWARD = -1 };
    DetectionDirection detection_ob_direction = NONE;*/
	size_t count;
	int etat = ETAT_OBSTACLE_NONE;
	
	//Constructeur et destruteur par defaut
	//Lidar();
	Lidar();
	~Lidar();

	//Verifie que le lidar est present et branche correctement
	bool checkRPLIDARHealth(RPlidarDriver* drv);

	//Lance un scan de 0 a 360 deg et stoque les valeurs dans nodes[]
	void Scan();
	void stopMotor();
    void setDirectionMotor(int direction) { this->m_direction = direction; };
    int getDirection() { return this->m_direction; };

private:
    rplidar_response_measurement_node_hq_t nodes[8192];
	RPlidarDriver * drv;
	u_result op_result;
	rplidar_response_device_info_t devinfo;


	float angle; // degré
	float distance; //mm
	int m_direction;


};

#endif // LIDAR_H