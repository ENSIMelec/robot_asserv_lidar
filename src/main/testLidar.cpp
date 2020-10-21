#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "MathUtils.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sstream>
#include <string>
#include <iostream>
#include <string>
#include "cmath"

#define PI 3.14159265

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

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace std;
using namespace rp::standalone::rplidar;

float angle;
float distance;
int speed;

size_t count;
rplidar_response_measurement_node_hq_t nodes[8192];
RPlidarDriver *drv;
u_result op_result;
rplidar_response_device_info_t devinfo;

typedef enum { NONE = 0, FORWARD = 1, BACKWARD = -1 } DetectionDirection;
DetectionDirection detection_ob_direction = NONE;




bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            drv->reset();
            return false;
        }
        else {
            return true;
        }

    }
    else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}
int main() {

    cout << "Création de l'objet Lidar..." << endl;

    drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    // connexion
    do
    {
        if (IS_OK(drv->connect("/dev/ttyLIDAR", 115200))) //256000 sinon
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result))
            {
                cout << "lidar connected" << endl;
                //drv->startMotor();

            }
            else
            {
                delete drv;
                drv = NULL;
                cout << "lidar not connected" << endl;

            }
        }

        if (!checkRPLIDARHealth(drv)) {
            RPlidarDriver::DisposeDriver(drv);
            drv = NULL;
            cout << "Rip" << endl;
        }

    } while (!checkRPLIDARHealth(drv));

    drv->startMotor();


    // scan
    count = _countof(nodes);

    // start scan...
    drv->startScan(0,1);

    int etat = ETAT_OBSTACLE_NONE;

    float distance_min = 0;


    while(1){

        distance_min = 65000;
        op_result = drv->grabScanDataHq(nodes, count, 2500);
        if (IS_OK(op_result))
        {

            drv->ascendScanData(nodes, count);
            for (int pos = 1; pos < (int)count ; ++pos)
            {

                /*printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
                       (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
                       (nodes[pos].angle_z_q14 * 90.f / (1 << 14)),
                       nodes[pos].dist_mm_q2/4.0f,
                       nodes[pos].quality);
                */

                //float angle_degree = nodes[pos].angle_q6_checkbit /128.0f;
                float angle_degree = nodes[pos].angle_z_q14 * 90.0f/ (1<<14);
                //float distance_mm = nodes[pos].distance_q2/4.0;
                float distance_mm = nodes[pos].dist_mm_q2/4.0f;

                // distance
                float dp_avant = distance_min * cosf(MathUtils::deg2rad(angle_degree)); //mm
                float dp_arriere = distance_min * -cosf(MathUtils::deg2rad(angle_degree)); //mm

                if((dp_avant > distance_mm || dp_arriere > distance_mm) && distance_mm != 0.0){
                    distance_min = distance_mm;
                    //distance = distance_min;
                    angle = angle_degree;

                    if(dp_avant > distance_mm) {
                        detection_ob_direction = DetectionDirection::FORWARD;
                    } else
                        detection_ob_direction = DetectionDirection::BACKWARD;
                    }
                }

                if(distance_min >= DISTANCE_MAX_SPEED){
                    etat = ETAT_OBSTACLE_NONE; // Pas d'obstacle
                }
                else if(distance_min >= DISTANCE_AVG_SPEED){
                    etat = ETAT_OBSTACLE_EXIST; // Un obstacle dans le coin
                }

                else if(distance_min >= DISTANCE_LOW_SPEED){
                    etat = ETAT_OBSTACLE_CLOSE; // Un obstacle proche du robot
                }
                else {
                    etat = ETAT_OBSTACLE_DANGEROUS; // Un obstacle qui necessite de stoper les moteurs
                }
        }


            cout << "Angle : " << angle << " - Distance min : " << distance_min  << " Direction detection " << detection_ob_direction << endl;
        }


        /*
        if((angle < (ANGLE_KRABS / 2)  || angle > (360 - ANGLE_KRABS / 2)) && Etat_Detection == 1){
            speed = etat;
            cout << "avant !!" << angle <<  " " << Etat_Detection << endl;
        }
        else if((angle > (180 - ANGLE_KRABS / 2) && angle < (180 + ANGLE_KRABS / 2)) && Etat_Detection == -1){
            speed = etat;
            cout << "arriere !!" << angle << " " << Etat_Detection << endl;
        }
        else{
            speed = ETAT_OBSTACLE_NONE; // Détection active mais on trouve rien dans la direction ou on va
            cout << "none !!" << angle << " " << Etat_Detection << endl;
        }
         */

    return 0;
}