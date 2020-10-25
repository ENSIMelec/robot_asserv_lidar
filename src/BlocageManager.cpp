#include "BlocageManager.h"

using namespace std;

BlocageManager::BlocageManager(Lidar *lid)
{
	this->lid = lid;
}



int BlocageManager::isBlocked() const
{
	int avant;
	int arriere;
	int Table_AVG[50]; // Tableau qui sert � calculer la moyenne des valeurs retourn�es
					   // par le lidar pour �viter les valeurs perdues
	float AVG_speed_detection = 0;
	int N_speed_STP;
	int N_speed_LOW;
	int N_Speed_HIGH;

	//lid->distance // Distance detection obstacle 
	

		if (lid->getDirection() == 1) {
			int i_Table_AVG = 0;
			while (lid->getDirection() == 1 && i_Table_AVG < 20) {
				AVG_speed_detection += lid->etat; //ANgle de detection
				i_Table_AVG++;
				
			}
			if (lid->getDirection() == 1) {
				AVG_speed_detection = AVG_speed_detection / i_Table_AVG;
				if (AVG_speed_detection <= 1.25 && AVG_speed_detection >= 0.75) {
					avant = 1;
				}
				else {
					if (AVG_speed_detection <= 2.25 && AVG_speed_detection >= 1.75) {
						avant = 2;
					}
					else {
						if (AVG_speed_detection <= 3.25 && AVG_speed_detection >= 2.75) {
							avant = 3;
						}
						else {
							if (AVG_speed_detection > 3.25) {
								avant = 4;
							}
							else {
								if (AVG_speed_detection <= 0.25) {
									avant = 0;
								}
								else
									avant = 1;
							}
						}
					}
				}
			}
		}
	

	if(lid->getDirection() == -1){
		int i_Table_AVG = 0;
		while (lid->getDirection() == -1 && i_Table_AVG < 20) {
			AVG_speed_detection += lid->etat;
			i_Table_AVG++;
			
		}
		if (lid->getDirection() == -1 ) {
			AVG_speed_detection = AVG_speed_detection / i_Table_AVG;
			if (AVG_speed_detection <= 1.25 && AVG_speed_detection >= 0.75) {
				arriere = 1;
			}
			else {
				if (AVG_speed_detection <= 2.25 && AVG_speed_detection >= 1.75) {
					arriere = 2;
				}
				else {
					if (AVG_speed_detection <= 3.25 && AVG_speed_detection >= 2.75) {
						arriere = 3;
					}
					else {
						if (AVG_speed_detection > 3.25) {
							arriere = 4;
						}
						else {
							if (AVG_speed_detection <= 0.25) {
								arriere = 0;
							}
							else
								arriere = 1;
						}
					}
				}
			}
		}


	}
	

	if (lid->getDirection() == 1)
		return avant;

	else  //(lid->getDirection() == -1)
		return arriere;
	
}
