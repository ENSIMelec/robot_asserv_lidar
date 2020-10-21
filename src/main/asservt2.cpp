#include <iostream>
#include <thread>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <sys/time.h>
#include <vector>  //for std::vector
using namespace std;

#include "Controller.h"
#include "SerialCodeurManager.h"
#include "Odometry.h"
#include "Point.h"
#include "FichierPoint.h"
#include "MathUtils.h"

/*********************************** Define ************************************/

#define EXIT_SUCCESS 0
#define EXIT_FAIL_PARAM -1
#define EXIT_FAIL_I2C -2
#define EXIT_FORCING_STOP -99

/***************************** Variables globales ******************************/

bool forcing_stop;
unsigned int deltaAsservTimer;

string PATH = "/home/pi/robot_asserv/";

/********************************* prototypes **********************************/

bool argc_control(int argc);
bool argv_contains_dummy(int argc, char** argv);

void jouerMatch(Controller& controller, Odometry& odometry);

void stopSignal(int signal);
void sleepMillis(int millis);
char nomFileStrategy[100];

///////////////////////////// PROGRAMME PRINCIPAL ///////////////////////////////
int main(int argc, char **argv) {
/***************************** Début du programme ******************************/
	cout << "Debut du programme" << endl;

/*************************** Controle des paramètres ***************************/


    // Interception du Ctrl-C pour l'arret des moteurs
	signal(SIGINT, stopSignal);

/***************** Déclaration et initialisation des variables *****************/
	// Initialise les GPIO de la Rasp
	// ATTENTION: à appeler absolument avant d’initialiser les managers
	wiringPiSetupGpio();

	Config config;
	config.loadFromFile("config.info"); //Charge la configuration à partir du fichier config.info
    int I2C_MOTEURS = 8;

	int i2cM = wiringPiI2CSetup(I2C_MOTEURS);
	
	if(i2cM < 0)
		return EXIT_FAIL_I2C;
	
	MoteurManager moteurs(i2cM);


	cout << "Déclaration et initialisation des variables" << endl;


	argv_contains_dummy(argc, argv); //En fonction des paramètres du main, on active les dummy motors ou non

	// Création du groupement de deux codeurs
	SerialCodeurManager codeurs(0);
	//reset du lancement précédent
	codeurs.Closes();
	codeurs.Initialisation();
	delay(100);


	//Setup Connexion udp to Serveur

   	//timer temps;

    deltaAsservTimer = config.getDeltaAsserv();
    Controller controller(codeurs, moteurs, config);
    Odometry odometry(codeurs, config);

    // charger les points pour la stratégie

    sprintf(nomFileStrategy, "%sfilepoint/%s", PATH.c_str(), argv[1]); //Dossier contenant le fichier main.strat et les fichier .point
    //Strategie strat(nomFile, argv[1]);

/***************************** Départ du robot *********************************/
	cout << "Depart du robot" << endl;
	codeurs.reset();
	codeurs.reset();
	cout <<"Codeur reset"<<endl;

	jouerMatch(ref(controller), odometry);

	if(forcing_stop) {
		cout << "Forcing stop" << endl;
	} else {
		cout << "Arrivee du robot" << endl;
	}

/************************ Libération de la mémoire *****************************/
	cout << "Liberation de la memoire" << endl;

	codeurs.Closes();
	close(i2cM);

/***************************** Fin du programme ********************************/
	cout << "Fin du programme" << endl;
	return forcing_stop ? EXIT_FORCING_STOP : EXIT_SUCCESS;
}


/////////////////////////// FIN PROGRAMME PRINCIPAL /////////////////////////////

void jouerMatch(Controller& controller, Odometry& odometry) {

    //timer asservTimer;
    
    vector<Point> strategy = FichierPoint::readPoints(nomFileStrategy);

    for(Point &pt : strategy) {
        cout << "Type de trajectoire : " << pt.getTrajectory() << " X: " << pt.getX() << " Y:" << pt.getY() << " T: " << pt.getTheta() << endl;
    }

    Point pinitial = strategy[0];
    controller.setPosition(pinitial.getX(), pinitial.getY(), MathUtils::deg2rad(pinitial.getTheta()));

    int strategyIndex = 0;

    while(!forcing_stop && strategy.size() >= strategyIndex) {

        // passage au point suivant
        if((controller.is_trajectory_reached() || strategyIndex == 0)
           && strategy.size() >= strategyIndex) {

            cout << "CHANGEMENT DE POINT => POINT: " << strategyIndex << endl;
            Point& point = strategy[strategyIndex];

            controller.set_trajectory(point.getTrajectory());
            controller.set_point(point.getX(), point.getY(), point.getTheta());

            strategyIndex++;
        }
        controller.update();
		sleepMillis(10);
	}
    sleepMillis(10);
    controller.motors_stop();
    sleepMillis(100); //Permet de laisser le temps de demander l'arrêt des moteurs :)
	return;
}


bool argc_control(int argc) {
/**
 * Renvoie EXIT_SUCCESS si tout va bien,
**/
	if(argc == 2)
		return true;
	else
		return false;
}

bool argv_contains_dummy(int argc, char** argv) {
/**
 * Renvoie true si un des arguments demande des dummy motors
**/
	//dummy motors
	const char* DUMMY_SHORTARG = "-dm";
	const char* DUMMY_LONGARG = "--dummy-motors";

	for(int i = 0; i < argc; i++)
	{
		if(strcmp(argv[i], DUMMY_SHORTARG) == 0 || strcmp(argv[i], DUMMY_LONGARG))
			return true;
	}
	return false;
}

void stopSignal(int signal) {
	if(signal)
		cout << "CTRL+C détecté" << endl;
	forcing_stop = true;
	return;
}

void sleepMillis(int millis) {
	std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}
