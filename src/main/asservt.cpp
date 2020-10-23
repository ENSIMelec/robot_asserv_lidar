#include <iostream>
#include <thread>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sstream>

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
#include "ActionManager.h"
#include "Configuration.h"
#include "Timering.cpp"
#include "lidar.h"
#include "BlocageManager.h"
#include "InitRobot.h"						   

/*********************************** Define ************************************/

#define EXIT_SUCCESS 0
#define EXIT_FAIL_PARAM -1
#define EXIT_FAIL_I2C -2
#define EXIT_FORCING_STOP -99

/***************************** Variables globales ******************************/

bool forcing_stop;
unsigned int deltaAsservTimer;
unsigned int nbActionFileExecuted;

string PATH = "/home/pi/robot_asserv_lidar/";

/********************************* prototypes **********************************/

bool argc_control(int argc);
bool argv_contains_dummy(int argc, char** argv);
void actionThreadFunc(ActionManager& actionManager, string filename, bool& actionEnCours, bool& actionDone);

void jouerMatch(Controller& controller, Odometry& odometry, ActionManager& actions, ClientUDP &clientUdp);

void stopSignal(int signal);
void sleepMillis(int millis);
char nomFileStrategy[100];


Lidar* lid;

bool relancer = false;

void lidar(Lidar* lidar) {
    cout << " ********************************************** ENTREE DANS SCAN ********************************************** " << endl;
    lidar->Scan();
    cout << " ********************************************** SORTIE DE SCAN ********************************************** " << endl;
    cout << "thread Lidar" << endl;
    relancer = true;
}
								 
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

    int i2cS = wiringPiI2CSetup(config.get_I2C_SERVOS());
    //Adresse i2c du nema, devrait passer dans le fichier conf si jamais
    int i2cSt = wiringPiI2CSetup(9);

    if(i2cS < 0 || i2cSt < 0 || i2cM < 0)
        return EXIT_FAIL_I2C;

    // initialisation du moteur
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
    string ipServeur = "172.24.1.50";
    int portServeur = 90;
    ClientUDP client(ipServeur, portServeur);

    ActionManager actions(i2cS, i2cSt, config.getNbAX12(), client);

   	//timer temps;

    deltaAsservTimer = config.getDeltaAsserv();
    Controller controller(codeurs, moteurs, config);
    Odometry odometry(codeurs);

    // charger les points pour la stratégie

    sprintf(nomFileStrategy, "%sfilepoint/%s", PATH.c_str(), argv[1]); //Dossier contenant le fichier main.strat et les fichier .point
    //Strategie strat(nomFile, argv[1]);

/***************************** Départ du robot *********************************/
	lid = new Lidar();

    cout << "lid init" << endl;
    thread lidarThread1(lidar, lid);
    cout << "thread1 init" << endl;

    sleepMillis(100);
    InitRobot init_robot;
    init_robot.waitForRobotInitialisation(0);
	cout << "Depart du robot" << endl;
	codeurs.reset();
	//codeurs.reset();
	cout <<"Codeur reset = done"<<endl;

	jouerMatch(ref(controller), odometry, actions, client);

	lid->stopMotor();
    delete lid;
	if(forcing_stop) {
		cout << "Forcing stop" << endl;
	} else if(!InitRobot::aruIsNotPush()) {
		cout << "ARU" << endl;
	} else {
		cout << "Arrivee du robot" << endl;
	}

/************************ Libération de la mémoire *****************************/
	cout << "Liberation de la memoire" << endl;
    actions.close(); //Ferme les AX12
	codeurs.Closes();
	close(i2cM);
    close(i2cS);
    close(i2cSt);

/***************************** Fin du programme ********************************/
	cout << "Fin du programme" << endl;
	return forcing_stop ? EXIT_FORCING_STOP : EXIT_SUCCESS;
}


/////////////////////////// FIN PROGRAMME PRINCIPAL /////////////////////////////

void actionThreadFunc(ActionManager& actionManager, string filename, bool& actionEnCours, bool& actionDone) {
    cout << "Debut du THREAD action " << filename << endl;
    actionManager.action(PATH + "fileaction/" + filename + ".as");
    actionEnCours = false;
    actionDone = true;
    cout << "Fin du THREAD action " << filename << endl;
    return;
}

void jouerMatch(Controller& controller, Odometry& odometry, ActionManager& actions, ClientUDP &clientUdp) {

    cout << "jouerMatch launch" << endl;
    ClientUDP ecranUDP("127.0.0.1", 1111);
    //ecranUDP.sendMessage("P 30");
	while(InitRobot::jackIsPresent()){
        sleepMillis(1);
    }

    timer asservTimer;


    bool thereIsAnAction = false, actionDone = false, actionEnCours = false;
    nbActionFileExecuted = 0;
    vector<Point> strategy = FichierPoint::readPoints(nomFileStrategy);

    for(Point &pt : strategy) {
        cout << "Type de trajectoire : " << pt.getTrajectory() << " X: " << pt.getX() << " Y:" << pt.getY() << " T: " << pt.getTheta() << endl;
    }

    // position of robot
    Point pinitial = strategy[0];
    controller.setPosition(pinitial.getX(), pinitial.getY(), MathUtils::deg2rad(pinitial.getTheta()));

    int strategyIndex = 0;
    Point point(0,0,0, Point::Trajectory::NOTHING);
    Point scorePoint(0,0,0, Point::Trajectory::NOTHING);
	
	const int FIN_MATCH_S = 100; // en secondes
	const int DEPLOYER_PAV_S = 95; // en secondes
	// match time
    Timering matchTimer;
    matchTimer.start();

    bool deployed_pav = false;
    // baisser le pavillon au début du match
    thread(actionThreadFunc, ref(actions), "BaisserPavillon", ref(actionEnCours), ref(actionDone)).detach();

    while(!forcing_stop && InitRobot::aruIsNotPush() && matchTimer.elapsedSeconds() < FIN_MATCH_S) {
        //cout << "time elapsed(s): " << matchTimer.elapsedSeconds() << endl;
		// déployer le pavillon
		if(matchTimer.elapsedSeconds() >= DEPLOYER_PAV_S && !deployed_pav) {
            cout << "Déployment du pavillon" << endl;
			// créer un thread qui effectue l'action
			actionEnCours = true;
			thread(actionThreadFunc, ref(actions), "HisserPavillon", ref(actionEnCours), ref(actionDone)).detach();
            deployed_pav = true;
            ecranUDP.addPoints(10,0);
		}
		// vérification de fin match
		// vérification asservissement
        if(strategy.size() >= strategyIndex+1  && asservTimer.elapsed_ms() >= deltaAsservTimer) {
            // passage au point suivant
            if ((controller.is_trajectory_reached() || strategyIndex == 0)
                && strategy.size() >= strategyIndex) {

                cout << "CHANGEMENT DE POINT => POINT: " << strategyIndex << endl;
                point = strategy[strategyIndex];

                // set trajectory type
                controller.set_trajectory(point.getTrajectory());
                // set point coords
                //controller.set_point(point.getX(), point.getY(), point.getTheta());
                controller.set_point_o(point);

                strategyIndex++;

                scorePoint =  strategy[strategyIndex-1];
                ecranUDP.addPoints(scorePoint.getScore(),0);
            }

			      
            //Ajout du bloc concernant le lidar
            BlocageManager blocage(lid); //Gestionnaire de blocage
            bool weAreBlocked = false; //Est ce qu'on bloque ?

            lid->setDirectionMotor(controller.getm_direction());
            cout << "etat blockage " << (blocage.isBlocked()) << InitRobot::aruIsNotPush() << endl;

            while ((blocage.isBlocked() < 2) && !forcing_stop && InitRobot::aruIsNotPush() 
                && point.getDetection() && matchTimer.elapsedSeconds() < FIN_MATCH_S) { //Si il y a e un obstacle genant
                weAreBlocked = true;
                controller.motors_stop(); //Tant qu'on est bloqué
                controller.set_trajectory(Point::Trajectory::LOCKED);
                //if(!controller.is_trajectory_reached())

                controller.update();

                if(matchTimer.elapsedSeconds() >= DEPLOYER_PAV_S && !deployed_pav) {
                    cout << "Déployment du pavillon" << endl;
			        // créer un thread qui effectue l'action
			        actionEnCours = true;
			        thread(actionThreadFunc, ref(actions), "HisserPavillon", ref(actionEnCours), ref(actionDone)).detach();
                    deployed_pav = true;
                    ecranUDP.addPoints(10,0);
		        }

                cout << endl << "************************* WeAreBlocked *************************" << endl;
                cout << endl << blocage.isBlocked() << endl;
                cout << endl << controller.getm_direction() << endl;
                sleepMillis(10);
            }
            if(weAreBlocked) {
                controller.set_trajectory(point.getTrajectory());
            }
			//Fin d'ajout du bloc concernant le lidar
			
            if (((point.getMAction()).compare("null") != 0) && ((point.getMAction()).compare("") != 0)) {
                //Si il y a un fileAction
                thereIsAnAction = true;
                actionDone = false;
            } else {
                thereIsAnAction = false;
                actionDone = true;
                //Permet de sécuriser le fait que si le point demande la fin de l'action, et qu'il n'y a pas d'action à lancer (null), alors l'action est marquée comme "faite"
            }
            actionEnCours = false;

            // Gestion des actions
            if (thereIsAnAction && !actionDone &&
                !actionEnCours) { //Si on a une action a faire et qu'elle n'est pas faite et qu'elle n'est pas en cours
                thread(actionThreadFunc, ref(actions), point.getMAction(), ref(actionEnCours),
                       ref(actionDone)).detach();//créer un nouveau Thread qui effectue l'action
                actionEnCours = true;
                nbActionFileExecuted++;
            }

            // debug position emulator
            stringstream ss;
            ss << "X:" << controller.getOdometry().getPosition().x << ","
               << "Y:" << controller.getOdometry().getPosition().y << ","
               << "T:" << MathUtils::rad2deg(controller.getOdometry().getPosition().theta) << "\n";

            std::string buffer = ss.str();
            clientUdp.sendMessage(buffer);

            // asservissement moteur
            controller.update();
            asservTimer.restart();
        }
	}

    std::cout << "End of boucle!" << endl;
    sleepMillis(20);
    controller.motors_stop();
    sleepMillis(100);
    //Permet de laisser le temps de demander l'arrêt des moteurs :)
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
