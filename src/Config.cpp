#include "Config.h"

using namespace std;

/**
 * @Deprected remplacé par Configuration en 2020
 */
// Raccourci de namespace
namespace pt = boost::property_tree;

void Config::loadFromFile(string filename)
{
	// Arbre de configuration
	pt::ptree tree;

	// Chargement depuis un fichier au format INFO
	pt::read_info(filename, tree);

	// Lecture des attributs depuis le fichier de configuration
	delta_asserv = tree.get<int>("asservissement.delta_asserv");

	nbAX12 = tree.get<int>("asservissement.nbAX12");

	pid_kpA = tree.get<double>("asservissement.pid_kpa");
	pid_kiA = tree.get<double>("asservissement.pid_kia");
	pid_kdA = tree.get<double>("asservissement.pid_kda");

	pid_kpDep = tree.get<double>("asservissement.pid_kpdep");
	pid_kiDep = tree.get<double>("asservissement.pid_kidep");
	pid_kdDep = tree.get<double>("asservissement.pid_kddep");

	pid_kpPos = tree.get<double>("asservissement.pid_kppos");
	pid_kiPos = tree.get<double>("asservissement.pid_kipos");
	pid_kdPos = tree.get<double>("asservissement.pid_kdpos");


	CoeffGLong = tree.get<double>("asservissement.CoeffGLong");
	CoeffDLong = tree.get<double>("asservissement.CoeffDLong");
	CoeffAngl = tree.get<double>("asservissement.CoeffAngl");
    CoeffCorrecteur = tree.get<double>("asservissement.coeff_correcteur");

	I2C_SERVOS = tree.get<int>("asservissement.I2C_SERVOS");
	I2C_LANCEUR = tree.get<int>("asservissement.I2C_LANCEUR");
	I2C_MOTEURS = tree.get<int>("asservissement.I2C_MOTEURS");

	temps_match = tree.get<int>("asservissement.temps_match");

	ipServeur = tree.get<string>("asservissement.ipServeur");
	port = tree.get<int>("asservissement.port");

	afficherConfig();
}

int Config::getDeltaAsserv() const { return delta_asserv; }
int Config::getNbAX12() const { return nbAX12; }

string Config::getIpServeur() const { return ipServeur; }
int Config::getPort() const { return port; }

double Config::getPIDkpA() const { return pid_kpA; }
double Config::getPIDkiA() const { return pid_kiA; }
double Config::getPIDkdA() const { return pid_kdA; }

double Config::getPIDkpDep() const { return pid_kpDep; }
double Config::getPIDkiDep() const { return pid_kiDep; }
double Config::getPIDkdDep() const { return pid_kdDep; }

double Config::getPIDkpPos() const { return pid_kpPos; }
double Config::getPIDkiPos() const { return pid_kiPos; }
double Config::getPIDkdPos() const { return pid_kdPos; }

double Config::getCoeffGLong() const { return CoeffGLong; }
double Config::getCoeffDLong() const { return CoeffDLong; }
double Config::getCoeffAngl() const { return CoeffAngl; }
double Config::getCoeffCorrecteur() const { return CoeffCorrecteur; }

int Config::get_I2C_SERVOS() const {return I2C_SERVOS; }
int Config::get_I2C_LANCEUR() const {return I2C_LANCEUR; }
int Config::get_I2C_MOTEURS() const {return I2C_MOTEURS; }


int Config::get_temps_match() const { return temps_match; }

void Config::afficherConfig() const {
	cout << endl << "--- CONFIGURATION ---" << endl;
	cout << "Temps du match : " << get_temps_match() << endl;

	cout << "Ip serveur : " << getIpServeur() << endl;
	cout << "Port serveur : " << getPort() << endl;
	cout << "Temps minimum entre deux appels d'asservissement : " << getDeltaAsserv() << endl << endl;
	cout << "Nombre d'AX12 à connecter : " << getNbAX12() << endl;

	cout << "Coefficients : " << endl;
	cout << "-Longueur :" << endl << "\tGauche : " << getCoeffGLong() << endl << "\tDroite : " << getCoeffDLong() << endl;
	cout << "-Angle : "<< getCoeffAngl()<< endl;

	cout << "PID : " << endl;
	cout << "-Angle :" << endl << "\tP = " << getPIDkpA() << endl << "\tI = " << getPIDkiA() << endl << "\tD = " << getPIDkdA() << endl;
	cout << "-Deplacement :" << endl << "\tP = " << getPIDkpDep() << endl << "\tI = " << getPIDkiDep() << endl << "\tD = " << getPIDkdDep() << endl;
	cout << "-Position :" << endl << "\tP = " << getPIDkpPos() << endl << "\tI = " << getPIDkiPos() << endl << "\tD = " << getPIDkdPos() << endl;

	cout << "I2C :" << endl;
	cout << "\tSERVOS : " << get_I2C_SERVOS() << endl;
	cout << "\tLANCEUR : " << get_I2C_LANCEUR() << endl;
	cout << "\tMOTEURS : " << get_I2C_MOTEURS() << endl;
	cout << "--- FIN DE LA CONFIGURATION ---" << endl;
	return;
}
