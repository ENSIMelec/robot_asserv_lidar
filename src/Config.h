#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#include <string>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

class Config
{
public:

	/// @brief Charge la configuration depuis un fichier
	///
	/// @param filename Chemin du fichier de config (.info)
	void loadFromFile(std::string filename);
	void afficherConfig() const;
	/// @brief Retourne le temps d’attente entre chaque appel à l’asservissement
	///
	/// @return Temps d’attente en millisecondes
	int getDeltaAsserv() const;
	int getNbAX12() const;

	std::string getIpServeur() const;
	int getPort() const;

	double getPIDkpA() const;
	double getPIDkiA() const;
	double getPIDkdA() const;

	double getPIDkpDep() const;
	double getPIDkiDep() const;
	double getPIDkdDep() const;

	double getPIDkpPos() const;
	double getPIDkiPos() const;
	double getPIDkdPos() const;

	double getCoeffGLong() const;
	double getCoeffDLong() const;
	double getCoeffAngl() const;
    double getCoeffCorrecteur() const;

	int get_I2C_SERVOS() const;
	int get_I2C_LANCEUR() const;
	int get_I2C_MOTEURS() const;

	int get_temps_match() const;

private:
	// Attributs
	int delta_asserv;
	int nbAX12;

	std::string ipServeur;
	int port;

	double pid_kpDep, pid_kiDep, pid_kdDep;
	double pid_kiPos,pid_kpPos, pid_kdPos;
	double pid_kpA, pid_kiA, pid_kdA;

	double CoeffGLong, CoeffDLong, CoeffAngl;
	double CoeffCorrecteur;

	int I2C_SERVOS, I2C_LANCEUR, I2C_MOTEURS;

	int temps_match;
};

#endif //CONFIG_H_INCLUDED