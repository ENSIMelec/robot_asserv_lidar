#ifndef BLOCAGEMANAGER_H_INCLUDED
#define BLOCAGEMANAGER_H_INCLUDED

#include <wiringPi.h>
#include <iostream>
#include <lidar.h>




// Système de flags pour savoir à un instant T quels sont les positions du
// blocage à l’aide de && logique
enum class PositionBlocage {
	AUCUN = 0,
	AVANT = 1,
	ARRIERE = 2,
	BOTH = 3
};

// Opérateur permettant de faire un && entre deux positions de blocage pour
// savoir si un endroit est bloqué ou pas sans devoir faire les casts à chaque
// fois
inline bool operator&&(PositionBlocage a, PositionBlocage b)
{
	return static_cast<int>(a) && static_cast<int>(b);
}

class BlocageManager
{
public:
	BlocageManager(Lidar *lid);

	int isBlocked() const;
	Lidar *lid;

	void sleepMillis(int millis);

};

#endif //BLOCAGEMANAGER_H_INCLUDED