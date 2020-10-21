#include "SerialCodeurManager.h"
/**
 * @Ta0uf19
 */
using namespace std;
unsigned int nextTime;
int fd ;
// time en ms boucle asservissement
unsigned int waitMs = 10;

SerialCodeurManager::SerialCodeurManager(int init) : initCodeur(init)
{ }

void SerialCodeurManager::Closes()
{
	serialClose(fd);
	cout << "Fermer codeur" << endl;
}

void SerialCodeurManager::Initialisation()
{
	cout << "Initialisation codeur" << endl;

	if ((fd = serialOpen ("/dev/ttyCODEUR", 115200)) < 0)
	{
    	fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
		exit(1);

	} else {
        printf("Codeur serial device : open ok");
    }
    nextTime = millis() + waitMs;
}

void SerialCodeurManager::readAndReset() {

        char data = ' ';
        char left_tics[10000];
        char right_tics[10000];
        char time[10000];
        int g = 0;
        int d = 0;
        int t = 0;

        memset(left_tics, ' ', 10000);
        memset(right_tics, ' ', 10000);
        memset(time, ' ', 10000);


        // récupérer les infos du codeur
        if (millis() > nextTime)
        {
            serialPutchar (fd, 'C') ;
            nextTime += waitMs;
        }

        // attendre 20ms
        delay(waitMs) ;

        while(serialDataAvail(fd)<=0){
            delay(3);
        }

        if(serialDataAvail(fd))
        {
            data = serialGetchar(fd);
            //printf("%c", data);
            if(data == '?') {

                // left tics
                data = serialGetchar(fd);
                while(data !=','){
                    left_tics[g++] = data;
                    data = serialGetchar(fd);
                }

                // right tics
                data = serialGetchar(fd);
                while(data !=':'){
                    right_tics[d++] = data;
                    data = serialGetchar(fd);
                }

                // time
                data = serialGetchar(fd);
                while(data !=';'){
                    time[t++] = data;
                    data = serialGetchar(fd);
                }
            }
            serialFlush(fd);

            // debug
            printf("d = %d, g = %d, t= %d \n", d,g,t);
            printf("Time : %d - ", atoi(time));
            printf("Time Diff : %d - ", atoi(time) - tempsLast);
            printf("Left Tick : %d -" , atoi(left_tics));
            printf("Right Tick : %d \n",  atoi(right_tics));


            // save old vars
            oldLeftTicks = leftTicks;
            oldRightTicks = rightTicks;
            oldTempsLast = tempsLast;

            // new values
            leftTicks = atoi(left_tics);
            rightTicks = atoi(right_tics);
            tempsLast = atoi(time);

             //sanity check
            if(isnan(leftTicks)){
                leftTicks=0;
            }
            if(isnan(rightTicks)){
                rightTicks=0;
            }
            if(isnan(tempsLast)){
                tempsLast=0;
            }
        }
        printf("\n deploqued.");
}

void SerialCodeurManager::reset()
{
	serialPutchar (fd, 'R');
}
