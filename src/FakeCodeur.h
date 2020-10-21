//
// Created by Taoufik on 11/11/2019.
//

#ifndef ROBOT_FAKECODEUR_H
#define ROBOT_FAKECODEUR_H

#include <thread>
#include "ICodeurManager.h"

class FakeCodeur : public ICodeurManager {

public:
    FakeCodeur();
    ~FakeCodeur();// destructeur
    void readAndReset() override;
    void reset() override;
    void static interrupt();

    static int getRightTicks();
    static int getLeftTicks();
    static int getTime();
private:
    std::thread* m_thread;

protected:
    static int leftTicks, rightTicks, tempsLast;
    int oldLeftTicks = 0, oldRightTicks = 0, oldTempsLast=0;
};


#endif //ROBOT_FAKECODEUR_H
