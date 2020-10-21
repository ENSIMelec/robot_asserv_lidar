#include "FakeCodeur.h"
#include "cstdlib"
#include <ctime>
#include <iostream>
#include "thread"
#include <unistd.h>
using namespace std;

int FakeCodeur::leftTicks = 0;
int FakeCodeur::rightTicks = 0;
int FakeCodeur::tempsLast = 0;

FakeCodeur::FakeCodeur() {
    m_thread = new thread([this]() { interrupt(); } );
}

void FakeCodeur::reset() {

}

void FakeCodeur::readAndReset() {

}

int FakeCodeur::getRightTicks() {
    return FakeCodeur::rightTicks;
}

int FakeCodeur::getLeftTicks() {
    return FakeCodeur::leftTicks;
}
int FakeCodeur::getTime() {
    return FakeCodeur::tempsLast;
}

void FakeCodeur::interrupt() {

    while(1) {
        leftTicks +=1;
        rightTicks +=1;
        tempsLast += 100;

        usleep(2000);
    }
}

FakeCodeur::~FakeCodeur() {
    m_thread->detach();
}