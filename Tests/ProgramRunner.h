#ifndef PROGRAMRUNNER_H
#define PROGRAMRUNNER_H


#include "audio.h"
#include "AlgorithmCompareTest.h"
#include "sound.h"

//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>


void RunListeningProgram() {
    Audio audio;
    audio.Init();
    audio.start();
    audio.end();
}

void RecordingProgram() {
    std::thread thread1(RunSoundProgram);
    std::thread thread2(RunListeningProgram);

    thread1.join();
    thread2.join();
}

int RunProgram(int ProgramChoice) {


    if (ProgramChoice == 1){
        RunListeningProgram();
    }else if (ProgramChoice == 2){
        RunCompareTest();
    }else if (ProgramChoice == 3){
        RecordingProgram();
    }




    return 0;

}
#endif // PROGRAMRUNNER_H
