#ifndef PROGRAMRUNNER_H
#define PROGRAMRUNNER_H


#include "audio.h"
#include "AlgorithmCompareTest.h"
#include "sound.h"

//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>


void RunListeningProgram() {
    Goertzel audio(2, 0.150);
    audio.Init();
    audio.start();
    audio.end();
}
void RunListeningProgramFFT() {
    Goertzel audio(0.2, 0.150);
    audio.InitForFFT();
    audio.startFFT();
}
void RunListeningProgramCompare() {
    Goertzel audio(0.5, 0.150);
    audio.InitForCompare();
    audio.startCompare();
}


void ListeningAndAudioProgram() {
    std::thread thread1(RunSoundProgram);
    std::thread thread2(RunListeningProgram);

    thread1.join();
    thread2.join();
}

void FileStoringProgram() {
    Goertzel audio(20, 0.140);
    audio.InitForStoringInFile();
    audio.startTimedRecording(52);
    audio.end();
}

void StoreRecordingInFile() {
    std::thread thread1(RunSoundProgram);
    std::thread thread2(FileStoringProgram);

    thread1.join();
    thread2.join();
}

void ListeningAndAudioProgramFFT() {
    std::thread thread1(RunSoundProgram);
    std::thread thread2(RunListeningProgramFFT);

    thread1.join();
    thread2.join();
}

void ListeningAndAudioProgramCompare() {
    std::thread thread1(RunSoundProgram);
    std::thread thread2(RunListeningProgramCompare);

    thread1.join();
    thread2.join();
}


int RunProgram(int ProgramChoice) {


    if (ProgramChoice == 1){
        RunListeningProgram();
    }else if (ProgramChoice == 2){
        RunCompareTest();
    }else if (ProgramChoice == 3){
        ListeningAndAudioProgram();
    }else if (ProgramChoice == 4){
        StoreRecordingInFile();
    }else if (ProgramChoice == 5){
        RunSoundProgram();
    }else if (ProgramChoice == 6){
        ListeningAndAudioProgramFFT();
    }else if (ProgramChoice == 7){
        ListeningAndAudioProgramCompare();
    }else{
        std::cout << "Invalid Program Choice" << std::endl;
    }




    return 0;

}
#endif // PROGRAMRUNNER_H
