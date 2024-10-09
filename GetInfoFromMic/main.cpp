#include "audio.h"



// #include <fstream>   // Only needed if you want to read from a file


int main() {

    Audio audio;
    audio.Init();
    audio.start();
    audio.end();





    // Testing the Goertzel algorithm from a dtmf signal loaded to a file sample rate 48000
    /*
    std::ifstream inFile;


    inFile.open("/home/pascal/Dokumenter/3.Semester/SemesterProjekt/Test/GetInfoFromMic/DTMF9R.txt");

    if (!inFile) {
        std::cerr << "Unable to open file datafile.txt";
        exit(1);   // call system to stop
    }
    double x;
    std::vector<double> soundData;

    while (inFile >> x) {
         soundData.push_back(x);
    }

    double pi = 3.14159265358979323846;
    std::vector<int> tones = {697, 770, 852, 941, 1209, 1336,1477};
    std::vector<double> mags(7);

    double FramesPerBuffer = soundData.size();
    double sampleRate = 48000;

    for (int i = 0; i < tones.size(); ++i) {

        double k0 = FramesPerBuffer*tones[i]/sampleRate;

        double omega_I = cos(2*pi*k0/FramesPerBuffer);
        double omega_Q = sin(2*pi*k0/FramesPerBuffer);
        double v1 = 0;
        double v2 = 0;
        for (int n = 0; n < FramesPerBuffer; ++n) {
            double v  = 2 * omega_I * v1 - v2 + soundData[n];
            v2 = v1;
            v1 = v;
        }

        double y_I = v1 - omega_I * v2;
        double y_Q = omega_Q * v2;

        mags[i] = sqrt(y_I*y_I + y_Q*y_Q);

    }

    printf("\r");
    printf("Tones: ");
    printf("%d ", tones[0]);
    printf("%f ", mags[0]);
    printf("    ");
    printf("%d ", tones[1]);
    printf("%f ", mags[1]);
    printf("    ");
    printf("%d ", tones[2]);
    printf("%f ", mags[2]);
    printf("    ");
    printf("%d ", tones[3]);
    printf("%f ", mags[3]);
    printf("    ");
    printf("%d ", tones[4]);
    printf("%f ", mags[4]);
    printf("    ");
    printf("%d ", tones[5]);
    printf("%f ", mags[5]);
    printf("    ");
    printf("%d ", tones[6]);
    printf("%f ", mags[6]);
    fflush(stdout);
    printf("\n");*/




    return EXIT_SUCCESS;
}
