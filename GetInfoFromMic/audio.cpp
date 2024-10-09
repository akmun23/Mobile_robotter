#include "audio.h"



std::ofstream outputFile("AudioOutput.txt");  // Open/create a file named "test.txt" for writing
int sampleNumber = 1;                         // To seperate the samples in the file
bool endProgram = false;                      // To end the program after 30 seconds
bool LetterReceived = false;

int Received1 = 8;
int MaxRowMag1 = 0;
int MaxColumnMag1 = 0;
int Received2 = 8;
int MaxRowMag2 = 0;
int MaxColumnMag2 = 0;
int Received3 = 8;
int MaxRowMag3 = 0;
int MaxColumnMag3 = 0;
char Received4 = 'k';
int MaxRowMag4 = 0;
int MaxColumnMag4 = 0;




Audio::Audio() {}

void Audio::checkErr(PaError err) {
    if (err != paNoError) {
        printf("PortAudio error: %s\n", Pa_GetErrorText(err));
        exit(EXIT_FAILURE);
    }
}

inline float Audio::min(float a, float b) {
     return a < b ? a : b;
 }

void Audio::start(){

    // Begin capturing audio
    err = Pa_StartStream(stream);
    checkErr(err);

    // Wait 30 seconds (PortAudio will continue to capture audio)
    // Pa_Sleep(RecordTimeMs);

    while (!endProgram) {

    }

    // Close the file
    // outputFile.close();  // Close the file after writing

}

void Audio::end(){

    // Stop capturing audio
    err = Pa_StopStream(stream);
    checkErr(err);

    // Close the PortAudio stream
    err = Pa_CloseStream(stream);
    checkErr(err);

    // Terminate PortAudio
    err = Pa_Terminate();
    checkErr(err);

    // Free allocated resources used for FFT calculation
    free(spectroData);
    printf("\n");

    printf("Received: %d%d%d%c\n", Received1, Received2, Received3, Received4);
    printf("MaxRowMag: %d - %d - %d - %d\n", MaxRowMag1, MaxRowMag2, MaxRowMag3, MaxRowMag4);
    printf("MaxColumnMag: %d - %d - %d - %d\n", MaxColumnMag1, MaxColumnMag2, MaxColumnMag3, MaxColumnMag4);

}

void Audio::getDevices(){
    int numDevices = Pa_GetDeviceCount();
    printf("Number of devices: %d\n", numDevices);

    if (numDevices < 0) {
        printf("Error getting device count.\n");
        exit(EXIT_FAILURE);
    } else if (numDevices == 0) {
        printf("There are no available audio devices on this machine.\n");
        exit(EXIT_SUCCESS);
    }

    // Display audio device information for each device accessible to PortAudio
    const PaDeviceInfo* deviceInfo;
    for (int i = 0; i < numDevices; i++) {
        deviceInfo = Pa_GetDeviceInfo(i);
        printf("Device %d:\n", i);
        printf("  name: %s\n", deviceInfo->name);
        printf("  maxInputChannels: %d\n", deviceInfo->maxInputChannels);
        printf("  maxOutputChannels: %d\n", deviceInfo->maxOutputChannels);
        printf("  defaultSampleRate: %f\n", deviceInfo->defaultSampleRate);
    }
}




int Audio::streamCallback(
    const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
    void* userData
    ) {

    // Cast our input buffer to a float pointer (since our sample format is `paFloat32`)
    float* in = (float*)inputBuffer;

    // Cast our user data to streamCallbackData* so we can access its struct members
    streamCallbackData* callbackData = (streamCallbackData*)userData;

    /*
    // Write the audio samples to the file
    if (outputFile.is_open()) {  // Check if the file was successfully opened

        // Write the sample number to the file
        outputFile << "Sample " << sampleNumber << ":\n";

        for (unsigned long i = 0; i < framesPerBuffer; i++) {

            // Write some text into the file
            outputFile << in[i * NUM_CHANNELS] << "\n";  // Write the audio sample to the file

        }
    }else {
        std::cout << "Failed to create the file." << std::endl;  // Display an error message if file creation failed
    }

    sampleNumber++;  // Increment the sample number for the
    outputFile << "\n" << "\n";  // Write a newline character to the file*/

    for (unsigned long i = 0; i < framesPerBuffer; i++) {
        callbackData->in[i] = in[i * NUM_CHANNELS];
    }
    double pi = 3.14159265358979323846;

    std::vector<int> tones = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
    std::vector<double> mags(tones.size());

    for (int i = 0; i < tones.size(); ++i) {

        double k0 = FRAMES_PER_BUFFER*tones[i]/SAMPLE_RATE;

        double omega_I = cos(2*pi*k0/FRAMES_PER_BUFFER);
        double omega_Q = sin(2*pi*k0/FRAMES_PER_BUFFER);
        double v1 = 0;
        double v2 = 0;
        for (int n = 0; n < FRAMES_PER_BUFFER; ++n) {
            double v  = 2*omega_I*v1 - v2 + callbackData->in[n];
            v2 = v1;
            v1 = v;
        }


        double y_I = v1 - omega_I * v2;
        double y_Q = omega_Q * v2;

        mags[i] = sqrt(y_I*y_I + y_Q*y_Q);

        //mags[i] = v1*v1 + v2*v2-v1*v2*(2*omega_I);

    }

    /*
    // Print the magnitudes of the tones
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
    printf("    ");
    printf("%d ", tones[7]);
    printf("%f ", mags[7]);
    fflush(stdout);*/

    if(analyseGoertzelOutput(mags)){
        endProgram = true;
        return paAbort;

    }else{
        return paContinue;
    }

}





void Audio::Init(){

    // Initialize PortAudio
    err = Pa_Initialize();
    checkErr(err);

    // Allocate and define the callback data
    spectroData = (streamCallbackData*)malloc(sizeof(streamCallbackData));
    spectroData->in = (double*)malloc(sizeof(double) * FRAMES_PER_BUFFER);
    if (spectroData->in == NULL) {
        printf("Could not allocate spectro data\n");
        exit(EXIT_FAILURE);
    }

    // Define stream capture specifications
    memset(&inputParameters, 0, sizeof(inputParameters));
    inputParameters.channelCount = NUM_CHANNELS;
    inputParameters.device = Pa_GetDefaultInputDevice();
    inputParameters.hostApiSpecificStreamInfo = NULL;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(Pa_GetDefaultInputDevice())->defaultLowInputLatency;

    // Open the PortAudio stream
    err = Pa_OpenStream(
        &stream,
        &inputParameters,
        NULL,
        SAMPLE_RATE,
        FRAMES_PER_BUFFER,
        paNoFlag,
        streamCallback,
        spectroData
        );
    checkErr(err);
}






bool Audio::analyseGoertzelOutput(std::vector<double> mags){
    std::vector<double> rowMags = {mags[0], mags[1], mags[2], mags[3]};
    std::vector<double> columnMags = {mags[4], mags[5], mags[6], mags[7]};

    int maxRow = 0;
    int maxColumn = 0;

    for (int i = 1; i < rowMags.size(); ++i) {
        if (rowMags[i] > rowMags[maxRow]) {
            maxRow = i;
        }
    }

    for (int i = 1; i < columnMags.size(); ++i) {
        if (columnMags[i] > columnMags[maxColumn]) {
            maxColumn = i;
        }
    }

    int MinMagnitude = 200;

    if(rowMags[maxRow] > MinMagnitude && columnMags[maxColumn] > MinMagnitude && !LetterReceived){
        LetterReceived = true;

        if(maxRow < 3 && maxColumn < 3){
            printf("\r");
            printf("Button pressed: %d", maxRow*3 + maxColumn + 1);
            fflush(stdout);

            if(Received1 == 8){
                Received1 = maxRow*3 + maxColumn + 1;
                MaxRowMag1 = rowMags[maxRow];
                MaxColumnMag1 = columnMags[maxColumn];
            }else if(Received2 == 8){
                Received2 = maxRow*3 + maxColumn + 1;
                MaxRowMag2 = rowMags[maxRow];
                MaxColumnMag2 = columnMags[maxColumn];
            }else if(Received3 == 8){
                Received3 = maxRow*3 + maxColumn + 1;
                MaxRowMag3 = rowMags[maxRow];
                MaxColumnMag3 = columnMags[maxColumn];
            }
        }else if(maxRow == 3){
            if(maxColumn == 0){
                printf("\r");
                printf("Button pressed: E");
                fflush(stdout);
            }else if(maxColumn == 1){
                printf("\r");
                printf("Button pressed: 0");
                fflush(stdout);
            }else if(maxColumn == 2){
                printf("\r");
                printf("Button pressed: F");
                fflush(stdout);
            }else{
                printf("\r");
                printf("Button pressed: D");
                fflush(stdout);
                Received4 = 'D';
                MaxRowMag4 = rowMags[maxRow];
                MaxColumnMag4 = columnMags[maxColumn];
                return true;
            }
        }else{
            if(maxRow == 0){
                printf("\r");
                printf("Button pressed: A");
                fflush(stdout);
            }else if(maxRow == 1){
                printf("\r");
                printf("Button pressed: B");
                fflush(stdout);
            }else if(maxRow == 2){
                printf("\r");
                printf("Button pressed: C");
                fflush(stdout);
            }
        }
    }else if((rowMags[maxRow] < MinMagnitude || columnMags[maxColumn] < MinMagnitude) && LetterReceived){
        printf("\r");
        printf("No button pressed");
        fflush(stdout);
        LetterReceived = false;
    }

    return false;



}
