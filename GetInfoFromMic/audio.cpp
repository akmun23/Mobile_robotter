#include "audio.h"

std::ofstream outputFile("AudioOutput.txt");  // Open/create a file named "test.txt" for writing
int sampleNumber = 1;                         // To seperate the samples in the file
bool endProgram = false;                      // To end the program after 30 seconds
bool LetterReceived = false;

bool startOfMessageReceived = false;
int direction = 0;
int drivingSpeed = 0;
std::vector<int> Received;
std::vector<int> AllReceived;

// For testing
std::string drivingDirection = "Stopped";
bool printDetectedTones = false;   // If this is set to true the terminal will display the detected tones as - "Button pressed: tone"



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

    printf("Received:");

    for (int i = 0; i < AllReceived.size(); ++i) {
        if(AllReceived[i] == 14){
            printf("\n New message: %i", AllReceived[i]);
        }else{
            printf(" %i ", AllReceived[i]);
        }
    }
    // Received: - 4 - 8 - 6 - 5 - 6 - 10 - 2 - 0 - 6 - 13 - 6 - 5 - 6 - 4 - 2 - 0 - 6 - 4 - 6 - 9 - 6 - 7
    /*
    printf("\r");
    for (int i = 1; i < Received.size(); i+=2){
        char c = Received[i-1]*16 + Received[i];
        printf("%c",c);
    }*/
    fflush(stdout);
    printf("\n");



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

        if(Received.size() == 6){
            if (Received[0] == 14 && Received[5] == 15){
                reactOnSignal();
            }else{
                printf("\n Invalid message\n");
                fflush(stdout);
                // Error handling some sort
                // Maybe just let the error handling be that the spectrum we set do not use the hex number = to * and #
            }

            AllReceived.push_back(Received[0]);
            AllReceived.push_back(Received[1]);
            AllReceived.push_back(Received[2]);
            AllReceived.push_back(Received[3]);
            AllReceived.push_back(Received[4]);
            AllReceived.push_back(Received[5]);

            Received.clear();
            startOfMessageReceived = false;
        }
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

    if(SaveSignal(rowMags,columnMags,maxRow,maxColumn)){
        return true;
    }else{
        return false;
    }

}


bool Audio::SaveSignal(std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn){
    int MinMagnitude = 200;

    if(rowMags[maxRow] > MinMagnitude && columnMags[maxColumn] > MinMagnitude && !LetterReceived && ((maxRow == 3 && maxColumn == 0) || startOfMessageReceived)){
        LetterReceived = true;

        if(maxRow == 0){
            if(maxColumn == 0){
                printDetectedSignal('1');
                Received.push_back(1);
            }else if(maxColumn == 1){
                printDetectedSignal('2');
                Received.push_back(2);
            }else if(maxColumn == 2){
                printDetectedSignal('3');
                Received.push_back(3);
            }else if(maxColumn == 3){
                printDetectedSignal('A');
                Received.push_back(10);
            }
        }else if(maxRow == 1){
            if(maxColumn == 0){
                printDetectedSignal('4');
                Received.push_back(4);
            }else if(maxColumn == 1){
                printDetectedSignal('5');
                Received.push_back(5);
            }else if(maxColumn == 2){
                printDetectedSignal('6');
                return true;
                Received.push_back(6);
            }else if(maxColumn == 3){
                printDetectedSignal('B');
                Received.push_back(11);
            }
        }else if(maxRow == 2){
            if(maxColumn == 0){
                printDetectedSignal('7');
                Received.push_back(7);
            }else if(maxColumn == 1){
                printDetectedSignal('8');
                Received.push_back(8);
            }else if(maxColumn == 2){
                printDetectedSignal('9');
                Received.push_back(9);
            }else if(maxColumn == 3){
                printDetectedSignal('C');
                Received.push_back(12);
            }
        }else if(maxRow == 3){
            if(maxColumn == 0){
                printDetectedSignal('E');
                startOfMessageReceived = true;
                Received.push_back(14);
            }else if(maxColumn == 1){
                printDetectedSignal('0');
                Received.push_back(0);
            }else if(maxColumn == 2){
                printDetectedSignal('F');
                Received.push_back(15);
            }else if(maxColumn == 3){
                printDetectedSignal('D');
                Received.push_back(13);
            }
        }
    }else if((rowMags[maxRow] < MinMagnitude || columnMags[maxColumn] < MinMagnitude) && LetterReceived){
        printDetectedSignal('N');
        LetterReceived = false;
    }

    return false;
}


void Audio::reactOnSignal(){

    drivingSpeed = Received[1]*16+Received[2];
    direction = Received[3]*16+Received[4];


    if(drivingSpeed == 0){
        printf("\r");
        printf("The robot has stopped");
        fflush(stdout);
    }else{
        printf("\n The robot is driving in direction %i at speed %i \n",direction,drivingSpeed);
        fflush(stdout);
    }

}


void Audio::printDetectedSignal(char foundTone){
    if(printDetectedTones){
        if(foundTone == 'N'){
            //printf("\r");
            //printf("No button pressed");
            //fflush(stdout);
        }else{
            printf("\r");
            printf("%c", foundTone);
            fflush(stdout);
        }
    }
}
