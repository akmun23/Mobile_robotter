#include "audio.h"

bool endProgram = false;                      // To end the program after 30 seconds
bool LetterReceived = false;

bool startOfMessageReceived = false;
int direction = 0;
int drivingSpeed = 0;
std::vector<int> Received;
DtmfToCmdVelNode* Audio::dtmfNode = nullptr;

void Audio::checkErr(PaError err) {
    if (err != paNoError) {
        printf("PortAudio error: %s\n", Pa_GetErrorText(err));
        exit(EXIT_FAILURE);
    }
}

inline float Audio::min(float a, float b) {
     return a < b ? a : b;
}

void Audio::Init(){

    // Initialize PortAudio
    err = Pa_Initialize();
    checkErr(err);

    // Allocate and define the callback data
    spectroData = (streamCallbackData*)malloc(sizeof(streamCallbackData));
    spectroData->in = (double*)malloc(sizeof(double) * FRAMES_PER_BUFFER);
    if (spectroData->in == nullptr) {
        printf("Could not allocate spectro data\n");
        exit(EXIT_FAILURE);
    }

    // Define stream capture specifications
    memset(&inputParameters, 0, sizeof(inputParameters));
    inputParameters.channelCount = NUM_CHANNELS;
    inputParameters.device = 0;
    inputParameters.hostApiSpecificStreamInfo = nullptr;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(0)->defaultLowInputLatency;

    // Open the PortAudio stream
    err = Pa_OpenStream(
        &stream,
        &inputParameters,
        nullptr,
        SAMPLE_RATE,
        FRAMES_PER_BUFFER,
        paNoFlag,
        streamCallback,
        spectroData
        );
    checkErr(err);
}

void Audio::start(){

    // Begin capturing audio
    err = Pa_StartStream(stream);
    checkErr(err);

    while (!endProgram) {

    }
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
    
    (void)outputBuffer;  // Unused variable
    (void)timeInfo;      // Unused variable
    (void)statusFlags;   // Unused variable

    // Cast our input buffer to a float pointer (since our sample format is `paFloat32`)
    float* in = (float*)inputBuffer;

    std::vector<int> tones = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
    std::vector<double> mags(tones.size());

	std::thread t0(calculateGoertzel, tones[0], in, std::ref(mags), 0);
    std::thread t1(calculateGoertzel, tones[1], in, std::ref(mags), 1);
    std::thread t2(calculateGoertzel, tones[2], in, std::ref(mags), 2);
    std::thread t3(calculateGoertzel, tones[3], in, std::ref(mags), 3);
    std::thread t4(calculateGoertzel, tones[4], in, std::ref(mags), 4);
    std::thread t5(calculateGoertzel, tones[5], in, std::ref(mags), 5);
    std::thread t6(calculateGoertzel, tones[6], in, std::ref(mags), 6);
    std::thread t7(calculateGoertzel, tones[7], in, std::ref(mags), 7);

    t0.join();
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();
    t7.join();

    
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
    fflush(stdout);

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
                dtmfNode->publishTwistMessage(128, 128);
            }

            Received.clear();
            startOfMessageReceived = false;
        }
        return paContinue;
    }

}

void Audio::calculateGoertzel(int tone, float* in, std::vector<double>& mags, int magsIterator) {
    double twoPi = 2*3.14159265358979323846;

    double k0 = FRAMES_PER_BUFFER*tone/SAMPLE_RATE;

    double omega_I = cos(twoPi*k0/FRAMES_PER_BUFFER);
    double omega_Q = sin(twoPi*k0/FRAMES_PER_BUFFER);
    double v1 = 0;
    double v2 = 0;
    for (int n = 0; n < FRAMES_PER_BUFFER; ++n) {
        double v  = 2*omega_I*v1 - v2 + in[n * NUM_CHANNELS];
        v2 = v1;
        v1 = v;
    }


    double y_I = v1 - omega_I * v2;
    double y_Q = omega_Q * v2;

    mags[magsIterator] = sqrt(y_I*y_I + y_Q*y_Q);
}

bool Audio::analyseGoertzelOutput(std::vector<double> mags){
    std::vector<double> rowMags = {mags[0], mags[1], mags[2], mags[3]};
    std::vector<double> columnMags = {mags[4], mags[5], mags[6], mags[7]};

    int maxRow = 0;
    int maxColumn = 0;

    for (size_t i = 1; i < rowMags.size(); ++i) {
        if (rowMags[i] > rowMags[maxRow]) {
            maxRow = i;
        }
    }

    for (size_t i = 1; i < columnMags.size(); ++i) {
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
    int MinMagnitude = 2;

    if(rowMags[maxRow] > MinMagnitude && columnMags[maxColumn] > MinMagnitude && !LetterReceived && ((maxRow == 3 && maxColumn == 0) || startOfMessageReceived)){
        LetterReceived = true;

        if(maxRow == 0){
            if(maxColumn == 0){
                Received.push_back(1);
            }else if(maxColumn == 1){
                Received.push_back(2);
            }else if(maxColumn == 2){
                Received.push_back(3);
            }else if(maxColumn == 3){
                Received.push_back(10);
            }
        }else if(maxRow == 1){
            if(maxColumn == 0){
                Received.push_back(4);
            }else if(maxColumn == 1){
                Received.push_back(5);
            }else if(maxColumn == 2){
                Received.push_back(6);
            }else if(maxColumn == 3){
                Received.push_back(11);
            }
        }else if(maxRow == 2){
            if(maxColumn == 0){
                Received.push_back(7);
            }else if(maxColumn == 1){
                Received.push_back(8);
            }else if(maxColumn == 2){
                Received.push_back(9);
            }else if(maxColumn == 3){
                Received.push_back(12);
            }
        }else if(maxRow == 3){
            if(maxColumn == 0){
                startOfMessageReceived = true;
                Received.push_back(14);
            }else if(maxColumn == 1){
                Received.push_back(0);
            }else if(maxColumn == 2){
                Received.push_back(15);
            }else if(maxColumn == 3){
                Received.push_back(13);
            }
        }
    }else if((rowMags[maxRow] < MinMagnitude || columnMags[maxColumn] < MinMagnitude) && LetterReceived){
        LetterReceived = false;
    }

    return false;
}

void Audio::reactOnSignal(){

    drivingSpeed = (Received[1]*16+Received[2]);
    direction = (Received[3]*16+Received[4]);

    if(drivingSpeed == 128 && direction == 128){
        printf("\r");
        printf("The robot has stopped");
        fflush(stdout);
    }else{
        printf("\n The robot is driving at speed %i in direction %i \n",drivingSpeed, direction);
        fflush(stdout);
    }
    
    // Use the global dtmfNode pointer to call the publishTwistMessage function
    if (dtmfNode) {
        dtmfNode->publishTwistMessage(drivingSpeed, direction);
    }
    
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
}
