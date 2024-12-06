#include "audio.h"
#include <unistd.h>

// Variables for the listening function
PaStream* stream;

bool newMagCalculation = false;
std::vector<int> tones = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
std::vector<double> mags = {0, 0, 0, 0, 0, 0, 0, 0};
std::vector<double> HammingWindow;
std::vector<double> InputAfterHammingWindow;


Goertzel::Goertzel(double minMagnitude, double timeToReadTone) : MagnitudeAnalysis(minMagnitude, timeToReadTone) {
    createHammingWindow(FRAMES_PER_BUFFER*NUM_CHANNELS);
    HammingWindow = getHammingWindow();
    InputAfterHammingWindow.resize(FRAMES_PER_BUFFER*NUM_CHANNELS);

}



void Goertzel::checkErr(PaError err) {
    if (err != paNoError) {
        printf("PortAudio error: %s\n", Pa_GetErrorText(err));
        exit(EXIT_FAILURE);
    }
}

inline float Goertzel::min(float a, float b) {
    return a < b ? a : b;
}

void Goertzel::Init(){

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
    inputParameters.hostApiSpecificStreamInfo = nullptr;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(Pa_GetDefaultInputDevice())->defaultLowInputLatency;

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

void Goertzel::start(){

    // Begin capturing audio
    err = Pa_StartStream(stream);
    checkErr(err);

    while (true) {
        if(newMagCalculation){
            analyseMagnitudes(mags);
            checkMessageState();
            newMagCalculation = false;

        }

        if(getMessagesReceived()){

            int direction = getDirection();
            int drivingSpeed = getDrivingSpeed();
            std::cout << "Direction: " << direction << " Speed: " << drivingSpeed << std::endl;
            setMessagesReceived(false);
        }
    }
}

void Goertzel::getDevices(){
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

void Goertzel::calculateInputWithHammingWindow(const float* in) {

    for (int i = 0; i < FRAMES_PER_BUFFER*NUM_CHANNELS; ++i) {
        InputAfterHammingWindow[i] = in[i] * HammingWindow[i];
    }
}

int Goertzel::streamCallback(
    const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
    void* userData
    ) {

    // Cast our input buffer to a float pointer (since our sample format is `paFloat32`)
    float* in = (float*)inputBuffer;


    /*
    calculateInputWithHammingWindow(in);

    std::thread t0(calculateGoertzel, tones[0], InputAfterHammingWindow, std::ref(mags), 0);
    std::thread t1(calculateGoertzel, tones[1], InputAfterHammingWindow, std::ref(mags), 1);
    std::thread t2(calculateGoertzel, tones[2], InputAfterHammingWindow, std::ref(mags), 2);
    std::thread t3(calculateGoertzel, tones[3], InputAfterHammingWindow, std::ref(mags), 3);
    std::thread t4(calculateGoertzel, tones[4], InputAfterHammingWindow, std::ref(mags), 4);
    std::thread t5(calculateGoertzel, tones[5], InputAfterHammingWindow, std::ref(mags), 5);
    std::thread t6(calculateGoertzel, tones[6], InputAfterHammingWindow, std::ref(mags), 6);
    std::thread t7(calculateGoertzel, tones[7], InputAfterHammingWindow, std::ref(mags), 7);
    */
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

    newMagCalculation = true;
    return paContinue;

}
void Goertzel::calculateGoertzel(int tone, const float* in, std::vector<double>& mags, int magsIterator) {

    double k0 = FRAMES_PER_BUFFER*tone/SAMPLE_RATE;

    double omega_I = cos(2*M_PI*k0/FRAMES_PER_BUFFER);
    mags[magsIterator] = 0;

    //double omega_Q = sin(2*pi*k0/FRAMES_PER_BUFFER); Only needed for normal goertzel
    double v1 = 0;
    double v2 = 0;
    for (int n = 0; n < FRAMES_PER_BUFFER; ++n) {
        double v  = 2*omega_I*v1 - v2 + in[n * NUM_CHANNELS];
        v2 = v1;
        v1 = v;
    }

    /* Normal goertzel
    double y_I = omega_I * v1 - v2;
    double y_Q = omega_Q * v1;

    mags[magsIterator] = sqrt(y_I*y_I + y_Q*y_Q);
    */
    // Optimized goertzel
    mags[magsIterator] = v1*v1 + v2*v2 - omega_I*v1*v2;

}


void Goertzel::end(){

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






















///////////////////////////////////// NOT PART OF NORMAL PROGRAM //////////////////////////////////////
// This is the functions to make the listening program store the data in a file instead of analyzing it

void Goertzel::InitForStoringInFile(){
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
    inputParameters.hostApiSpecificStreamInfo = nullptr;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(Pa_GetDefaultInputDevice())->defaultLowInputLatency;

    // Open the PortAudio stream
    err = Pa_OpenStream(
        &stream,
        &inputParameters,
        nullptr,
        SAMPLE_RATE,
        FRAMES_PER_BUFFER,
        paNoFlag,
        streamCallbackForStoringInFile,
        spectroData
        );
    checkErr(err);

}


void Goertzel::startTimedRecording(int RecordingTime){

    // Begin capturing audio
    err = Pa_StartStream(stream);
    checkErr(err);

    usleep(RecordingTime*1000000);
}




int Goertzel::streamCallbackForStoringInFile(
    const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
    void* userData
    ) {

    // Cast our input buffer to a float pointer (since our sample format is `paFloat32`)
    float* in = (float*)inputBuffer;

    std::ofstream outputFile;

    outputFile.open("Recording1ChannelsShort.txt", std::ios_base::app); // The file is opened in append mode meaning that the data will be added to the end of the file

    for (int i = 0; i < framesPerBuffer; ++i) {
        outputFile << in[i] << std::endl;
    }



    return paContinue;

}



///////////////////////////////////////////// FFT //////////////////////////////////////////////
/// \brief Goertzel::InitForFFT

void Goertzel::InitForFFT(){

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
    inputParameters.hostApiSpecificStreamInfo = nullptr;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(Pa_GetDefaultInputDevice())->defaultLowInputLatency;

    // Open the PortAudio stream
    err = Pa_OpenStream(
        &stream,
        &inputParameters,
        nullptr,
        SAMPLE_RATE,
        FRAMES_PER_BUFFER,
        paNoFlag,
        streamCallbackFFT,
        spectroData
        );
    checkErr(err);


}

void Goertzel::startFFT(){
    // Begin capturing audio
    err = Pa_StartStream(stream);
    checkErr(err);

    while (true) {
        if(newMagCalculation){
            analyseMagnitudes(mags);
            checkMessageState();
            newMagCalculation = false;

        }
    }
}

int Goertzel::streamCallbackFFT(
    const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
    void* userData
    ){
    // Cast our input buffer to a float pointer (since our sample format is `paFloat32`)
    float* in = (float*)inputBuffer;
    FFTProcessing fft = FFTProcessing(0.5,0.140, 20);
    mags = fft.processSound(in, SAMPLE_RATE, FRAMES_PER_BUFFER);

    newMagCalculation = true;
    return paContinue;

}

