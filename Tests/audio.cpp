#include "audio.h"

bool endProgram = false;
bool LetterReceived = false;

PaStream* stream;

bool startOfMessageReceived = false;
int direction = 0;
int drivingSpeed = 0;
std::vector<int> Received;
double clockStartMessage = clock();
double clockEndMessage;
// double timeToSendMessage = ((136+250)*5+136)/1000;  // BUFFER 6000  // 136 = transmission time of message (ms), 250 = time between messages (ms), 5 messages have a delay the last message delay doesn't matter
double timeToSendMessage = (6*136.1)/1000;
double clockStartTone = clock();
double clockEndTone = clock();
double SamplesPerFrame = 4000; // 4000 samples per frame comes from controller program making the tones
double timeToReadTone = 0.0907;  // 102 ms is the time from the start of fade in to end of fade out



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

    // Cast our input buffer to a float pointer (since our sample format is `paFloat32`)
    float* in = (float*)inputBuffer;


    std::vector<int> tones = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
    std::vector<double> mags(tones.size());

    //int start = clock();
    // Uden threads //
    /*
    for (int i = 0; i < tones.size(); ++i) {
        calculateGoertzel(tones[i], in, mags, i);

    }*/
    ////////////////////////////////////////////////////////////////////////////

    // Threads

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

    /*
    int end = clock();

    printf("\r");
    printf("%f seconds --- 697: %f, 770: %f, 852: %f, 941: %f, 1209: %f, 1336: %f, 1477: %f, 1633: %f",((float)end - start)/CLOCKS_PER_SEC, mags[0], mags[1], mags[2], mags[3], mags[4], mags[5], mags[6], mags[7]);
    fflush(stdout);
    */
    printf("\r");
    printf("697: %f, 770: %f, 852: %f, 941: %f, 1209: %f, 1336: %f, 1477: %f, 1633: %f", mags[0], mags[1], mags[2], mags[3], mags[4], mags[5], mags[6], mags[7]);
    fflush(stdout);


    if(analyseGoertzelOutput(mags)){
        endProgram = true;
        return paAbort;

    }else{
        clockEndMessage = clock();

        if(((clockEndMessage - clockStartMessage)/CLOCKS_PER_SEC > timeToSendMessage)  && (Received.size() < 6) && (Received.size() > 0)){
            printf("\n Message timed out \n");
            fflush(stdout);
            for (int i = 0; i < Received.size(); ++i) {
                std::cout << Received[i] << " ";
            }
            std::cout << std::endl;
            std::cout <<"----------------------------------------------" << std::endl;
            Received.clear();
            startOfMessageReceived = false;
            clockStartMessage = clock();

        }else if(Received.size() == 6){
            if((Received[0] == 14 && Received[5] == 15) && ((clockEndMessage - clockStartMessage)/CLOCKS_PER_SEC < timeToSendMessage)){
                reactOnSignal();
            }else{
                printf("\n Invalid message \n");
                fflush(stdout);
            }
            for (int i = 0; i < Received.size(); ++i) {
                std::cout << Received[i] << " ";
            }
            std::cout << std::endl;
            std::cout <<"----------------------------------------------" << std::endl;
            Received.clear();
            startOfMessageReceived = false;
            clockStartMessage = clock();
        }
        return paContinue;
    }

}

void Audio::calculateGoertzel(int tone, float* in, std::vector<double>& mags, int magsIterator) {
    double pi = 3.14159265358979323846;

    double k0 = FRAMES_PER_BUFFER*tone/SAMPLE_RATE;

    double omega_I = cos(2*pi*k0/FRAMES_PER_BUFFER);
    //double omega_Q = sin(2*pi*k0/FRAMES_PER_BUFFER);
    double v1 = 0;
    double v2 = 0;
    for (int n = 0; n < FRAMES_PER_BUFFER; ++n) {
        double v  = 2*omega_I*v1 - v2 + in[n * NUM_CHANNELS];
        v2 = v1;
        v1 = v;
    }

    /*
    double y_I = omega_I * v1 - v2;
    double y_Q = omega_Q * v1;

    mags[magsIterator] = sqrt(y_I*y_I + y_Q*y_Q);
    */

    mags[magsIterator] = v1*v1 + v2*v2 - omega_I*v1*v2;
}

bool Audio::analyseGoertzelOutput(std::vector<double> mags){
    std::vector<double> rowMags = {mags[0], mags[1], mags[2], mags[3]};
    std::vector<double> columnMags = {mags[4], mags[5], mags[6], mags[7]};

    int maxRow = 0;
    int maxColumn = 0;

    for (int i = 1; i < 4; ++i) {
        if (rowMags[i] > rowMags[maxRow]) {
            maxRow = i;
        }
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
    int MinMagnitude = 18000;

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
                if(!startOfMessageReceived){
                    clockStartMessage = clock();
                    clockStartTone = clock();
                }
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
    }/*else if((rowMags[maxRow] < MinMagnitude || columnMags[maxColumn] < MinMagnitude) && LetterReceived){
        LetterReceived = false;
    }*/
    else if(LetterReceived && ((clock() - clockStartTone)/(double)CLOCKS_PER_SEC > timeToReadTone)){
        LetterReceived = false;
        std::cout << std::endl;
        std::cout << "Time passed:   " <<(clockEndTone - clockStartTone)/(double)CLOCKS_PER_SEC << "           ";
        std::cout << "Time to read:  " << timeToReadTone << "        ";
        std::cout << "Letter received: " << Received[Received.size()-1] << std::endl;
        std::cout << std::endl;
        clockStartTone = clock();

    }

    return false;
}

void Audio::reactOnSignal(){

    drivingSpeed = Received[1]*16+Received[2];
    direction = Received[3]*16+Received[4];

    // Convert back to -1.0 to 1.0
    float linearSpeed = (static_cast<float>(drivingSpeed) / 255.0) * 2.0 - 1.0;
    float angularSpeed = (static_cast<float>(direction) / 255.0) * 2.0 - 1.0;

    if(drivingSpeed == 0){
        printf("\r");
        printf("The robot has stopped");
        fflush(stdout);
    }else{
        printf("\n The robot is driving in direction %i at speed %i \n",direction,drivingSpeed);
        fflush(stdout);
    }

    // Publish to cmd_vel topic
    /*
    geometry_msgs::Twist twistMsg;
    twistMsg.linear.x = linearSpeed;
    twistMsg.angular.z = angularSpeed;
    cmdVelPub.publish(twistMsg);
    */
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
