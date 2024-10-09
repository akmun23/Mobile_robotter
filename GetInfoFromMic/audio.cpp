#include "audio.h"

std::ofstream outputFile("AudioOutput.txt");  // Open/create a file named "test.txt" for writing
int sampleNumber = 1;                         // To seperate the samples in the file

Audio::Audio() {}


/**
*@brief Method to check for errors in PortAudio functions
*
*@param PaError err
*
*@return nothing
*
*/
void Audio::checkErr(PaError err) {
    if (err != paNoError) {
        printf("PortAudio error: %s\n", Pa_GetErrorText(err));
        exit(EXIT_FAILURE);
    }
}

/**
*@brief Method to return the minimum of two float values
*
*@param float a, float b
*
*@return float
*
*/
inline float Audio::min(float a, float b) {
     return a < b ? a : b;
 }


 /**
*@brief Method to start the audio stream and record audio for a specified amount of time (RecordTimeMs)
*
*@param nothing
*
*@return nothing
*
*/

void Audio::start(){

    // Begin capturing audio
    err = Pa_StartStream(stream);
    checkErr(err);

    // Wait 30 seconds (PortAudio will continue to capture audio)
    Pa_Sleep(RecordTimeMs);

    // Stop capturing audio
    err = Pa_StopStream(stream);
    checkErr(err);

    // Close the PortAudio stream
    err = Pa_CloseStream(stream);
    checkErr(err);

    // Close the file
    outputFile.close();  // Close the file after writing

}

/**
*@brief Method to end the audio stream and free allocated resources
*
*@param nothing
*
*@return nothing
*
*/
void Audio::end(){


    // Terminate PortAudio
    err = Pa_Terminate();
    checkErr(err);

    // Free allocated resources used for FFT calculation
    fftw_free(spectroData->in);
    free(spectroData);
    printf("\n");

}
/**
*@brief Get the audio devices accessible to PortAudio and their specifications
*
*@param nothing
*
*@return nothing
*
*/
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

    // Use device 0 (for a programmatic solution for choosing a device,
    // `numDevices - 1` is typically the 'default' device
    int device = 10;
}




int Audio::streamCallbackTest(
    const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
    void* userData
    ) {

    // Cast our input buffer to a float pointer (since our sample format is `paFloat32`)
    float* in = (float*)inputBuffer;

    // We will not be modifying the output buffer. This line is a no-op.
    (void)outputBuffer;

    // Cast our user data to streamCallbackData* so we can access its struct members
    streamCallbackData* callbackData = (streamCallbackData*)userData;

    /*
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

        double y_I = v1 - omega_I*v2;
        double y_Q = omega_Q*v2;

        mags[i] = sqrt(y_I*y_I + y_Q*y_Q);

    }

    /*
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



    analyseGoertzelOutput(mags);


    return 0;
}


void Audio::printData(){

    // Initialize PortAudio
    err = Pa_Initialize();
    checkErr(err);

    // Allocate and define the callback data used to calculate/display the spectrogram
    spectroData = (streamCallbackData*)malloc(sizeof(streamCallbackData));
    spectroData->in = (double*)malloc(sizeof(double) * FRAMES_PER_BUFFER);
    if (spectroData->in == NULL) {
        printf("Could not allocate spectro data\n");
        exit(EXIT_FAILURE);
    }

    // Get and display the number of audio devices accessible to PortAudio


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
        streamCallbackTest,
        spectroData
        );
    checkErr(err);
}

void Audio::analyseGoertzelOutput(std::vector<double> mags){
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

    if(rowMags[maxRow] > 200 && columnMags[maxColumn] > 200){

        if(maxRow < 3 && maxColumn < 3){
            printf("\r");
            printf("Button pressed: %d", maxRow*3 + maxColumn + 1);
            fflush(stdout);
        }else if(maxRow == 3){
            if(maxColumn == 0){
                printf("\r");
                printf("Button pressed: *");
                fflush(stdout);
            }else if(maxColumn == 1){
                printf("\r");
                printf("Button pressed: 0");
                fflush(stdout);
            }else if(maxColumn == 2){
                printf("\r");
                printf("Button pressed: #");
                fflush(stdout);
            }else{
                printf("\r");
                printf("Button pressed: D");
                fflush(stdout);
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
    }else{
        printf("\r");
        printf("No button pressed");
        fflush(stdout);
    }



}
