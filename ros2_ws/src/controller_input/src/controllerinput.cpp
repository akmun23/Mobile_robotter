#include "controllerinput.h"

controllerInput::controllerInput() : Node("controller_input")
{
    // Subscribe to the joystick messages
    _joySubscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&controllerInput::joy_callback, this, std::placeholders::_1));

    // Initialize the timer to call the play_dtmf_if_active function every 1 second
    _timer = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&controllerInput::play_dtmf_if_active, this)
        );

    makeAmplitudeFading();
}

void controllerInput::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Map joystick values to the range [0, 100]
    uint8_t linear_value = mapAxisToByte(msg->axes[1]);  // Linear velocity value
    uint8_t angular_value = mapAxisToByte(msg->axes[0]); // Angular velocity value

    // Update the active status based on whether any joystick axes are non-zero
    _joystickActive = (msg->axes[0] != 0.0f || msg->axes[1] != 0.0f);

    // Save the values to member variables for use in the timer callback
    _latestLinearValue = linear_value;
    _latestAngularValue = angular_value;
}

void controllerInput::play_dtmf_if_active() {
    // Check if the joystick values have changed, including changing to zero
    if (_latestLinearValue != _previousLinearValue || _latestAngularValue != _previousAngularValue) {
        // Debug print to show the mapped values
        std::cout << "Playing DTMF for Linear Value: " << static_cast<int>(_latestLinearValue)
                  << ", Angular Value: " << static_cast<int>(_latestAngularValue) << std::endl;

        // Play the DTMF sequence if joystick is active or has changed to zero
        if (_joystickActive || _latestLinearValue != 0 || _latestAngularValue != 0) {
            playDTMFSequence(_latestLinearValue, _latestAngularValue);
        }

        // Update the previous values after playing the sequence
        _previousLinearValue = _latestLinearValue;
        _previousAngularValue = _latestAngularValue;
    }
}

void controllerInput::makeAmplitudeFading(){
    double AudioStart = 0;
    double fadeInEnd = AudioSamplesPerFrame/6;
    double fadeOutBegin = AudioSamplesPerFrame-fadeInEnd;
    double fadeOutEnd = AudioSamplesPerFrame;
    double End = AudioSamplesPerFrame+Delay;

    for (int i = AudioStart; i < fadeInEnd; i++) {
        AmplitudeFading.push_back(i/fadeInEnd);
    }
    for (int i = fadeInEnd; i < fadeOutBegin; i++) {
        AmplitudeFading.push_back(1.0);
    }
    for (int i = fadeOutBegin; i < fadeOutEnd; i++) {
        AmplitudeFading.push_back((fadeOutEnd-i)/(fadeOutEnd-fadeOutBegin));
    }
    for (int i = fadeOutEnd; i < End; i++) {
        AmplitudeFading.push_back(0);
    }
}

int controllerInput::mapCharToIndex(char key) {

    if(key == '*'){
        key = 'e';
    }
    if(key == '#'){
        key = 'f';
    }

    if (key >= '0' && key <= '9') {
        return key - '0';
    } else if (key >= 'a' && key <= 'f') {
        return 10 + (key - 'a');
    } else {
        return -1; // Invalid key
    }
}

void controllerInput::playTone(double freq1, double freq2){
    sf::SoundBuffer buffer;
    std::vector<sf::Int16> samples;

    double amp = 0.50;

    int time = SamplesPerFrame;
    int sleep = (1000/(AudioPlayRate/SamplesPerFrame))*1000;    // AudioPlayRate/SamplesPerFrame = 7.35 which is times it is sent per sec
        // This 1000 divided by this gives the time in ms it is sent then times 1000 to get mikrosec

    for (int i = 0; i < time; i++) {
        samples.push_back(SineWave(i, freq1, amp * AmplitudeFading[i]) + SineWave(i, freq2, amp*AmplitudeFading[i]));
    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, AudioPlayRate);

    sf::Sound sound;
    sound.setBuffer(buffer);
    sound.play();
    usleep(sleep);
    samples.clear();
}

void controllerInput::playSequence(const std::string &sequence) {
    for (char key : sequence) {
        // Convert the character to the corresponding index
        int index = mapCharToIndex(tolower(key));

        if (index != -1) { // Valid DTMF key
            double freq1 = LOW_FREQ[index];
            double freq2 = HIGH_FREQ[index];
            playTone(freq1, freq2);
        } else {
            std::cout << "Invalid key: " << key << std::endl;
        }
    }
}

uint8_t controllerInput::mapAxisToByte(float axis_value) {
    return static_cast<uint8_t>(round((axis_value + 1.0) / 2.0 * 100));
}

// Function to play the DTMF sequence for the joystick input
void controllerInput::playDTMFSequence(uint8_t linear_value, uint8_t angular_value) {
    // Play the start tone (e.g., DTMF for '*')
    playTone(941, 1209);

    // Play two tones for the linear velocity
    int linear_tone1 = (linear_value >> 4) & 0x0F;  // Higher nibble (4 bits)
    int linear_tone2 = linear_value & 0x0F;          // Lower nibble (4 bits)
    playTone(LOW_FREQ[linear_tone1], HIGH_FREQ[linear_tone1]);
    playTone(LOW_FREQ[linear_tone2], HIGH_FREQ[linear_tone2]);

    // Play two tones for the angular velocity
    int angular_tone1 = (angular_value >> 4) & 0x0F; // Higher nibble (4 bits)
    int angular_tone2 = angular_value & 0x0F;        // Lower nibble (4 bits)
    playTone(LOW_FREQ[angular_tone1], HIGH_FREQ[angular_tone1]);
    playTone(LOW_FREQ[angular_tone2], HIGH_FREQ[angular_tone2]);

    // Play the stop tone (e.g., DTMF for '#')
    playTone(941, 1477);
}

double controllerInput::SineWave(int time, double freq, double amp) {
    double result = 0;
    double tpc = AudioPlayRate / freq; // ticks per cycle
    double cycles = time / tpc;
    double rad = 2 * M_PI * cycles;
    double amplitude = 32767 * amp;
    result = amplitude * sin(rad);
    return result;
}
