# Embedded Systems CW2, Synthesiser
## Introduction

The project was to build a music synthesiser. The project started on a single keyboard playing a simple sawtooth sound, but as it evolved it became a synthesiser comprised of three different boards which can be plugged into each other in order to expand the notes and to be able to create a more advanced symphony of sounds. Furthermore more advanced features were added such as different waveforms, a handshake autodetect, filtering and a Low-Frequency 

## Sections

- [Base Features](#base-features)
- [Advanced features](#advanced-features)
- [Critical timing analysis](#critical-timing-analysis)

## Base Features
The base functions we have implemented are all the ones given to us in the specifications for the coursework. These are all implemented using a task scheduler as defined in the labs. These tasks also use a delay to ensure they are not called before the set period has passed. This is a blocking assignment which ensures that all tasks can be accomplished simultaneously without creating a timing issue.

- [Scan Keys Task](#scan-keys-task)
- [Display Update Task](#display-update-task)
- [The Knob Class](#the-knob-class)
- [The CAN Bus](#the-can-bus)

### Scan keys task
The scan keys function is the central function of the musical synthesiser. Its functioning is quite simple; it scans through all the rows of the KeyArray to find what knobs or notes are being pressed, and then, depending on the action taken, it will take different actions. If a note or a chord of notes is played, the keys pressed variables get updated, and a sound starts to play. The knobs regulate the volume of the notes being played to set the octave and the type of wave being played. The waves being played can be the sawtooth wave, the triangle wave and a sinusoidal wave.

### Display Update Task
The display update function is made to enhance user experience and to make it more usable for first-time users. Its purpose is to update the screen to help users understand what they are doing with the device. The function displays a large amount of data for the user to be in tune with what is happening. It shows them the octave they are playing on (which is controlled by the second knob), the volume (third knob) and the wave type which is being played (first knob). It also shows the user the note they are playing to facilitate their learning and playing if they have limited experience with the instrument. The function works like the ScanKeys function with a task scheduler; however, since the frequency, it is called is lower than the Scankeys function, it has a lower priority in the scheduling process.

### The Knob Class
The Knob Class enables declaration of knob objects which consists of the necessary parameters and methods to retrieve the current ‘value’ of the given knob.

Each knob is decoded every time the key matrix is scanned, therefore updated in scanKeys task. As the knobs are rotated and generate the quadrature signal, the value of rotation is decoded by calling the knob value retrieval method, comparing the current state of the knob which is read from the key matrix with the previous state. The amount/direction of rotation is found by interpreting the state changes.

As expected, the knob value (a parameter of the class instance) is incremented or decremented depending on the state change, and is bounded by the upper or lower limits of the knob, specified by the developer. Additionally, in effort to improve accuracy of knob decoding when rotation is quick, or the key matrix isn’t scanned quickly enough to detect transient states, there are cases in decoding to interpret the impossible state transition. This is done by keeping track of previous increments and assuming the impossible transition was in the same direction as the last legal transition.

### The CAN Bus

## Advanced Features

- [Auto Detection using Handshake](#auto-detection-using-handshake)
- [Joystick](#joystick)
- [Filtering](#filtering)
- [Low-Frequency Oscillation (LFO)](#low-frequency-oscillation)
- [Waveform selection](#waveform-selection)
- [Polyphony](#polyphony) 

### Auto Detection using Handshake
An auto handshake detection is implemented which, on startup, correctly interprets lower, middle and upper octave modules for a two or three keyboard set-up. 

Handshaking is done in its own task and involves the exchange of handshake signals and messages on startup to determine the octave of the module. The handshake sequence is initiated from the leftmost module, and triggers sequential handshakes to the right by changing the values in the outBits array that latch onto the DFF. Doing so enables a check on the state of handshake inputs, and triggers the module to change its position, as well as send the next handshake message.

Handshake messages contain the necessary information to distinguish a handshake message from a regular note message over the CAN bus. Once handshaking is complete, further handshake messages are not sent to avoid filling the message queue with unnecessary messages. Furthermore, these handshake message payloads contain information on previous module ‘positions’, so that the current module decoding its position is assigned the next correct value.

### Joystick

Decoding the joystick drag value is done in its own task as its values are read from pins corresponding to the analogue joystick inputs x and y, rather than from the key matrix.

### Filtering

The sound outputted by the keyboard was always quite synthetic and electronic, especially with the Sawtooth wave, which is not common in real musical instruments. To make the sound more realistic, we included an FIR filter on the output. The FIR filter attenuates certain frequencies to make the sound more precise and controlled. The FIR filter is only applied on the output wave to smooth it.

### Low-Frequency Oscillation

Low-Frequency Oscillation was added to the system to create different sound effects for the Music Synthesizer. The two effects in our device are the Tremolo and the Wobble effects. They modify the output sound by either modulating the frequency or the amplitude of the output signal. The modulation is done by multiplying either the output volume or the frequency being inputted into the phase accumulator by a $5$Hz Sine wave, hence LFO. To achieve the volume modulation, the Sine wave was kept between 0 and 1 as those values seemed the most reasonable for modulating sound since they help create a wobble effect. However, for the frequency modulation, the deviation was kept small (+- 1/1000th of the base signal) to produce a subtle vibrato effect given the rate of signal compression and rarefaction.

The LFO’s frequency is designed to be selected by the user as a choice of 3 values: 5Hz, 10Hz and 15Hz. The current selection is the decoded value of knob 0, or left-most knob. Each selection iterates over the values in the LFO wave array at different step sizes in the sample buffer task, thereby changing the perceived frequency of the LFO.

### Waveform selection

Another essential part of the synthesiser is the possibility to change waveforms. Our synthesiser is capable of using four different waveforms. These are the standard sawtooth implemented as part of the labs but also square, sinusoidal and triangle waves. These enable the user to play different waveforms and simulate different output sounds for the notes pressed. These waves are controlled with knob2, and the one being used is displayed on the User interface to enhance the device's usability.
% Include a detailed description of how each wave works

### Polyphony

Polyphony is an essential feature of every music synthesiser. It enables the user to play multiple notes simultaneously. Beforehand, if two notes were played, the board would only emit the sound of the last note played, which is unrealistic for any modern music synthesiser. Therefore polyphony was added to enable the user to play chords and enable them to generate more complex and intricate sounds. 
%Functioning 

## Critical timing analysis 
