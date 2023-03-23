# Embedded Systems Coursework 2 Report #

Bardia, Bakhtiar, Luca, Timur

## Base functions ##

The base functions we have implemented are all the ones which were given to us in the specifications for the coursework.

### The scankeys function ###

The scankeys function is the central function to the musical synthesizer. Its functioning is quite simple, it scans through all the rows of the KeyArray in order to find what knobs or notes are being pressed and then depending on the action taken it will take different actions. If a note or a a chord of notes is played, the keyspressed variables get updated and a sound starts to play. The knobs are used in order to regulate the volume of the notes being played, to set the octave and the type of wave being played. The waves being played can be the sawtooth wave, the triangle wave and a sinusoidal wave. The efgicency of the function is not as high as others as it sets a large amount of parameters but as the funt

### The Display update function ###

The display update function is made in order to enhance user experience and to make it more usable for first time users. Its purpose is to update the screen to help the user understand what they are doing with the device. The function displays a large amount of data for the user to be in tune with what is happening. It shows them the octave they are playing on (which is controlled by the second know), the volume (third knob) and the wave type which is being played (first knob). It also shows the user the note they are playing in order to facilitate them learning and playing if they have limited experience with the instrument. The function works like the ScanKeys function with a task scheduler however since the frequency at which it is called is lower than the Scankeys function it has a lower priority in the scheduling process.


## Advanced Features ##

### Filtering ###

The sound outputted by the keyboard was always quite synthetic and electronic, especially with the Sawtooth wave which is not common in real musical instruments. To make the sound more realistic we decided to include an FIR filter on the output. The FIR filter is used in order to attenuate certain frequencies to make the sound more precise and controlled.
