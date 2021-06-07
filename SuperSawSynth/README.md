# Super Saw Synthesizer
This is an Allolib Playground application that implements a "Super Saw Synth" using tools from the Gamma synthesis library. It creates the "Super Saw" sound by including 4 saw oscillators that can be detuned from one another. This makes each oscillator's phase slightly different from one another, which results in a nice, rich sound. It also includes a low-pass filter and an LFO that can modulate parameters. Currently, the LFO is set up to modulate the amplitude and low-pass filter frequency cutoff to get a "pumping" Super Saw synth that is heard a lot in popular EDM songs. In this application, you are also able to translate your keyboard notes down an octave by pressing the "a" key and also up and octave by pressing the "f" key.

This application should serve as an example for implementing a super saw sound, a filter from Gamma, and an LFO that can be used to modulate parameters. In this README, I will explain the usage of the synthesizer, as well as how it was implemented.

## Usage and Explanation
First, copy the `SuperSawEnv.cpp` file into `allolib_playground/tutorials/synthesis`. You can then `cd` into  `allolib_playground` run the file like so: `./run.sh /tutorials/synthesis/SuperSawEnv.cpp`.


<img src="super_saw_params.png"
     height=450px/>

**Parameters** 

`mOsc1Detune`: Amount of detuning in Hz for oscillator 1 (Similar for the rest of the oscillators)

`lfoFreq`: Frequency of the LFO curve in Hz. For example, a value of `1.0` makes the curve go through one cycle per second. 

`lfoDepth`: Depth of the LFO curve. This controls the minimum value that your modulated parameter will go to. For example, if the depth is set to `0.5`, the parameter you are modulating will go down to half of it's set value.

`filterFreq`: Cutoff frequency for the low-pass filter.

**LFO Run-Down**

An LFO is a Low-frequency oscillator. It is essentially used to modulate different musical parameters to create pulsing effects such such as tremolo and vibrato. It works by mapping the values on the LFO curve to the parameter you want to modulate, so the paramter will modulate automatically according to the LFO values. In this synth, the LFO curve is set to a Cosine function that is translated and transformed so the range of values is between 1 and the `lfoDepth` that the user set. In this synth, the LFO is mapped to modulate the `amplitude` and `filterFreq` parameters.

**Additional Feature: Octave Switching**

To go down an octave on your keyboard, simply press the `a` button your keyboard. To go up an octave, simply press the `f` button on your keyboard.

