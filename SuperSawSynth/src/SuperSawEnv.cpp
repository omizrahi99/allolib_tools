#include <cstdio> // for printing to stdout

#include "Gamma/Analysis.h"
#include "Gamma/Effects.h"
#include "Gamma/Envelope.h"
#include "Gamma/Oscillator.h"
#include "Gamma/Filter.h"

#include "al/app/al_App.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/scene/al_PolySynth.hpp"
#include "al/scene/al_SynthSequencer.hpp"
#include "al/ui/al_ControlGUI.hpp"
#include "al/ui/al_Parameter.hpp"
#include "al/sound/al_Reverb.hpp"

#include <cmath>

// using namespace gam;
using namespace al;

// This example shows how to use SynthVoice and SynthManagerto create an audio
// visual synthesizer. In a class that inherits from SynthVoice you will
// define the synth's voice parameters and the sound and graphic generation
// processes in the onProcess() functions.

class SuperSaw : public SynthVoice
{
public:
  // Unit generators
  gam::Pan<> mPan;
  gam::Saw<> mOsc;
  gam::Saw<> mOsc2;
  gam::Saw<> mOsc3;
  gam::Saw<> mOsc4;
  gam::Env<3> mAmpEnv;
  // envelope follower to connect audio output to graphics
  gam::EnvFollow<> mEnvFollow;
  // Reverb effect
  Reverb<float> reverb;
  // LFO (Low Frequency Oscillator)
  gam::LFO<> lfo;
  // All Pass Filter
  gam::AllPass1<> allPass1;
  // Additional members
  Mesh mMesh;

  // Initialize voice. This function will only be called once per voice when
  // it is created. Voices will be reused if they are idle.
  void init() override
  {
    lfo.freq(1000);
    // Intialize envelope
    mAmpEnv.curve(0); // make segments lines
    mAmpEnv.levels(0, 1, 1, 0);
    mAmpEnv.sustainPoint(2); // Make point 2 sustain until a release is issued

    // We have the mesh be a sphere
    addDisc(mMesh, 1.0, 30);

    // This is a quick way to create parameters for the voice. Trigger
    // parameters are meant to be set only when the voice starts, i.e. they
    // are expected to be constant within a voice instance. (You can actually
    // change them while you are prototyping, but their changes will only be
    // stored and aplied when a note is triggered.)

    createInternalTriggerParameter("amplitude", 0.3, 0.0, 1.0);
    createInternalTriggerParameter("frequency", 60, 20, 5000);
    createInternalTriggerParameter("attackTime", 1.0, 0.01, 3.0);
    createInternalTriggerParameter("releaseTime", 3.0, 0.1, 10.0);
    createInternalTriggerParameter("pan", 0.0, -1.0, 1.0);
    createInternalTriggerParameter("mOsc1Detune", 0.0, -2.0, 2.0);
    createInternalTriggerParameter("mOsc2Detune", 0.0, -2.0, 2.0);
    createInternalTriggerParameter("mOsc3Detune", 0.0, -2.0, 2.0);
    createInternalTriggerParameter("mOsc4Detune", 0.0, -2.0, 2.0);
    createInternalTriggerParameter("lfoFreq", 1, 0.0, 20);
    createInternalTriggerParameter("lfoDepth", 1, 0.01, 1);
    createInternalTriggerParameter("filterFreq", 22000.0, 0.0, 22000.0);
    // createInternalTriggerParameter("filterBandwith", 0.0, 0.0, 8000);
    // initalize the reverb diffusion
    reverb.diffusion(0.76, 0.666, 0.707, 0.571);
  }

  // The audio processing function
  void onProcess(AudioIOData &io) override
  {
    // Set lfo cosine wave

    // Get the values from the parameters and apply them to the corresponding
    // unit generators. You could place these lines in the onTrigger() function,
    // but placing them here allows for realtime prototyping on a running
    // voice, rather than having to trigger a new voice to hear the changes.
    // Parameters will update values once per audio callback because they
    // are outside the sample processing loop.
    mOsc.freq(getInternalParameterValue("frequency") + getInternalParameterValue("mOsc1Detune"));
    mOsc2.freq(getInternalParameterValue("frequency") + getInternalParameterValue("mOsc2Detune"));
    mOsc3.freq(getInternalParameterValue("frequency") + getInternalParameterValue("mOsc3Detune"));
    mOsc4.freq(getInternalParameterValue("frequency") + getInternalParameterValue("mOsc4Detune"));
    mAmpEnv.lengths()[0] = getInternalParameterValue("attackTime");
    mAmpEnv.lengths()[2] = getInternalParameterValue("releaseTime");
    mPan.pos(getInternalParameterValue("pan"));

    // Set frequency of LFO using desired frequency from user
    lfo.freq(getInternalParameterValue("lfoFreq"));

    while (io())
    {
      float depth = getInternalParameterValue("lfoDepth");

      // Calculate LFO value after getting desired depth from user
      float lfoVal = ((lfo.cos() + 1.0) / (2 / depth)) + (1 - depth);

      // Since we want to modulate the filter frequency, we multiply the LFO value by the user's desired filter frequency
      allPass1.freq(getInternalParameterValue("filterFreq") * lfoVal);

      // Similar to above, we multiply the LFO value by the amplitude to modulate the amplitude parameter
      float s1 = (mOsc() + mOsc2() + mOsc3() + mOsc4()) * mAmpEnv() * getInternalParameterValue("amplitude") * lfoVal;
      float s2;

      mEnvFollow(s1);
      mPan(s1, s1, s2);

      // Before we output our sample values, we feed them through our low-pass filter
      io.out(0) += allPass1.low(s1);
      io.out(1) += allPass1.low(s2);
    }
    // We need to let the synth know that this voice is done
    // by calling the free(). This takes the voice out of the
    // rendering chain
    if (mAmpEnv.done() && (mEnvFollow.value() < 0.001f))
      free();
  }

  // The graphics processing function
  void onProcess(Graphics &g) override
  {
    // Get the paramter values on every video frame, to apply changes to the
    // current instance
    float frequency = getInternalParameterValue("frequency");
    float amplitude = getInternalParameterValue("amplitude");
    // Now draw
    g.pushMatrix();
    g.translate(frequency / 200 - 3, amplitude, -8);
    g.scale(1 - amplitude, amplitude, 1);
    g.color(mEnvFollow.value(), frequency / 1000, mEnvFollow.value() * 10, 0.4);
    g.draw(mMesh);
    g.popMatrix();
  }

  // The triggering functions just need to tell the envelope to start or release
  // The audio processing function checks when the envelope is done to remove
  // the voice from the processing chain.
  void onTriggerOn() override { mAmpEnv.reset(); }

  void onTriggerOff() override { mAmpEnv.release(); }
};

// We make an app.
class MyApp : public App
{
  float octaveOffset = 0;

public:
  // GUI manager for SineEnv voices
  // The name provided determines the name of the directory
  // where the presets and sequences are stored
  SynthGUIManager<SuperSaw> synthManager{"SuperSaw"};

  // This function is called right after the window is created
  // It provides a grphics context to initialize ParameterGUI
  // It's also a good place to put things that should
  // happen once at startup.

  void setOctaveOffset(float num)
  {
    this->octaveOffset = this->octaveOffset + num;
  }
  float getOctaveOffset()
  {
    return this->octaveOffset;
  }
  void onCreate() override
  {
    navControl().active(false); // Disable navigation via keyboard, since we
                                // will be using keyboard for note triggering

    // Set sampling rate for Gamma objects from app's audio
    gam::sampleRate(audioIO().framesPerSecond());

    imguiInit();

    // Play example sequence. Comment this line to start from scratch
    // synthManager.synthSequencer().playSequence("synth1.synthSequence");
    synthManager.synthRecorder().verbose(true);
  }

  // The audio callback function. Called when audio hardware requires data
  void onSound(AudioIOData &io) override
  {
    synthManager.render(io); // Render audio
  }

  void onAnimate(double dt) override
  {
    // The GUI is prepared here
    imguiBeginFrame();
    // Draw a window that contains the synth control panel
    synthManager.drawSynthControlPanel();
    imguiEndFrame();
  }

  // The graphics callback function.
  void onDraw(Graphics &g) override
  {
    g.clear();
    // Render the synth's graphics
    synthManager.render(g);

    // GUI is drawn here
    imguiDraw();
  }

  // Whenever a key is pressed, this function is called
  bool onKeyDown(Keyboard const &k) override
  {
    if (ParameterGUI::usingKeyboard())
    { // Ignore keys if GUI is using
      // keyboard
      return true;
    }
    if (k.shift())
    {
      // If shift pressed then keyboard sets preset
      int presetNumber = asciiToIndex(k.key());
      synthManager.recallPreset(presetNumber);
    }
    if (k.key() == 97)
    {
      this->setOctaveOffset(-12);
    }
    if (k.key() == 102)
    {
      this->setOctaveOffset(12);
    }
    else
    {
      // Otherwise trigger note for polyphonic synth
      int midiNote = asciiToMIDI(k.key());

      if (midiNote > 0)
      {
        synthManager.voice()->setInternalParameterValue(
            "frequency", ::pow(2.f, (midiNote - 69.f + this->getOctaveOffset()) / 12.f) * 432.f);
        synthManager.triggerOn(midiNote);
      }
    }
    return true;
  }

  // Whenever a key is released this function is called
  bool onKeyUp(Keyboard const &k) override
  {
    int midiNote = asciiToMIDI(k.key());
    if (midiNote > 0)
    {
      synthManager.triggerOff(midiNote);
    }
    return true;
  }

  void onExit() override { imguiShutdown(); }
};

int main()
{
  // Create app instance
  MyApp app;

  // Set up audio
  app.configureAudio(47000., 512, 2, 0);

  app.start();
  return 0;
}
