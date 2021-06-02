#include <cstdio>  // for printing to stdout
#include <ostream>

#include "al/app/al_App.hpp"

#include "voices/SineEnv.h"
#include "compressor.h"

// using namespace gam;
using namespace al;

// This example shows how to use SynthVoice and SynthManagerto create an audio
// visual synthesizer. In a class that inherits from SynthVoice you will
// define the synth's voice parameters and the sound and graphic generation
// processes in the onProcess() functions.

const int BLOCK_SIZE = 128;













class MyApp : public App, public MIDIMessageHandler {
 public:
  filter::CompressorPlugin<BLOCK_SIZE> quiet{
    filter::HARD_CLIP_0DB<BLOCK_SIZE>,
    48000., false
  };

  filter::CompressorPlugin<BLOCK_SIZE> limiter{
      filter::STRONG_LIMITER_MAX_12DB<BLOCK_SIZE>,
      48000., false
  };

  filter::CompressorPlugin<BLOCK_SIZE> overdrive{
    filter::HARD_CLIP_0DB<BLOCK_SIZE>,
    48000., false
  };

  filter::CompressorPlugin<BLOCK_SIZE> clipper{
      filter::SOFT_CLIP<BLOCK_SIZE>,
      48000., false
  };


  void onInit() override {
      quiet.setMakeUpGain(-6.f);
      overdrive.setThreshold(-12.f).setMakeUpGain(-6.f);
      clipper.setThreshold(-6.f);
      setupMidi();
  }


  void onSound(AudioIOData& io) override {
    synthManager.render(io);
    //limiter(io);

    filter::CompressorPlugin<BLOCK_SIZE>* compressor;
    std::string compressor_desc;

    // Change the current compressor with = on an ASCII keyboard or G7 on a MIDI keyboard
    switch (curr_compressor) {
        case 0:
          compressor = &quiet;
          compressor_desc = "Quiet (negative make-up gain)";
          break;
        case 1:
          compressor = &limiter;
          compressor_desc = "Limiter";
          break;
        case 2:
          compressor = &overdrive;
          compressor_desc = "Overdrive (hard clipper)";
          break;
        case 3: default:
          compressor = &clipper;
          compressor_desc = "Soft clipper";
          break;
    }
    (*compressor)(io);

    //*/
    std::cout << "Compressor: " << compressor_desc << std::endl;
    std::cout << "Peak before compression: " << compressor->getDebugPrePeak() << std::endl;
    std::cout << "Max gain reduction: " << compressor->getDebugMaxCompress() << std::endl;
    std::cout << "Peak after compression: " << compressor->getDebugPostPeak() << std::endl;
    std::cout << std::endl;
    //*/
  }











  SynthGUIManager<voice::SineEnv> synthManager{"SineEnv"};
  RtMidiIn midiIn;

  int curr_compressor = 0;

  // This gets called whenever a MIDI message is received on the port
  void onMIDIMessage(const MIDIMessage& m) {
      printf("%s: ", MIDIByte::messageTypeString(m.status()));

      // Here we demonstrate how to parse common channel messages
      switch (m.type()) {
      case MIDIByte::NOTE_ON:
      if (m.noteNumber() == 103) {
          if (m.velocity() > 0.) {
              curr_compressor++;
              curr_compressor &= 3;
          }
	  } else if (m.velocity() > 0.) {
		  synthManager.voice()->setInternalParameterValue(
		      "frequency", ::pow(2.f, (m.noteNumber() - 69.f) / 12.f) * 440.f);
		  synthManager.triggerOn(m.noteNumber());
	  } else {
              synthManager.triggerOff(m.noteNumber());
	  }
          break;

      case MIDIByte::NOTE_OFF:
          if (m.noteNumber() > 0) {
              synthManager.triggerOff(m.noteNumber());
          }
          break;

      case MIDIByte::PITCH_BEND:
          printf("Value %f", m.pitchBend());
          break;

          // Control messages need to be parsed again...
      case MIDIByte::CONTROL_CHANGE:
          printf("%s ", MIDIByte::controlNumberString(m.controlNumber()));
          switch (m.controlNumber()) {
          case MIDIByte::MODULATION:
              printf("%f", m.controlValue());
              break;

          case MIDIByte::EXPRESSION:
              printf("%f", m.controlValue());
              break;
          }
          break;
      default:;
      }

      // If it's a channel message, print out channel number
      if (m.isChannelMessage()) {
          printf(" (MIDI chan %u)", m.channel() + 1);
      }

      printf("\n");

      // Print the raw byte values and time stamp
      printf("\tBytes = ");
      for (unsigned i = 0; i < 3; ++i) {
          printf("%3u ", (int)m.bytes[i]);
      }
      printf(", time = %g\n", m.timeStamp());
  }

  // This function is called right after the window is created
  // It provides a grphics context to initialize ParameterGUI
  // It's also a good place to put things that should
  // happen once at startup.
  void onCreate() override {
    navControl().active(false);  // Disable navigation via keyboard, since we
                                 // will be using keyboard for note triggering

    // Set sampling rate for Gamma objects from app's audio
    gam::sampleRate(audioIO().framesPerSecond());

    imguiInit();

    // Play example sequence. Comment this line to start from scratch
    //synthManager.synthSequencer().playSequence("synth1.synthSequence");
    synthManager.synthRecorder().verbose(true);
  }

  void onAnimate(double dt) override {
    // The GUI is prepared here
    imguiBeginFrame();
    // Draw a window that contains the synth control panel
    synthManager.drawSynthControlPanel();
    imguiEndFrame();
  }

  // The graphics callback function.
  void onDraw(Graphics& g) override {
    g.clear();
    // Render the synth's graphics
    synthManager.render(g);

    // GUI is drawn here
    imguiDraw();
  }

  // Whenever a key is pressed, this function is called
  bool onKeyDown(Keyboard const& k) override {
    if (ParameterGUI::usingKeyboard()) {  // Ignore keys if GUI is using
                                          // keyboard
      return true;
    }
    if (k.key() == '=') {
      curr_compressor++;
      curr_compressor &= 3;
      return true;
    }
    if (k.shift()) {
      // If shift pressed then keyboard sets preset
      int presetNumber = asciiToIndex(k.key());
      synthManager.recallPreset(presetNumber);
    } else {
      // Otherwise trigger note for polyphonic synth
      int midiNote = asciiToMIDI(k.key());
      if (midiNote > 0) {
        synthManager.voice()->setInternalParameterValue(
            "frequency", ::pow(2.f, (midiNote - 69.f) / 12.f) * 432.f);
        synthManager.triggerOn(midiNote);
      }
    }
    return true;
  }

  // Whenever a key is released this function is called
  bool onKeyUp(Keyboard const& k) override {
    int midiNote = asciiToMIDI(k.key());
    if (midiNote > 0) {
      synthManager.triggerOff(midiNote);
    }
    return true;
  }

  void setupMidi() {
    // Check for connected MIDI devices
    if (midiIn.getPortCount() > 0) {
      // Bind ourself to the RtMidiIn object, to have the onMidiMessage()
      // callback called whenever a MIDI message is received
      MIDIMessageHandler::bindTo(midiIn);

      // Open the last device found
      unsigned int port = midiIn.getPortCount() - 1;
      midiIn.openPort(port);
      printf("Opened port to %s\n", midiIn.getPortName(port).c_str());
    }
    else {
      printf("Error: No MIDI devices found.\n");
    }
  }

  void onExit() override { imguiShutdown(); }
};

int main() {
  // Create app instance
  MyApp app;

  // Set up audio
  app.configureAudio(48000., BLOCK_SIZE, 2, 0);

  app.start();
  return 0;
}
