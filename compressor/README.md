# Compressor plugin for Allolib

## Overview

This is a compressor plugin designed for plug-and-play use in Allolib sound
synthesis programs. It operates on `AudioIOData` buffers, which are produced
and consumed by `SynthGUIManager::render()` and `App::onSound()`, among others.
It is a wrapper around the
[SimpleCompressor](https://github.com/DanielRudrich/SimpleCompressor) plugin by
[Daniel Rudrich](https://github.com/DanielRudrich).

## Usage

This plugin uses a preset-based system for compressor creation. There are just a
few presets so far:
- `HARD_CLIP_0DB`: clamp the audio signal to the range [-1, 1], equivalent to
  what Allolib already does
- `STRONG_LIMITER_MAX_12DB`: some settings I came up with through
  experimentation that seem to work well for playing on a keyboard with high
  amplitudes
- `SOFT_CLIP`: a limiter with a 3dB knee and a hard limit, like `HARD_CLIP_0DB`
  but with smoothing and attack/release

To create a compressor, you need the audio buffer size (`BLOCK_SIZE`), which is
also passed to `app.configureAudio()` as the second argument. You also need the
sample rate (e.g. 48000 Hz) and whether to use lookahead (`false` since not yet
supported):

```c++
filter::CompressorPlugin<BLOCK_SIZE> compressor{
    filter::HARD_CLIP_0DB<BLOCK_SIZE>,  // Start from HARD_CLIP_0DB preset
    48000., false                       // 48000 Hz sample rate, no lookahead
}
```

You can change the compressor parameters using setter methods in `onInit()`,
which can be chained together:

```c++
compressor.setThreshold(-6.f)  // Apply compression at -6dB
          .setKnee(3.f);       // Ease into compression across 3dB
```

To apply the compressor, simply call it on the `io` object in `app::onSound()`
after rendering the synthesizer:

```
synthManager.render(io);  // Render the synthesizer to the AudioIOData output
compressor(io);           // Apply the compressor to the output
```

If you find some compressor settings that work well, feel free to add them to
the bottom of `compressor.h`! You can use `HARD_CLIP_0DB` as a reference.

The demo program `compressor_demo.cpp` compares a few different compressors
with a MIDI-capable sine synthesizer at amplitude 0.75. Press MIDI note 107 (G7)
or the "=" key on a keyboard to cycle between the compressors. The standard
output shows the maximum amplitude before compression, the maximum change in
amplitude from the compressor, and the maximum amplitude after compression, all
in dB relative to an amplitude of 1.

## Future work

This plugin currently does not support lookahead. I'm not sure exactly why the
lookahead doesn't work, but I think it may have to do with needing to delay the
input signal. Some other future improvements would be to add more presets for
useful compressor settings and to convert the signal processing code to Gamma.

## License

This plugin is licensed under the GNU General Public License v3.0. It includes
[SimpleCompressor](https://github.com/DanielRudrich/SimpleCompressor) plugin by
[Daniel Rudrich](https://github.com/DanielRudrich), whcih is also licensed under
the GNU General Public License v3.0.