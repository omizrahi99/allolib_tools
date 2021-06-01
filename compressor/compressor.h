#pragma once

#include "al/io/al_AudioIOData.hpp"
#include <iostream>

namespace al {

namespace filter {

// BEGIN FILE SimpleCompressor/src/GainReductionComputer.h

/*
 This file is part of the SimpleCompressor project.
 https://github.com/DanielRudrich/SimpleCompressor
 Copyright (c) 2019 Daniel Rudrich

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, version 3.

 This program is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <vector>
#include <limits>
#include <cmath>
#include <atomic>

/**
 This class acts as the side-chain path of a dynamic range compressor. It processes a given side-chain signal and computes the gain reduction samples depending on the parameters threshold, knee, attack-time, release-time, ratio, and make-up gain.
 */
class GainReductionComputer
{
public:
    GainReductionComputer();
    ~GainReductionComputer() {}

    // ======================================================================
    /**
     Sets the attack time of the compressor in seconds.
     */
    void setAttackTime (const float attackTimeInSeconds);

    /**
     Sets the release time of the compressorin seconds
     */
    void setReleaseTime (const float releaseTimeInSeconds);

    /**
     Sets the knee-width in decibels.
     */
    void setKnee (const float kneeInDecibels);
    const float getKnee() { return knee; }

    /**
     Sets the threshold above which the compressor will start to compress the signal.
     */
    void setThreshold (const float thresholdInDecibels);
    const float getThreshold() { return threshold; }

    /**
     Sets the make-up-gain of the compressor in decibels.
     */
    void setMakeUpGain (const float makeUpGainInDecibels);
    const float getMakeUpGain()  { return makeUpGain; }

    /**
     Sets the ratio of input-output signal above threshold. Set to 1 for no compression, up to infinity for a brickwall limiter.
     */
    void setRatio (const float ratio);

    // ======================================================================
    /**
     Computes the static output levels for an array of input levels in decibels. Useful for visualization of the compressor's characteristic. Will contain make-up gain.
     */
    void getCharacteristic (float* inputLevelsInDecibels, float* destination, const int numSamples);

    /**
     Computes the static output levels for a given input level in decibels. Useful for visualization of the compressor's characteristic. Will contain make-up gain.
     */
    float getCharacteristicSample (const float inputLevelInDecibels);

    // ======================================================================
    /**
     Prepares the compressor with sampleRate and expected blockSize. Make sure you call this before you do any processing!
     */
    void prepare (const double sampleRate);

    /**
     Resets the internal state of the compressor.
     */
    void reset() { state = 0.0f; }

    /**
     Computes the gain reduction for a given side-chain signal. The values will be in decibels and will NOT contain the make-up gain.
     */
    void computeGainInDecibelsFromSidechainSignal (const float* sideChainSignal, float* destination, const int numSamples);

    /**
     Computes the linear gain including make-up gain for a given side-chain signal. The gain written to the destination can be directly applied to the signals which should be compressed.
     */
    void computeLinearGainFromSidechainSignal (const float* sideChainSignal, float* destination, const int numSamples);

    const float getMaxInputLevelInDecibels() { return maxInputLevel; }
    const float getMaxGainReductionInDecibels() { return maxGainReduction; }

private:
    inline const float timeToGain (const float timeInSeconds);
    inline const float applyCharacteristicToOverShoot (const float overShootInDecibels);

    double sampleRate;

    // parameters
    float knee, kneeHalf;
    float threshold;
    float attackTime;
    float releaseTime;
    float slope;
    float makeUpGain;

    std::atomic<float> maxInputLevel {-std::numeric_limits<float>::infinity()};
    std::atomic<float> maxGainReduction {0};

    //state variable
    float state;

    float alphaAttack;
    float alphaRelease;
};

// END FILE SimpleCompressor/src/GainReductionComputer.h











// BEGIN FILE SimpleCompressor/src/GainReductionComputer.cpp

/*
 This file is part of the SimpleCompressor project.
 https://github.com/DanielRudrich/SimpleCompressor
 Copyright (c) 2019 Daniel Rudrich

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, version 3.

 This program is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

//#include "GainReductionComputer.h"

GainReductionComputer::GainReductionComputer()
{
    sampleRate = 0.0f;

    threshold = -10.0f;
    knee = 0.0f;
    kneeHalf = 0.0f;
    attackTime = 0.01f;
    releaseTime = 0.15f;
    setRatio (2); // 2 : 1
    makeUpGain = 0.0f;
    reset();
}

void GainReductionComputer::prepare (const double newSampleRate)
{
    sampleRate = newSampleRate;

    alphaAttack = 1.0f - timeToGain (attackTime);
    alphaRelease = 1.0f - timeToGain (releaseTime);
}

void GainReductionComputer::setAttackTime (const float attackTimeInSeconds)
{
    attackTime = attackTimeInSeconds;
    alphaAttack = 1.0f - timeToGain (attackTime);
}

void GainReductionComputer::setReleaseTime (const float releaseTimeInSeconds)
{
    releaseTime = releaseTimeInSeconds;
    alphaRelease = 1.0f - timeToGain (releaseTime);
}

const float GainReductionComputer::timeToGain (const float timeInSeconds)
{
    return std::exp (-1.0f / (static_cast<float> (sampleRate) * timeInSeconds));
}

void GainReductionComputer::setKnee (const float kneeInDecibels)
{
    knee = kneeInDecibels;
    kneeHalf = knee / 2.0f;
}

void GainReductionComputer::setThreshold (const float thresholdInDecibels)
{
    threshold = thresholdInDecibels;
}

void GainReductionComputer::setMakeUpGain (const float makeUpGainInDecibels)
{
    makeUpGain = makeUpGainInDecibels;
}

void GainReductionComputer::setRatio (const float ratio)
{
    slope = 1.0f / ratio - 1.0f;
}


inline const float GainReductionComputer::applyCharacteristicToOverShoot (const float overShootInDecibels)
{
    if (overShootInDecibels <= -kneeHalf)
        return 0.0f;
    else if (overShootInDecibels > -kneeHalf && overShootInDecibels <= kneeHalf)
        return 0.5f * slope * (overShootInDecibels + kneeHalf) * (overShootInDecibels + kneeHalf) / knee;
    else
        return slope * overShootInDecibels;
}

void GainReductionComputer::computeGainInDecibelsFromSidechainSignal (const float* sideChainSignal, float* destination, const int numSamples)
{
    maxInputLevel = -std::numeric_limits<float>::infinity();
    maxGainReduction = 0.0f;

    for (int i = 0; i < numSamples; ++i)
    {
        // convert sample to decibels
        const float levelInDecibels = 20.0f * std::log10 (std::abs(sideChainSignal[i]));
        
        if (levelInDecibels > maxInputLevel)
            maxInputLevel = levelInDecibels;

        // calculate overshoot and apply knee and ratio
        const float overShoot = levelInDecibels - threshold;
        const float gainReduction = applyCharacteristicToOverShoot (overShoot);

        // apply ballistics
        const float diff = gainReduction - state;
        if (diff < 0.0f) // wanted gain reduction is below state -> attack phase
            state += alphaAttack * diff;
        else // release phase
            state += alphaRelease * diff;

        // write back gain reduction
        destination[i] = state;

        if (state < maxGainReduction)
            maxGainReduction = state;
    }
}

void GainReductionComputer::computeLinearGainFromSidechainSignal (const float* sideChainSignal, float* destination, const int numSamples)
{
    computeGainInDecibelsFromSidechainSignal (sideChainSignal, destination, numSamples);
    for (int i = 0; i < numSamples; ++i)
        destination[i] = std::pow (10.0f, 0.05f * (destination[i] + makeUpGain));
}


void GainReductionComputer::getCharacteristic (float* inputLevelsInDecibels, float* dest, const int numSamples)
{
    for (int i = 0; i < numSamples; ++i)
        dest[i] = getCharacteristicSample (inputLevelsInDecibels[i]);
}

float GainReductionComputer::getCharacteristicSample (const float inputLevelInDecibels)
{
    float overShoot = inputLevelInDecibels - threshold;
    overShoot = applyCharacteristicToOverShoot (overShoot);
    return overShoot + inputLevelInDecibels + makeUpGain;
}

// END FILE SimpleCompressor/src/GainReductionComputer.cpp











// BEGIN FILE SimpleCompressor/src/LookAheadGainReduction.h

/*
 This file is part of the SimpleCompressor project.
 https://github.com/DanielRudrich/SimpleCompressor
 Copyright (c) 2019 Daniel Rudrich

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, version 3.

 This program is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once
#include <vector>

/** This class acts as a delay line for gain-reduction samples, which additionally fades in high gain-reduction values in order to avoid distortion when limiting an audio signal.
 */
class LookAheadGainReduction
{
public:
    LookAheadGainReduction() : sampleRate (0.0) {}
    ~LookAheadGainReduction() {}

    void setDelayTime (float delayTimeInSeconds);

    const int getDelayInSamples() { return delayInSamples; }

    /** Prepares the processor so it can resize the buffers depending on samplerate and the expected buffersize.
     */
    void prepare (const double sampleRate, const int blockSize);

    /** Writes gain-reduction samples into the delay-line. Make sure you call process() afterwards, and read the same amount of samples with the readSamples method. Make also sure the pushed samples are decibel values.
     */
    void pushSamples (const float* src, const int numSamples);

    /** Processes the data within the delay line, i.e. fades-in high gain-reduction, in order to reduce distortions.
     */
    void process();

    /** Reads smoothed gain-reduction samples back to the destination. Make sure you read as many samples as you've pushed before!
     */
    void readSamples (float* dest, const int numSamples);


private:
    /** A little helper-function which calulcates how many samples we should process in a first step before we have to wrap around, as our buffer is a ring-buffer.
     */
    inline void getProcessPositions (int startIndex, int numSamples, int& blockSize1, int& blockSize2);

    inline void getWritePositions (int numSamples, int& startIndex, int& blockSize1, int& blockSize2);

    inline void getReadPositions (int numSamples, int& startIndex, int& blockSize1, int& blockSize2);


private:
    //==============================================================================
    double sampleRate;
    int blockSize;

    float delay;
    int delayInSamples = 0;
    int writePosition = 0;
    int lastPushedSamples = 0;
    std::vector<float> buffer;
};

// END FILE SimpleCompressor/src/LookAheadGainReduction.h











// BEGIN FILE SimpleCompressor/src/LookAheadGainReduction.cpp

/*
 This file is part of the SimpleCompressor project.
 https://github.com/DanielRudrich/SimpleCompressor
 Copyright (c) 2019 Daniel Rudrich

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, version 3.

 This program is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


//#include "LookAheadGainReduction.h"
#include <cmath>
#include <algorithm>

void LookAheadGainReduction::setDelayTime (float delayTimeInSeconds)
{
    if (delayTimeInSeconds <= 0.0f)
        delay = 0.0f;
    else
        delay = delayTimeInSeconds;

    if (sampleRate != 0.0)
        prepare (sampleRate, blockSize);
}

void LookAheadGainReduction::prepare (const double newSampleRate, const int newBlockSize)
{
    sampleRate = newSampleRate;
    blockSize = newBlockSize;

    delayInSamples = static_cast<int> (delay * sampleRate);

    buffer.resize (blockSize + delayInSamples);
    std::fill (buffer.begin(), buffer.end(), 0.0f);
    writePosition = 0;
}

void LookAheadGainReduction::pushSamples (const float* src, const int numSamples)
{
    int startIndex, blockSize1, blockSize2;

    // write in delay line
    getWritePositions (numSamples, startIndex, blockSize1, blockSize2);

    for (int i = 0; i < blockSize1; ++i)
        buffer[startIndex + i] = src[i];

    if (blockSize2 > 0)
        for (int i = 0; i < blockSize2; ++i)
            buffer[i] = src[blockSize1 + i];

    writePosition += numSamples;
    writePosition = writePosition % buffer.size();

    lastPushedSamples = numSamples;
}

void LookAheadGainReduction::process()
{
    /** The basic idea here is to look for high gain-reduction values in the signal, and apply a fade which starts exactly  `delayInSamples` many samples before that value appears. Depending on the value itself, the slope of the fade will vary.

     Some things to note:
        - as the samples are gain-reduction values in decibel, we actually look for negative peaks or local minima.
        - it's easier for us to time-reverse our search, so we start with the last sample
        - once we find a minimum, we calculate the slope, which will be called `step`
        - with that slope, we can calculate the next value of our fade-in `nextGainReductionValue`
        - once a value in the buffer is below our fade-in value, we found a new minimum, which might not be as deep as the previous one, but as it comes in earlier, it needs more attention, so we update our fade-in slope
        - our buffer is a ring-buffer which makes things a little bit messy
     */


    // As we don't know any samples of the future, yet, we assume we don't have to apply a fade-in right now, and initialize both `nextGainReductionValue` and step (slope of the fade-in) with zero.
    float nextGainReductionValue = 0.0f;
    float step = 0.0f;


    // Get the position of the last sample in the buffer, which is the sample right before our new write position.
    int index = writePosition - 1;
    if (index < 0) // in case it's negative...
        index += static_cast<int> (buffer.size()); // ... add the buffersize so we wrap around.

    // == FIRST STEP: Process all recently pushed samples.

    // We want to process all `lastPushedSamples` many samples, so let's find out, how many we can process in a first run before we have to wrap around our `index` variable (ring-buffer).
    int size1, size2;
    getProcessPositions (index, lastPushedSamples, size1, size2);

    // first run
    for (int i = 0; i < size1; ++i)
    {
        const float smpl = buffer[index];

        if (smpl > nextGainReductionValue) // in case the sample is above our ramp...
        {
            buffer[index] = nextGainReductionValue; // ... replace it with the current ramp value
            nextGainReductionValue += step; // and update the next ramp value
        }
        else // otherwise... (new peak)
        {
            step = - smpl / delayInSamples; // calculate the new slope
            nextGainReductionValue = smpl + step; // and also the new ramp value
        }
        --index;
    }

    // second run
    if (size2 > 0) // in case we have some samples left for the second run
    {
        index = static_cast<int> (buffer.size()) - 1; // wrap around: start from the last sample of the buffer

        // exactly the same procedure as before... I guess I could have written that better...
        for (int i = 0; i < size2; ++i)
        {
            const float smpl = buffer[index];

            if (smpl > nextGainReductionValue)
            {
                buffer[index] = nextGainReductionValue;
                nextGainReductionValue += step;
            }
            else
            {
                step = - smpl / delayInSamples;
                nextGainReductionValue = smpl + step;
            }
            --index;
        }
    }

    /*
     At this point, we have processed all the new gain-reduction values. For this, we actually don't need a delay/lookahead at all.
     !! However, we are not finished, yet !!
     What if the first pushed sample has such a high gain-reduction value, that itself needs a fade-in? So we have to apply a gain-ramp even further into the past. And that is exactly the reason why we need lookahead, why we need to buffer our signal for a short amount of time: so we can apply that gain ramp for the first handful of gain-reduction samples.
     */

    if (index < 0) // it's possible the index is exactly -1
        index = static_cast<int> (buffer.size()) - 1; // so let's take care of that

    /*
     This time we only need to check `delayInSamples` many samples.
     And there's another cool thing!
        We know that the samples have been processed already, so in case one of the samples is below our ramp value, that's the new minimum, which has been faded-in already! So what we do is hit the break, and call it a day!
     */
    getProcessPositions (index, delayInSamples, size1, size2);
    bool breakWasUsed = false;

    // first run
    for (int i = 0; i < size1; ++i) // we iterate over the first size1 samples
    {
        const float smpl = buffer[index];

        if (smpl > nextGainReductionValue) // in case the sample is above our ramp...
        {
            buffer[index] = nextGainReductionValue; // ... replace it with the current ramp value
            nextGainReductionValue += step; // and update the next ramp value
        }
        else // otherwise... JACKPOT! Nothing left to do here!
        {
            breakWasUsed = true; // let the guys know we are finished
            break;
        }
        --index;
    }

    // second run
    if (! breakWasUsed && size2 > 0) // is there still some work to do?
    {
        index = static_cast<int> (buffer.size()) - 1; // wrap around (ring-buffer)
        for (int i = 0; i < size2; ++i)
        {
            const float smpl = buffer[index];

            // same as before
            if (smpl > nextGainReductionValue) // in case the sample is above our ramp...
            {
                buffer[index] = nextGainReductionValue; // ... replace it with the current ramp value
                nextGainReductionValue += step; // and update the next ramp value
            }
            else // otherwise... already processed -> byebye!
                break;
            --index;
        }
    }
}


void LookAheadGainReduction::readSamples (float* dest, int numSamples)
{
    int startIndex, blockSize1, blockSize2;

    // read from delay line
    getReadPositions (numSamples, startIndex, blockSize1, blockSize2);

    for (int i = 0; i < blockSize1; ++i)
        dest[i] = buffer[startIndex + i];

    if (blockSize2 > 0)
        for (int i = 0; i < blockSize2; ++i)
            dest[blockSize1 + i] = buffer[i];
}


inline void LookAheadGainReduction::getProcessPositions (int startIndex, int numSamples, int& blockSize1, int& blockSize2)
{
    if (numSamples <= 0)
    {
        blockSize1 = 0;
        blockSize2 = 0;
    }
    else
    {
        blockSize1 = std::min (startIndex + 1, numSamples);
        numSamples -= blockSize1;
        blockSize2 = numSamples <= 0 ? 0 : numSamples;
    }
}

inline void LookAheadGainReduction::getWritePositions (int numSamples, int& startIndex, int& blockSize1, int& blockSize2)
{
    const int L = static_cast<int> (buffer.size());
    int pos = writePosition;

    if (pos < 0)
        pos = pos + L;
    pos = pos % L;

    if (numSamples <= 0)
    {
        startIndex = 0;
        blockSize1 = 0;
        blockSize2 = 0;
    }
    else
    {
        startIndex = pos;
        blockSize1 = std::min (L - pos, numSamples);
        numSamples -= blockSize1;
        blockSize2 = numSamples <= 0 ? 0 : numSamples;
    }
}

inline void LookAheadGainReduction::getReadPositions (int numSamples, int& startIndex, int& blockSize1, int& blockSize2)
{
    const int L = static_cast<int> (buffer.size());
    int pos = writePosition - lastPushedSamples - delayInSamples;

    if (pos < 0)
        pos = pos + L;
    pos = pos % L;

    if (numSamples <= 0)
    {
        startIndex = 0;
        blockSize1 = 0;
        blockSize2 = 0;
    }
    else
    {
        startIndex = pos;
        blockSize1 = std::min (L - pos, numSamples);
        numSamples -= blockSize1;
        blockSize2 = numSamples <= 0 ? 0 : numSamples;
    }
}

// END FILE SimpleCompressor/src/LookAheadGainReduction.cpp











float linearToDecibels(float linear) {
  return 20.f * std::log10(std::abs(linear));
}

float decibelsToLinear(float decibels) {
  return std::pow(10.f, decibels / 20.f);
}


template <int block_size>
class CompressorPlugin {
  private:
  GainReductionComputer gain;
  LookAheadGainReduction lookahead;

  float sidechain_buf[block_size];
  float gain_buf[block_size];
  float look_buf[block_size];

  double sampleRate;
  float knee;
  float threshold;
  float attackTime;
  float releaseTime;
  float ratio;
  float makeUpGain;

  float debugPrePeak;
  float debugMaxCompress;
  float debugPostPeak;

  bool _useLookAhead;

  public:
  CompressorPlugin(
      double sampleRateHz, float kneeDb, float thresholdDb,
      float attackTimeSeconds, float releaseTimeSeconds, float ratioDbDb,
      float makeUpGainDb, bool useLookAhead
           ) : 
      sampleRate(sampleRateHz), knee(kneeDb), threshold(thresholdDb),
      attackTime(attackTimeSeconds), releaseTime(releaseTimeSeconds),
      ratio(ratioDbDb), makeUpGain(makeUpGainDb), _useLookAhead(useLookAhead)
      {
    gain.prepare       (sampleRate);
    gain.setKnee       (knee);
    gain.setThreshold  (threshold);
    gain.setAttackTime (attackTime);
    gain.setReleaseTime(releaseTime);
    gain.setRatio      (ratio);
    gain.setMakeUpGain (makeUpGain);

    lookahead.prepare(48000., 2 * block_size);
    lookahead.setDelayTime(0.005f);

    resetDebugStats();
  }

  CompressorPlugin(
      const CompressorPlugin& other, double sampleRate,
      bool useLookAhead) {
    new (this) CompressorPlugin(
      sampleRate, other.knee, other.threshold, other.attackTime,
      other.releaseTime, other.ratio, other.makeUpGain, useLookAhead
    );
  }

  CompressorPlugin(const CompressorPlugin& other) = delete;

  //CompressorPlugin(const CompressorPlugin&& other) = delete;

  //CompressorPlugin& operator=(const CompressorPlugin& other) = delete;

  //if I add a typical copy constructor that calls my second constructor, it 

  CompressorPlugin<block_size>& setSampleRate(const double sampleRateHz) {
    sampleRate = sampleRateHz;
    gain.prepare(sampleRate);
    return *this;
  }

  CompressorPlugin<block_size>& setKnee(const float kneeDb) {
    knee = kneeDb;
    gain.setKnee(knee);
    return *this;
  }

  CompressorPlugin<block_size>& setThreshold(const float thresholdDb) {
    threshold = thresholdDb;
    gain.setThreshold(threshold);
    return *this;
  }

  CompressorPlugin<block_size>& setAttackTime(const float attackTimeSeconds) {
    attackTime = attackTimeSeconds;
    gain.setAttackTime(attackTime);
    return *this;
  }

  CompressorPlugin<block_size>& setReleaseTime(const float releaseTimeSeconds) {
    releaseTime = releaseTimeSeconds;
    gain.setReleaseTime(releaseTime);
    return *this;
  }

  CompressorPlugin<block_size>& setRatio(const float ratioDbDb) {
    ratio = ratioDbDb;
    gain.setRatio(ratio);
    return *this;
  }

  CompressorPlugin<block_size>& setMakeUpGain(const float makeUpGainDb) {
    makeUpGain = makeUpGainDb;
    gain.setMakeUpGain(makeUpGain);
    return *this;
  }

  AudioIOData& operator()(AudioIOData& io) {
    // TODO get look-ahead working
    io.frame(0);
    for (int i = 0; io() && i < block_size; i++) {
      sidechain_buf[i] = std::max(std::abs(io.out(0)), std::abs(io.out(1)));
    }

    if (_useLookAhead) {
      gain.computeGainInDecibelsFromSidechainSignal(sidechain_buf, gain_buf, block_size);
    } else {
      gain.computeLinearGainFromSidechainSignal(sidechain_buf, gain_buf, block_size);
    }


    if (_useLookAhead) {
      lookahead.pushSamples(gain_buf, block_size);
      lookahead.process();
      lookahead.readSamples(look_buf, block_size);

      for (int i = 0; i < block_size; i++) {
        gain_buf[i] = decibelsToLinear(look_buf[i]);
      }
    }


    debugPrePeak = -std::numeric_limits<float>::infinity();
    debugMaxCompress = 0;
    debugPostPeak = -std::numeric_limits<float>::infinity();

    io.frame(0);
    for (int i = 0; io() && i < block_size; i++) {
      debugPrePeak = std::max(debugPrePeak, std::abs(io.out(0)));
      debugPrePeak = std::max(debugPrePeak, std::abs(io.out(1)));

      debugMaxCompress = std::min(debugMaxCompress, gain_buf[i]);

      io.out(0) *= gain_buf[i];
      io.out(1) *= gain_buf[i];

      debugPostPeak = std::max(debugPostPeak, std::abs(io.out(0)));
      debugPostPeak = std::max(debugPostPeak, std::abs(io.out(1)));
    }

    return io;
  }

  void resetDebugStats() {
      debugPrePeak = -std::numeric_limits<float>::infinity();
      debugMaxCompress = 0;
      debugPostPeak = -std::numeric_limits<float>::infinity();
  }

  float getDebugPrePeak() {
      /* Get the peak of the latest audio sample buffer before compression. */
      return debugPrePeak;
  }

  float getDebugMaxCompress() {
      /* Get the largest gain reduction from compression for the latest audio sample buffer. */
      return debugMaxCompress;
  }

  float getDebugPostPeak() {
      /* Get the peak of the latest audio sample buffer after compression. */
      return debugPostPeak;
  }
};


// A hard clipper on the output to keep it from exceeding 0dB. This shouldn't
// have any effect when applied directly to the output since Allolib already
// clamps the amplitudes. However, it can be used as a basis for various hard
// clipping filters by adjusting the threshold and make-up gain.
template<int block_size> const CompressorPlugin<block_size>
HARD_CLIP_0DB{
    48000.f,  // sampleRateHz
    0.f,      // kneeDb
    0.f,      // thresholdDb
    0.f,      // attackTimeSeconds
    0.f,      // releaseTimeSeconds
    std::numeric_limits<float>::infinity(),  // ratioDbDb
    0.f,      // makeUpGainDb
    false     // useLookAhead
};

// A fairly robust limiter that can handle up to about 12dB over the threshold
// without noticeable clipping. It works well for playing on a keyboard at high
// amplitude, but its dynamic range could likely be improved.
template<int block_size> const CompressorPlugin<block_size>
STRONG_LIMITER_MAX_12DB{
    48000.f,
    20.f,
    -6.f,
    0.0025f,
    0.15f,
    100.f,
    0.0f,
    false
};

// A limiter with a 3dB knee and a hard limit.
template<int block_size> const CompressorPlugin<block_size>
SOFT_CLIP{
    48000.f,
    3.f,
    0.f,
    0.0025f,
    0.15f,
    std::numeric_limits<float>::infinity(),
    0.0f,
    false
};

}  // namespace filter

}  // namespace al
