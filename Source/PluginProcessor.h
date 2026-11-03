#pragma once
#include <juce_audio_processors/juce_audio_processors.h>
#include <juce_dsp/juce_dsp.h>

class SidechainCrushAudioProcessor : public juce::AudioProcessor
{
public:
    SidechainCrushAudioProcessor();
    ~SidechainCrushAudioProcessor() override = default;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}
    bool isBusesLayoutSupported (const BusesLayout&) const override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }
    juce::AudioProcessorParameter* getBypassParameter() const override { return apvts.getParameter ("bypass"); }

    const juce::String getName() const override { return "SidechainCrush"; }
    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return 0.0; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram (int) override {}
    const juce::String getProgramName (int) override { return {}; }
    void changeProgramName (int, const juce::String&) override {}

    void getStateInformation (juce::MemoryBlock&) override;
    void setStateInformation (const void*, int) override;

    juce::AudioProcessorValueTreeState apvts;

private:
    static juce::AudioProcessorValueTreeState::ParameterLayout createLayout();

    float tickTimed  (float detector);
    float tickFollow (float detector);

    double sr = 44100.0;

    // ---- Timed mode state ----
    enum class Stage { idle, delay, attack, hold, release };
    Stage stage = Stage::idle;
    int   counter = 0;
    bool  armed = true;
    float trigEnv = 0.f, timedEnv = 0.f;

    // ---- Follow ("crush") mode state ----
    juce::dsp::IIR::Filter<float> lp1, lp2;
    float slowEnv = 0.f, fastEnv = 0.f;
    float lastFocus = -1.f;

    // ---- Per-block cached values ----
    int   delaySamples = 0, holdSamples = 0;
    float atkInc = 1.f, relDec = 1.f;
    float thrLin = 0.03f, thrDb = -30.f;
    float atkCoef = 0.f, relCoef = 0.f, fastAtkCoef = 0.f, fastRelCoef = 0.f, trigRelCoef = 0.f;
    float transientAmt = 0.f;

    float modSmoothed = 0.f, modSmoothCoef = 0.f;
    float bypassSm = 0.f, bypassCoef = 0.f;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (SidechainCrushAudioProcessor)
};
