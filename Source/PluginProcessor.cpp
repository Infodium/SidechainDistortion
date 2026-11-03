#include "PluginProcessor.h"

using namespace juce;

static float msToCoef (float ms, double sr)
{
    return std::exp (-1.0f / (std::max (ms, 0.01f) * 0.001f * (float) sr));
}

static float shapeSample (int type, float x, float drive)
{
    const float v = x * drive;
    switch (type)
    {
        case 0:  return std::tanh (v) / std::tanh (std::max (drive, 1.0f)); // soft
        case 1:  return jlimit (-1.0f, 1.0f, v);                             // hard clip
        default: return std::sin (v * MathConstants<float>::halfPi);         // fold
    }
}

//==============================================================================
AudioProcessorValueTreeState::ParameterLayout SidechainCrushAudioProcessor::createLayout()
{
    using P  = AudioParameterFloat;
    using PC = AudioParameterChoice;
    std::vector<std::unique_ptr<RangedAudioParameter>> p;

    auto skewed = [] (float lo, float hi, float centre)
    {
        NormalisableRange<float> r (lo, hi);
        r.setSkewForCentre (centre);
        return r;
    };

    p.push_back (std::make_unique<PC> (ParameterID { "mode", 1 }, "Mode",
                 StringArray { "Timed", "Follow (Crush)", "Both" }, 0));

    p.push_back (std::make_unique<P> (ParameterID { "threshold", 1 }, "Threshold",
                 NormalisableRange<float> (-60.f, 0.f, 0.1f), -30.f));

    // Timed section
    p.push_back (std::make_unique<P> (ParameterID { "delay", 1 }, "Delay (ms)",
                 skewed (0.f, 500.f, 50.f), 0.f));
    p.push_back (std::make_unique<P> (ParameterID { "attack", 1 }, "Attack (ms)",
                 skewed (0.1f, 200.f, 10.f), 5.f));
    p.push_back (std::make_unique<P> (ParameterID { "hold", 1 }, "Hold (ms)",
                 skewed (0.f, 1000.f, 100.f), 80.f));
    p.push_back (std::make_unique<P> (ParameterID { "release", 1 }, "Release (ms)",
                 skewed (1.f, 2000.f, 150.f), 150.f));

    // Follow section
    p.push_back (std::make_unique<P> (ParameterID { "focus", 1 }, "Low Focus (Hz)",
                 skewed (20.f, 500.f, 120.f), 120.f));
    p.push_back (std::make_unique<P> (ParameterID { "transient", 1 }, "Transient",
                 NormalisableRange<float> (0.f, 1.f, 0.01f), 0.3f));

    // Distortion
    p.push_back (std::make_unique<PC> (ParameterID { "shape", 1 }, "Shape",
                 StringArray { "Soft", "Hard", "Fold" }, 0));
    p.push_back (std::make_unique<P> (ParameterID { "drive", 1 }, "Max Drive (dB)",
                 NormalisableRange<float> (0.f, 48.f, 0.1f), 24.f));
    p.push_back (std::make_unique<P> (ParameterID { "amount", 1 }, "Amount",
                 NormalisableRange<float> (0.f, 1.f, 0.001f), 0.8f)); // the "80%" control
    p.push_back (std::make_unique<P> (ParameterID { "output", 1 }, "Output (dB)",
                 NormalisableRange<float> (-24.f, 12.f, 0.1f), 0.f));

    // Utility
    p.push_back (std::make_unique<AudioParameterBool> (ParameterID { "delta", 1 }, "Delta", false));
    p.push_back (std::make_unique<AudioParameterBool> (ParameterID { "bypass", 1 }, "Bypass", false));

    return { p.begin(), p.end() };
}

SidechainCrushAudioProcessor::SidechainCrushAudioProcessor()
    : AudioProcessor (BusesProperties()
                          .withInput  ("Input",     AudioChannelSet::stereo(), true)
                          .withOutput ("Output",    AudioChannelSet::stereo(), true)
                          .withInput  ("Sidechain", AudioChannelSet::stereo(), false)),
      apvts (*this, nullptr, "STATE", createLayout())
{
}

bool SidechainCrushAudioProcessor::isBusesLayoutSupported (const BusesLayout& l) const
{
    const auto out = l.getMainOutputChannelSet();
    if (out != AudioChannelSet::mono() && out != AudioChannelSet::stereo()) return false;
    if (out != l.getMainInputChannelSet()) return false;

    const auto sc = l.getChannelSet (true, 1);
    return sc.isDisabled() || sc == AudioChannelSet::mono() || sc == AudioChannelSet::stereo();
}

void SidechainCrushAudioProcessor::prepareToPlay (double sampleRate, int)
{
    sr = sampleRate;
    stage = Stage::idle; counter = 0; armed = true;
    trigEnv = timedEnv = slowEnv = fastEnv = modSmoothed = bypassSm = 0.f;
    lastFocus = -1.f;
    lp1.reset(); lp2.reset();

    fastAtkCoef   = msToCoef (0.5f, sr);
    fastRelCoef   = msToCoef (30.f, sr);
    trigRelCoef   = msToCoef (5.f, sr);
    modSmoothCoef = 1.0f - msToCoef (1.0f, sr);
    bypassCoef    = 1.0f - msToCoef (5.0f, sr);
}

//==============================================================================
float SidechainCrushAudioProcessor::tickTimed (float d)
{
    // Peak detector with short release, with hysteresis so one kick = one trigger
    trigEnv = std::max (std::abs (d), trigEnv * trigRelCoef);

    if (armed && trigEnv > thrLin)           { armed = false; stage = Stage::delay; counter = 0; }
    else if (! armed && trigEnv < thrLin * 0.5f) armed = true;

    switch (stage)
    {
        case Stage::idle:    break;
        case Stage::delay:   timedEnv = std::max (0.f, timedEnv - relDec);
                             if (++counter >= delaySamples) { stage = Stage::attack; counter = 0; }
                             break;
        case Stage::attack:  timedEnv += atkInc;
                             if (timedEnv >= 1.f) { timedEnv = 1.f; stage = Stage::hold; counter = 0; }
                             break;
        case Stage::hold:    if (++counter >= holdSamples) stage = Stage::release;
                             break;
        case Stage::release: timedEnv -= relDec;
                             if (timedEnv <= 0.f) { timedEnv = 0.f; stage = Stage::idle; }
                             break;
    }
    return timedEnv;
}

float SidechainCrushAudioProcessor::tickFollow (float d)
{
    // Only listen to the low end (808 body), so hats etc. don't trigger it
    const float a = std::abs (lp2.processSample (lp1.processSample (d)));

    slowEnv = a + (a > slowEnv ? atkCoef : relCoef) * (slowEnv - a);
    fastEnv = a + (a > fastEnv ? fastAtkCoef : fastRelCoef) * (fastEnv - a);

    const float db = Decibels::gainToDecibels (slowEnv, -100.f);
    const float level = jlimit (0.f, 1.f, (db - thrDb) / 12.f); // threshold -> +12 dB = 0..1

    // Transient emphasis: how far the fast follower is above the slow one
    float tr = 0.f;
    if (fastEnv > 1.0e-5f && Decibels::gainToDecibels (fastEnv, -100.f) > thrDb)
        tr = jlimit (0.f, 1.f, (fastEnv - slowEnv) / fastEnv);

    return jlimit (0.f, 1.f, level + transientAmt * tr);
}

//==============================================================================
void SidechainCrushAudioProcessor::processBlock (AudioBuffer<float>& buffer, MidiBuffer&)
{
    ScopedNoDenormals noDenormals;

    auto main = getBusBuffer (buffer, false, 0);
    const int numSamples = main.getNumSamples();
    const int numCh      = main.getNumChannels();

    // Detector source: sidechain bus if enabled, otherwise the plugin's own input
    const bool scOn = getBusCount (true) > 1 && getBus (true, 1)->isEnabled();
    const float* det[2] = { nullptr, nullptr };
    int detCh = 0;
    if (scOn)
    {
        auto sc = getBusBuffer (buffer, true, 1);
        detCh = jmin (sc.getNumChannels(), 2);
        for (int c = 0; c < detCh; ++c) det[c] = sc.getReadPointer (c);
    }
    else
    {
        detCh = jmin (numCh, 2);
        for (int c = 0; c < detCh; ++c) det[c] = main.getReadPointer (c);
    }

    // ---- Read params ----
    auto get = [this] (const char* id) { return apvts.getRawParameterValue (id)->load(); };
    const int   mode      = (int) get ("mode");
    const int   shape     = (int) get ("shape");
    const float amount    = get ("amount");
    const float maxDriveDb= get ("drive");
    const float outGain   = Decibels::decibelsToGain (get ("output"));
    const bool  delta     = get ("delta")  > 0.5f;
    const float bypassTarget = get ("bypass") > 0.5f ? 1.f : 0.f;
    const float focus     = get ("focus");
    transientAmt          = get ("transient");
    thrDb                 = get ("threshold");
    thrLin                = Decibels::decibelsToGain (thrDb);

    const float atkMs = get ("attack"), relMs = get ("release");
    delaySamples = (int) (get ("delay") * 0.001 * sr);
    holdSamples  = (int) (get ("hold")  * 0.001 * sr);
    atkInc = 1.f / std::max (1.f, atkMs * 0.001f * (float) sr);
    relDec = 1.f / std::max (1.f, relMs * 0.001f * (float) sr);
    atkCoef = msToCoef (atkMs, sr);
    relCoef = msToCoef (relMs, sr);

    if (focus != lastFocus)
    {
        auto c = dsp::IIR::Coefficients<float>::makeLowPass (sr, focus);
        lp1.coefficients = c; lp2.coefficients = c;
        lastFocus = focus;
    }

    // ---- Per-sample ----
    for (int i = 0; i < numSamples; ++i)
    {
        float d = 0.f;
        for (int c = 0; c < detCh; ++c) d += det[c][i];
        if (detCh > 0) d /= (float) detCh;

        float env = 0.f;
        if (mode != 1) env = std::max (env, tickTimed (d));
        if (mode != 0) env = std::max (env, tickFollow (d));

        modSmoothed += (env * amount - modSmoothed) * modSmoothCoef;
        const float drive = Decibels::decibelsToGain (modSmoothed * maxDriveDb);

        // Detectors keep running while bypassed, so un-bypassing never glitches
        bypassSm += (bypassTarget - bypassSm) * bypassCoef;

        for (int c = 0; c < numCh; ++c)
        {
            float* x = main.getWritePointer (c);
            const float dry  = x[i];
            const float wet  = shapeSample (shape, dry, drive);
            const float proc = (dry + (wet - dry) * modSmoothed) * outGain;
            const float res  = delta ? (proc - dry) : proc;   // delta = only what the effect adds
            x[i] = res + (dry - res) * bypassSm;               // click-free bypass crossfade
        }
    }
}

//==============================================================================
AudioProcessorEditor* SidechainCrushAudioProcessor::createEditor()
{
    return new GenericAudioProcessorEditor (*this); // placeholder UI, swap for a custom one later
}

void SidechainCrushAudioProcessor::getStateInformation (MemoryBlock& dest)
{
    if (auto xml = apvts.copyState().createXml()) copyXmlToBinary (*xml, dest);
}

void SidechainCrushAudioProcessor::setStateInformation (const void* data, int size)
{
    if (auto xml = getXmlFromBinary (data, size))
        if (xml->hasTagName (apvts.state.getType()))
            apvts.replaceState (ValueTree::fromXml (*xml));
}

AudioProcessor* JUCE_CALLTYPE createPluginFilter() { return new SidechainCrushAudioProcessor(); }
