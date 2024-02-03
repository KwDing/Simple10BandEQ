/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
EQAudioProcessor::EQAudioProcessor()
#ifndef JucePlugin_PreferredChannelConfigurations
     : AudioProcessor (BusesProperties()
                     #if ! JucePlugin_IsMidiEffect
                      #if ! JucePlugin_IsSynth
                       .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                      #endif
                       .withOutput ("Output", juce::AudioChannelSet::stereo(), true)
                     #endif
                       )
#endif
{
}

EQAudioProcessor::~EQAudioProcessor()
{
}

//==============================================================================
const juce::String EQAudioProcessor::getName() const
{
    return JucePlugin_Name;
}

bool EQAudioProcessor::acceptsMidi() const
{
   #if JucePlugin_WantsMidiInput
    return true;
   #else
    return false;
   #endif
}

bool EQAudioProcessor::producesMidi() const
{
   #if JucePlugin_ProducesMidiOutput
    return true;
   #else
    return false;
   #endif
}

bool EQAudioProcessor::isMidiEffect() const
{
   #if JucePlugin_IsMidiEffect
    return true;
   #else
    return false;
   #endif
}

double EQAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}

int EQAudioProcessor::getNumPrograms()
{
    return 1;   // NB: some hosts don't cope very well if you tell them there are 0 programs,
                // so this should be at least 1, even if you're not really implementing programs.
}

int EQAudioProcessor::getCurrentProgram()
{
    return 0;
}

void EQAudioProcessor::setCurrentProgram (int index)
{
}

const juce::String EQAudioProcessor::getProgramName (int index)
{
    return {};
}

void EQAudioProcessor::changeProgramName (int index, const juce::String& newName)
{
}

//==============================================================================
void EQAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    // Use this method as the place to do any pre-playback
    // initialisation that you need..
    juce::dsp::ProcessSpec spec;
    spec.maximumBlockSize = samplesPerBlock;
    spec.numChannels = 1;
    spec.sampleRate = sampleRate;
    leftChain.prepare(spec);
    rightChain.prepare(spec);
    
    updateFilters();
    
    leftFifo.prepare(samplesPerBlock);
    rightFifo.prepare(samplesPerBlock);
    
//    osc.initia
////    auto highCutCoefs = juce::dsp::FilterDesign<float>::designIIRLowpassHighOrderButterworthMethod(float frequency, sampleRate, float normalisedTransitionWidth, float passbandAmplitudedB, float stopbandAmplitudedB)()

}

void EQAudioProcessor::releaseResources()
{
    // When playback stops, you can use this as an opportunity to free up any
    // spare memory, etc.
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool EQAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
  #if JucePlugin_IsMidiEffect
    juce::ignoreUnused (layouts);
    return true;
  #else
    // This is the place where you check if the layout is supported.
    // In this template code we only support mono or stereo.
    // Some plugin hosts, such as certain GarageBand versions, will only
    // load plugins that support stereo bus layouts.
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::mono()
     && layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    // This checks if the input layout matches the output layout
   #if ! JucePlugin_IsSynth
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;
   #endif

    return true;
  #endif
}
#endif

void EQAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels  = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    // In case we have more outputs than inputs, this code clears any output
    // channels that didn't contain input data, (because these aren't
    // guaranteed to be empty - they may contain garbage).
    // This is here to avoid people getting screaming feedback
    // when they first compile a plugin, but obviously you don't need to keep
    // this code if your algorithm always overwrites all the output channels.
    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear (i, 0, buffer.getNumSamples());

    // This is the place where you'd normally do the guts of your plugin's
    // audio processing...
    // Make sure to reset the state if your inner loop is processing
    // the samples and the outer loop is handling the channels.
    // Alternatively, you can process the samples with the channels
    // interleaved by keeping the same state.
    updateFilters();
    
    juce::dsp::AudioBlock<float> block(buffer);
    auto leftBlock = block.getSingleChannelBlock(0);
    auto rightBlock = block.getSingleChannelBlock(1);
    juce::dsp::ProcessContextReplacing<float> leftContext(leftBlock);
    juce::dsp::ProcessContextReplacing<float> rightContext(rightBlock);
    leftChain.process(leftContext);
    rightChain.process(rightContext);
    
    leftFifo.update(buffer);
    rightFifo.update(buffer);
//    for (int channel = 0; channel < totalNumInputChannels; ++channel)
//    {
//        auto* channelData = buffer.getWritePointer (channel);
//
//        // ..do something to the data...
//    }
}

//==============================================================================
bool EQAudioProcessor::hasEditor() const
{
    return true; // (change this to false if you choose to not supply an editor)
}

juce::AudioProcessorEditor* EQAudioProcessor::createEditor()
{
    return new EQAudioProcessorEditor (*this);
//    return new juce::GenericAudioProcessorEditor(*this);
}

//==============================================================================
void EQAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    // You should use this method to store your parameters in the memory block.
    // You could do that either as raw data, or use the XML or ValueTree classes
    // as intermediaries to make it easy to save and load complex data.
    juce::MemoryOutputStream mos(destData,true);
    apvts.state.writeToStream(mos);
}

void EQAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    // You should use this method to restore your parameters from this memory block,
    // whose contents will have been created by the getStateInformation() call.
    auto tree = juce::ValueTree::readFromData(data, sizeInBytes);
    if(tree.isValid()){
        apvts.replaceState(tree);
        updateFilters();
    }
}
juce::AudioProcessorValueTreeState::ParameterLayout EQAudioProcessor::createParameterLayout(){
    juce::AudioProcessorValueTreeState::ParameterLayout layout;
    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("LowCut Freq", 1),
                                                           "LowCut Freq",
                                                           juce::NormalisableRange<float>(20.f, 20000.f, 1.f, 0.1f),
                                                           20.f));
    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("HighCut Freq",1),
                                                           "HighCut Freq",
                                                           juce::NormalisableRange<float>(20.f, 20000.f, 1.f, 0.25f),
                                                           20000.f));
    for(int i = 0; i < NUMPEAKFILTER; ++i){
        juce::String name;
        name << "Peak" << i+1 << " ";
        layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID(name+"Freq",1),
                                                               name+"Freq",
                                                               juce::NormalisableRange<float>(20.f, 20000.f, 1.f, 0.25f),
                                                               defaultPeakFreqs[i]));
        layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID(name+"Gain",1),
                                                               name+"Gain",
                                                               juce::NormalisableRange<float>(-24.0f, 24.0f, 0.1f, 1.f),
                                                               0.0f));
        layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID(name+"Q",1),
                                                               name+"Q",
                                                               juce::NormalisableRange<float>(0.1f, 20.0f, 0.01f, 1.f),
                                                               1.0f));
    }
    juce::StringArray choices;
    for(int i = 0; i < 4; i++){
        juce::String s;
        s << (12+i*12);
        s << " db/Oct";
        choices.add(s);
    }
    layout.add(std::make_unique<juce::AudioParameterChoice>(juce::ParameterID("LowCut Slope",1),
                                                            "LowCut Slope",choices,0));
    layout.add(std::make_unique<juce::AudioParameterChoice>(juce::ParameterID("HighCut Slope",1),"HighCut Slope",choices,0));
    return layout;
}
ChainSettings getChainSettings(juce::AudioProcessorValueTreeState& apvts){
    ChainSettings settings;
    settings.lowCutFreq = apvts.getRawParameterValue("LowCut Freq")->load();
    settings.highCutFreq = apvts.getRawParameterValue("HighCut Freq")->load();
    for(int i=0; i< NUMPEAKFILTER;i++){
        juce::String name;
        name << "Peak" << i+1 << " ";
        settings.peakFreqs[i] = apvts.getRawParameterValue(name+"Freq")->load();
        settings.peakGainsDB[i] = apvts.getRawParameterValue(name+"Gain")->load();
        settings.peakQs[i] = apvts.getRawParameterValue(name+"Q")->load();
    }
    settings.lowCutSlope = static_cast<Slope>( apvts.getRawParameterValue("LowCut Slope")->load() );
    settings.highCutSlope = static_cast<Slope>( apvts.getRawParameterValue("HighCut Slope")->load() );
    return settings;
}
void updateCoefficients(Filter::CoefficientsPtr &oldCoef, const Filter::CoefficientsPtr &newCoef){
    *oldCoef = *newCoef;
}
std::vector<Filter::CoefficientsPtr> getPeakFilters(const ChainSettings& chainSettings, double sampleRate){
    std::vector<Filter::CoefficientsPtr> peakCoefs(8);
    for(int i = 0; i < NUMPEAKFILTER; i++){
        peakCoefs[i] = juce::dsp::IIR::Coefficients<float>::makePeakFilter(sampleRate,
                                                                           chainSettings.peakFreqs[i],
                                                                           chainSettings.peakQs[i],
                                                                           juce::Decibels::decibelsToGain(chainSettings.peakGainsDB[i]));
    }
    return peakCoefs;
}
void EQAudioProcessor::updatePeakFilter(const ChainSettings& chainSettings){
//    auto peakCoef = juce::dsp::IIR::Coefficients<float>::makePeakFilter(getSampleRate(),
//                                                                         chainSettings.peak1Freq,
//                                                                         chainSettings.peak1Q,
//                                                                         juce::Decibels::decibelsToGain(chainSettings.peak1GainDB));
//    *leftChain.get<ChainPos::Peak1>().coefficients = *peakCoef;
//    *rightChain.get<ChainPos::Peak1>().coefficients = *peakCoef;
//    updateCoefficients(leftChain.get<ChainPos::Peak1>().coefficients, peakCoef);
//    updateCoefficients(rightChain.get<ChainPos::Peak1>().coefficients, peakCoef);
    auto& leftPeaks = leftChain.get<ChainPos::Peaks>();
    auto& rightPeaks = rightChain.get<ChainPos::Peaks>();
    auto peakCoefs = getPeakFilters(chainSettings, getSampleRate());
    updateCoefficients(leftPeaks.get<0>().coefficients, peakCoefs[0]);
    updateCoefficients(rightPeaks.get<0>().coefficients,peakCoefs[0]);

    updateCoefficients(leftPeaks.get<1>().coefficients, peakCoefs[1]);
    updateCoefficients(rightPeaks.get<1>().coefficients,peakCoefs[1]);
    
    updateCoefficients(leftPeaks.get<2>().coefficients, peakCoefs[2]);
    updateCoefficients(rightPeaks.get<2>().coefficients,peakCoefs[2]);
    
    updateCoefficients(leftPeaks.get<3>().coefficients, peakCoefs[3]);
    updateCoefficients(rightPeaks.get<3>().coefficients,peakCoefs[3]);
    
    updateCoefficients(leftPeaks.get<4>().coefficients, peakCoefs[4]);
    updateCoefficients(rightPeaks.get<4>().coefficients,peakCoefs[4]);
    
    updateCoefficients(leftPeaks.get<5>().coefficients, peakCoefs[5]);
    updateCoefficients(rightPeaks.get<5>().coefficients,peakCoefs[5]);
    
    updateCoefficients(leftPeaks.get<6>().coefficients, peakCoefs[6]);
    updateCoefficients(rightPeaks.get<6>().coefficients,peakCoefs[6]);
    
    updateCoefficients(leftPeaks.get<7>().coefficients, peakCoefs[7]);
    updateCoefficients(rightPeaks.get<7>().coefficients,peakCoefs[7]);
    
//    for(int i = 0; i < 8; i++){
//        auto peakCoefs = juce::dsp::IIR::Coefficients<float>::makePeakFilter(getSampleRate(),
//                                                                             chainSettings.peakFreqs[i],
//                                                                             chainSettings.peakQs[i],
//                                                                             juce::Decibels::decibelsToGain(chainSettings.peakGainsDB[i]));
//        peakHelper<i>(leftPeaks, peakCoefs);
//        updateCoefficients(leftPeaks.get<peakIndex[i]>().coefficients, peakCoefs);
//    }
}
void EQAudioProcessor::updateLowCutFilters(const ChainSettings &chainSettings){
    auto lowCutCoefs = getLowCutFilter(chainSettings, getSampleRate());
    auto& leftLowCut = leftChain.get<ChainPos::LowCut>();
    auto& rightLowCut = rightChain.get<ChainPos::LowCut>();
    updateCutFilter(leftLowCut, lowCutCoefs, chainSettings.lowCutSlope);
    updateCutFilter(rightLowCut, lowCutCoefs, chainSettings.lowCutSlope);
}
void EQAudioProcessor::updateHighCutFilters(const ChainSettings &chainSettings){
    auto highCutCoefs = getHighCutFilter(chainSettings, getSampleRate());
    auto& leftHighCut = leftChain.get<ChainPos::HighCut>();
    auto& rightHighCut = rightChain.get<ChainPos::HighCut>();
    updateCutFilter(leftHighCut, highCutCoefs, chainSettings.highCutSlope);
    updateCutFilter(rightHighCut, highCutCoefs, chainSettings.highCutSlope);
}
void EQAudioProcessor::updateFilters(){
    auto chainSettings = getChainSettings(apvts);
    updateLowCutFilters(chainSettings);
    updatePeakFilter(chainSettings);
    updateHighCutFilters(chainSettings);
}
//==============================================================================
// This creates new instances of the plugin..

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new EQAudioProcessor();
}
