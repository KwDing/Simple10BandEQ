/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#define NUMPEAKFILTER 8
using Filter = juce::dsp::IIR::Filter<float>;
using CutFilter = juce::dsp::ProcessorChain<Filter, Filter,Filter,Filter>;
using PeakFilters = juce::dsp::ProcessorChain<Filter, Filter, Filter, Filter, Filter, Filter, Filter, Filter>;
using MonoChain = juce::dsp::ProcessorChain<CutFilter, PeakFilters, CutFilter>;

enum Slope{
    Slope_12,
    Slope_24,
    Slope_36,
    Slope_48
};
enum ChainPos{
    LowCut,
//    Peak1,
    Peaks,
    HighCut
};
struct ChainSettings{
//    float peak1Freq {0}, peak1GainDB{0}, peak1Q{1.f};
    float peakFreqs[NUMPEAKFILTER]={100.f,200.f,500.f,1000.f,2000.f,3000.f,5000.f,10000.f}, peakGainsDB[NUMPEAKFILTER] = {0}, peakQs[NUMPEAKFILTER]={1.f};
    float lowCutFreq{0}, highCutFreq{0};
//    int lowCutSlope{0}, highCutSlope{0};
    Slope lowCutSlope{Slope::Slope_12}, highCutSlope{Slope::Slope_12};
};

ChainSettings getChainSettings(juce::AudioProcessorValueTreeState& apvts);
void updateCoefficients(Filter::CoefficientsPtr& oldCoef, const Filter::CoefficientsPtr& newCoef);
std::vector<Filter::CoefficientsPtr> getPeakFilters(const ChainSettings& chainSettings, double sampleRate);

template<int index, typename ChainType, typename CoefficientType>
void cutHelper(ChainType& chain, const CoefficientType& coefs){
    updateCoefficients(chain.template get<index>().coefficients, coefs[index]);
    chain.template setBypassed<index>(false);
};

template<typename ChainType, typename CoefficientType>
void updateCutFilter(ChainType& cutChain, const CoefficientType& cutCoefs, const Slope& lowCutSlope){
    cutChain.template setBypassed<0>(true);
    cutChain.template setBypassed<1>(true);
    cutChain.template setBypassed<2>(true);
    cutChain.template setBypassed<3>(true);
    switch(lowCutSlope){
    case Slope_48:
        cutHelper<3>(cutChain, cutCoefs);
    case Slope_36:
        cutHelper<2>(cutChain, cutCoefs);
    case Slope_24:
        cutHelper<1>(cutChain, cutCoefs);
    case Slope_12:
        cutHelper<0>(cutChain, cutCoefs);
    }

};

inline auto getLowCutFilter(const ChainSettings& chainSettings, double sampleRate){
    return juce::dsp::FilterDesign<float>::designIIRHighpassHighOrderButterworthMethod(chainSettings.lowCutFreq,
                                                                                                sampleRate,
                                                                                                (2*chainSettings.lowCutSlope + 2) );
}

inline auto getHighCutFilter(const ChainSettings& chainSettings, double sampleRate){
    return juce::dsp::FilterDesign<float>::designIIRLowpassHighOrderButterworthMethod(chainSettings.highCutFreq,
                                                                                                sampleRate,
                                                                                                (2*chainSettings.highCutSlope + 2) );
}

template<typename T>
struct Fifo
{
    void prepare(int numChannels, int numSamples){
        static_assert(std::is_same_v<T, juce::AudioBuffer<float>>, "Fifo not holding uce::AudioBuffer<float>");
        for( auto& buffer: buffers){
            buffer.setSize(numChannels, numSamples,
                           false,
                           true,
                           true);
            buffer.clear();
        }
    }
    
    void prepare(size_t numElements){
        static_assert(std::is_same_v<T, std::vector<float>>, "Fifo not holding uce::AudioBuffer<float>");
        for(auto& buffer : buffers){
            buffer.clear();
            buffer.resize(numElements, 0);
        }
    }
    bool push(const T& t){
        auto write = fifo.write(1);
        if(write.blockSize1 > 0){
            buffers[write.startIndex1] = t;
            return true;
        }
        return false;
    }
    bool pull(T& t){
        auto read = fifo.read(1);
        if(read.blockSize1 > 0){
            t = buffers[read.startIndex1];
            return true;
        }
        return false;
    }
    int getNumAvailableForReading() const{
        return fifo.getNumReady();
    }
private:
    static constexpr int Capacity = 30;
    std::array<T, Capacity> buffers;
    juce::AbstractFifo fifo{Capacity};
};

template<typename BlockType>
struct SingleChannelSampleFifo
{
    SingleChannelSampleFifo(int ch): channel(ch)
    {
        prepared.set(false);
    }
    void update(const BlockType& buffer){
        jassert(prepared.get());
        jassert(buffer.getNumChannels() > channel);
        auto* chPtr = buffer.getReadPointer(channel);
        for(int i = 0; i<buffer.getNumSamples(); i++){
            pushNextSampleIntoFifo(chPtr[i]);
        }
    }
    void prepare(int bufferSize){
        prepared.set(false);
        size.set(bufferSize);
        bufferToFill.setSize(1, bufferSize, false, true,true);
        audioBufferFifo.prepare(1, bufferSize);
        fifoIndex = 0;
        prepared.set(true);
    }
    int getNumCompleteBuffersAvailable() const {return audioBufferFifo.getNumAvailableForReading();}
    bool isPrepared() const {return prepared.get();}
    int getSize() const{return size.get();}
    bool getAudioBuffer(BlockType& buffer){return audioBufferFifo.pull(buffer);}
private:
    int channel;
    int fifoIndex = 0;
    Fifo<BlockType> audioBufferFifo;
    BlockType bufferToFill;
    juce::Atomic<bool> prepared = false;
    juce::Atomic<int> size = 0;
    void pushNextSampleIntoFifo(float sample){
        if(fifoIndex == bufferToFill.getNumSamples()){
            auto pushed = audioBufferFifo.push(bufferToFill);
            juce::ignoreUnused(pushed);
            fifoIndex = 0;
        }
        bufferToFill.setSample(0, fifoIndex, sample);
        fifoIndex++;
    }
};

//==============================================================================
/**
*/
class EQAudioProcessor  : public juce::AudioProcessor
{
public:
    //==============================================================================
    EQAudioProcessor();
    ~EQAudioProcessor() override;

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

   #ifndef JucePlugin_PreferredChannelConfigurations
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
   #endif

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const juce::String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    //==============================================================================
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;
    juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();
    juce::AudioProcessorValueTreeState apvts{*this, nullptr, "Parameters", createParameterLayout()};
    
    MonoChain leftChain, rightChain;
    SingleChannelSampleFifo<juce::AudioBuffer<float>> leftFifo{0}, rightFifo{1};
private:
    float defaultPeakFreqs[NUMPEAKFILTER] = {100.f,200.f,500.f,1000.f,2000.f,3000.f,5000.f,10000.f};
    
    
    
//    static void updateCoefficients(Filter::CoefficientsPtr& oldCoef, const Filter::CoefficientsPtr& newCoef);
    void updateLowCutFilters(const ChainSettings& chainSettings);
    void updatePeakFilter(const ChainSettings& chainSettings);
    void updateHighCutFilters(const ChainSettings& chainSettings);
    void updateFilters();
    
//    template<int index, typename ChainType, typename CoefficientType>
//    void peakHelper(ChainType& chain, const CoefficientType& coefs){
//       updateCoefficients(chain.template get<index>().coefficients, coefs[index]);
//    };
    
    
    
    
    // juce::dsp::ProcessorChain & juce::dsp::ProcessContext
    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EQAudioProcessor)
};
