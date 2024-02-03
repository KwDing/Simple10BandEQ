/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"

//==============================================================================
/**
*/
//struct CustomRotarySlider : juce::Slider{
//    CustomRotarySlider() : juce::Slider(juce::Slider::SliderStyle::RotaryHorizontalVerticalDrag,
//                                        juce::Slider::TextEntryBoxPosition::TextBoxBelow)
//    {
//
//    }
//};
enum SliderType{
    Freq,
    Gain,
    Q
};
struct LookAndFeel: juce::LookAndFeel_V4{
    void drawRotarySlider (juce::Graphics& g, int x, int y, int width, int height,
                           float sliderPosProportional, float rotaryStartAngle,
                           float rotaryEndAngle, juce::Slider&) override;
};
struct RotarySliderWithLabels: juce::Slider{
    RotarySliderWithLabels():juce::Slider(juce::Slider::SliderStyle::RotaryHorizontalVerticalDrag,
                                          juce::Slider::TextEntryBoxPosition::NoTextBox//juce::Slider::TextEntryBoxPosition::TextBoxBelow
                                          ){
        setLookAndFeel(&lnf);
        
    }
//    RotarySliderWithLabels():juce::Slider(){setLookAndFeel(&lnf);}
    RotarySliderWithLabels(juce::RangedAudioParameter& rap, const juce::String& unitsuffix):
    param(&rap),suffix(unitsuffix)
    {
        setLookAndFeel(&lnf);
    }
    ~RotarySliderWithLabels(){setLookAndFeel(nullptr);}
    void set(juce::RangedAudioParameter* param, juce::String suffix){
        this->param = param;
        this->suffix = suffix;
    }
    
    struct LabelPos{
        float pos;
        juce::String label;
    };
    
    juce::Array<LabelPos> labels;
    
    void paint(juce::Graphics& g) override{
        float startAng = juce::degreesToRadians(180.f + 45.f);
        float endAng = juce::degreesToRadians(180.f - 45.f) + juce::MathConstants<float>::twoPi;
        auto range = getRange();
        auto sliderBounds = getSliderBounds();
        
//        g.setColour(juce::Colours::white);
//        g.drawRect(getLocalBounds());
//        g.setColour(juce::Colours::grey);
//        g.drawRect(sliderBounds);
        
        getLookAndFeel().drawRotarySlider(g, sliderBounds.getX(), sliderBounds.getY(),
                                          sliderBounds.getWidth(), sliderBounds.getHeight(),
                                          juce::jmap(getValue(), range.getStart(), range.getEnd(), 0.0, 1.0),
                                          startAng, endAng, *this);
        
        auto center = sliderBounds.toFloat().getCentre();
        auto radius = sliderBounds.getWidth() * 0.5f;
        g.setColour(juce::Colours::white);
        g.setFont(getTextHeight());
        for(int i = 0; i < labels.size(); i++){
            auto pos = labels[i].pos;
            jassert(0.f <= pos && pos <= 1.f);
            auto ang = juce::jmap(pos, 0.f, 1.f, startAng, endAng);
            auto centerPoint = center.getPointOnCircumference(1+ radius + getTextHeight() * 0.5f, ang);
            juce::Rectangle<float> rec;
            auto str = labels[i].label;
            rec.setSize(g.getCurrentFont().getStringWidth(str), getTextHeight());
            rec.setCentre(centerPoint);
            rec.setY(rec.getY()+getTextHeight());
            g.drawFittedText(str, rec.toNearestInt(), juce::Justification::centred, 1);
        }
        
    }
    juce::Rectangle<int> getSliderBounds(){
        auto bounds = getLocalBounds();
        auto size = juce::jmin(bounds.getWidth(), bounds.getHeight());
        size -= getTextHeight() * 2;
        juce::Rectangle<int> rec;
        rec.setSize(size, size);
        rec.setCentre(bounds.getCentreX(),0);
        rec.setY(2);
        return rec;
    }
    int getTextHeight() const{return 12;}
    juce::String getDisplayString() const{
        if(auto* choiceParam = dynamic_cast<juce::AudioParameterChoice*> (param))
            return choiceParam->getCurrentChoiceName();
        juce::String str;
        bool addK = false;
        if(auto* floatParam = dynamic_cast<juce::AudioParameterFloat*>(param)){
            float v = getValue();
            if(v >= 999.f){
                v /= 1000.f;
                addK = true;
            }
            if(sliderType == SliderType::Freq)
                str << juce::String(v, addK? 2:0) << (addK? "k ": " ") << suffix;
            else if(sliderType == SliderType::Gain)
                str << juce::String(v, 1) << " " << suffix;
            else str = juce::String(v);
        }
        return str;
    }
    
    SliderType sliderType;
private:
    juce::RangedAudioParameter* param;
    juce::String suffix;
    LookAndFeel lnf;
    
};


struct SpectrogramData{
    juce::dsp::FFT fft;
    
};
struct CustomComboBox : juce::ComboBox{
    CustomComboBox() : juce::ComboBox(){
        this->addItem("12 db/oct", 1);
        this->addItem("24 db/oct", 2);
        this->addItem("36 db/oct", 3);
        this->addItem("48 db/oct", 4);
        this->setSelectedId(1);
    };
};
template<typename BlockType>
struct FFTDataGenerator{
    FFTDataGenerator(): channel(0){}
    FFTDataGenerator(int ch) : channel(ch){}
    void pushFFTData(const juce::AudioBuffer<float>& audioData, const float negativeInfinity){
        const auto fftSize = getFFTSize();
        fftData.assign(fftData.size(),0);
        auto* readIndex = audioData.getReadPointer(0);//channel number
        std::copy(readIndex, readIndex + fftSize, fftData.begin());
        
        window->multiplyWithWindowingTable(fftData.data(), fftSize);
        forwardFFT->performFrequencyOnlyForwardTransform(fftData.data());
        
        int numBins = (int)fftSize/2;
        
        for(int i=0; i < numBins; i++){
            float v = fftData[i];
            if(!std::isinf(v) && !std::isnan(v)) {
                fftData[i] = juce::Decibels::gainToDecibels(v / float(numBins), negativeInfinity);
            }
            else fftData[i] = juce::Decibels::gainToDecibels(0.f, negativeInfinity);
        }
        fftFifo.push(fftData);
        
    }
    void changeOrder(int newOrder){
        order = newOrder;
        int fftSize = getFFTSize();
        
        forwardFFT = std::make_unique<juce::dsp::FFT>(order);
        window = std::make_unique<juce::dsp::WindowingFunction<float>>(fftSize,
                                                                           juce::dsp::WindowingFunction<float>::blackmanHarris);
        fftData.clear();
        fftSize *= 2;
        fftData.resize(fftSize, 0);
        fftFifo.prepare(fftSize);
    }
    int getFFTSize() const{
        return 1 << order;
    }
    bool getFFTData(BlockType& fftData){
        return fftFifo.pull(fftData);
    }
    int getNumAvailableFFTDataBlocks() const {
        return fftFifo.getNumAvailableForReading();
    }
private:
    int order;
    int channel;
    BlockType fftData;
    std::unique_ptr<juce::dsp::FFT> forwardFFT;
    std::unique_ptr<juce::dsp::WindowingFunction<float>> window;
    
    Fifo<BlockType> fftFifo;
};

template<typename PathType>
struct SpectrumPathGenerator{
    SpectrumPathGenerator(SingleChannelSampleFifo<juce::AudioBuffer<float>>& fifo):
    monoSampleFifo(&fifo){
        monoFFTDataGenerator.changeOrder(13);
        monoBuffer.setSize(1, monoFFTDataGenerator.getFFTSize());
    }
    void generate(const std::vector<float>& data, juce::Rectangle<float> bounds,
                  int fftSize, float binWidth, float negInf){
        float t = bounds.getY();
        float b = bounds.getBottom();
        float w = bounds.getWidth();
        float l = bounds.getX();
        
        int numBins = (int) fftSize/2;
        PathType p;
        p.preallocateSpace(6*(int)w);
        auto map = [b, t, negInf](float v){
            return juce::jmap(v, negInf, 0.f, b, t);
        };
        //        auto y = map(data[0]);
        //        if(std::isnan(y) || std::isinf(y)) y = b;
        //        p.startNewSubPath(l,y);
        
        p.startNewSubPath(l,b);
        int x = l, startBin = int(20/binWidth)+1, endBin = std::min(int(20000/binWidth), numBins);
        int n = startBin;
        int start = l, end = int(juce::mapFromLog10(n*binWidth, 20.f, 20000.f) * w) + l;
        float y, v1, v = n > 0? data[n-1] : 0.f, delta = (data[n] - v) / float(end-start);
        
//        while(x++<end+l){
//            v += delta;
//            y = map(v);
//            if(std::isnan(y) || std::isinf(y)) y = b;
//            p.lineTo(x, y);
//            p.lineTo(x, b);
//        }

//        while(n++ < endBin){
//            v = n > 0? data[n-1] : 0.f;
//            v1 = data[n];
////            delta = (data[n] - v)/float(end-start);
//            while(x++ < end){
////                v += delta;
//                y = map(linearInterp(v, v1, x,start, end-1));
////                y = map(v);
//                if(std::isnan(y) || std::isinf(y)) y = b;
//                p.lineTo(x, y);
//                p.lineTo(x, b);
//            }
//            start = end+1;
//            end = std::min(int(juce::mapFromLog10(n*binWidth, 20.f, 20000.f) * w) + l, l+w);
//        }


        
        for(int i = 1; i < numBins; i++){
            y = map(data[i]);
            if(std::isnan(y) || std::isinf(y)) y = b;
            float normalized = juce::mapFromLog10(i*binWidth, 20.f, 20000.f);
            if(normalized < 1 && normalized > 0){
                int normaled = int(normalized * w);
                p.lineTo(normaled + l, y);
            }
        }
        pathFifo.push(p);
    }
    void process(juce::Rectangle<float> bounds, double fs, float negInf = -48.f){
        juce::AudioBuffer<float> tempBuffer;
        while(monoSampleFifo->getNumCompleteBuffersAvailable()>0){
            if(monoSampleFifo->getAudioBuffer(tempBuffer)){
                int numSamples = tempBuffer.getNumSamples();
                juce::FloatVectorOperations::copy(monoBuffer.getWritePointer(0,0),
                                                  monoBuffer.getReadPointer(0,numSamples),
                                                  monoBuffer.getNumSamples()-numSamples);
//                juce::FloatVectorOperations::copy(<#float *dest#>, <#const float *src#>, <#int numValues#>)
                juce::FloatVectorOperations::copy(monoBuffer.getWritePointer(0, monoBuffer.getNumSamples()-numSamples),
                                                  tempBuffer.getReadPointer(0, 0),
                                                  numSamples);
                monoFFTDataGenerator.pushFFTData(monoBuffer, negInf);
            }
        }
        const auto fftSize = monoFFTDataGenerator.getFFTSize();
        const auto binWidth = fs/double(fftSize);
        while(monoFFTDataGenerator.getNumAvailableFFTDataBlocks()>0){
            std::vector<float> fftData;
            if(monoFFTDataGenerator.getFFTData(fftData)){
                generate(fftData, bounds, fftSize, binWidth, negInf);
            }
        }
        while(getNumPathAvailable()>0){
            pullPath(monoFFTPath);
        }
    }
    int getNumPathAvailable() const{
        return pathFifo.getNumAvailableForReading();
    }
    bool pullPath(PathType& path){
        return pathFifo.pull(path);
    }
    PathType getPath(){
        return monoFFTPath;
    }
private:
    float cubicInterp(float p0, float p1, float p2, float p3, float x){
        float a = -0.5 * p0 + 1.5 * p1 - 1.5 * p2 + 0.5* p3;
        float b = p0 - 2.5 * p1 + 2 * p2 - 0.5 * p3;
        float c = -0.5 * p0 + 0.5 * p2;
        return (a * pow(x,3) + b * pow(x,2) + c * x + p1);
    };
    inline float linearInterp(float p0, float p1, float x, float start = 0, float end = 0){
        return ( (x - start)*p1+(end-x)*p0 ) / (end - start);
    }
    Fifo<PathType> pathFifo;
    juce::AudioBuffer<float> monoBuffer;
    SingleChannelSampleFifo<juce::AudioBuffer<float>>* monoSampleFifo;
    FFTDataGenerator<std::vector<float>> monoFFTDataGenerator;
    PathType monoFFTPath;
};



struct ResponseCurveComponent : public juce::AudioProcessorEditor, juce::AudioProcessorParameter::Listener, juce::Timer{
    ResponseCurveComponent(EQAudioProcessor& p): juce::AudioProcessorEditor(&p), audioProcessor(p)
,leftPathGenerator(audioProcessor.leftFifo), rightPathGenerator(audioProcessor.rightFifo)
    {
        const auto& params = audioProcessor.getParameters();
        for(auto param: params){param->addListener(this);}
        updateChain();
        startTimerHz(60);
    };
    ~ResponseCurveComponent(){
        const auto& params = audioProcessor.getParameters();
        for(auto param: params){param->removeListener(this);}
    };
    juce::Atomic<bool> paramChanged{false};
    MonoChain monoChain;
    void parameterValueChanged (int parameterIndex, float newValue) override{paramChanged.set(true);};
    void parameterGestureChanged (int parameterIndex, bool gestureIsStarting) override {};
    
    void timerCallback() override
    {
//        juce::AudioBuffer<float> tempInBuffer;
        if(showFreqAnalysis){
            juce::Rectangle<float> bounds = getAnalysisArea().toFloat();
            auto fs = audioProcessor.getSampleRate();
            leftPathGenerator.process(bounds, fs);
            rightPathGenerator.process(bounds, fs);
        }
        if(paramChanged.compareAndSetBool(false, true)){
            //update chain and repaint
            updateChain();
            updateResponseCurve();
        }
        repaint();
    };
    void updateChain(){
        auto chainSettings = getChainSettings(audioProcessor.apvts);
        double sampleRate = audioProcessor.getSampleRate();
        auto peakCoefs = getPeakFilters(chainSettings, sampleRate);
        auto& chain = monoChain.get<ChainPos::Peaks>();
        updateCoefficients(chain.get<0>().coefficients, peakCoefs[0]);
        updateCoefficients(chain.get<1>().coefficients, peakCoefs[1]);
        updateCoefficients(chain.get<2>().coefficients, peakCoefs[2]);
        updateCoefficients(chain.get<3>().coefficients, peakCoefs[3]);
        updateCoefficients(chain.get<4>().coefficients, peakCoefs[4]);
        updateCoefficients(chain.get<5>().coefficients, peakCoefs[5]);
        updateCoefficients(chain.get<6>().coefficients, peakCoefs[6]);
        updateCoefficients(chain.get<7>().coefficients, peakCoefs[7]);
        auto coefs = getLowCutFilter(chainSettings, sampleRate);
        updateCutFilter(monoChain.get<ChainPos::LowCut>(), coefs, chainSettings.lowCutSlope);
        coefs = getHighCutFilter(chainSettings, sampleRate);
        updateCutFilter(monoChain.get<ChainPos::HighCut>(), coefs, chainSettings.highCutSlope);
    }
    void updateResponseCurve(){
        auto responseArea = getAnalysisArea();
        int w = responseArea.getWidth();
        auto& lowcut = monoChain.get<ChainPos::LowCut>();
        auto& peaks = monoChain.get<ChainPos::Peaks>();
        auto& highcut = monoChain.get<ChainPos::HighCut>();
        double sampleRate = audioProcessor.getSampleRate();
        std::vector<double> mags(w);
        for(int i = 0; i < w; i++){
            double mag = 1.f;
            auto freq = juce::mapToLog10<double>(double(i)/double(w), 20, 20000);
            if(!monoChain.isBypassed<ChainPos::Peaks>()){
                mag*= peaks.get<0>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
                mag*= peaks.get<1>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
                mag*= peaks.get<2>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
                mag*= peaks.get<3>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
                mag*= peaks.get<4>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
                mag*= peaks.get<5>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
                mag*= peaks.get<6>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
                mag*= peaks.get<7>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
            }
            if(!lowcut.isBypassed<0>()) mag*= lowcut.get<0>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
            if(!lowcut.isBypassed<1>()) mag*= lowcut.get<1>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
            if(!lowcut.isBypassed<2>()) mag*= lowcut.get<2>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
            if(!lowcut.isBypassed<3>()) mag*= lowcut.get<3>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
            if(!highcut.isBypassed<0>()) mag*= highcut.get<0>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
            if(!highcut.isBypassed<1>()) mag*= highcut.get<1>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
            if(!highcut.isBypassed<2>()) mag*= highcut.get<2>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
            if(!highcut.isBypassed<3>()) mag*= highcut.get<3>().coefficients->getMagnitudeForFrequency(freq, sampleRate);
            mags[i] = juce::Decibels::gainToDecibels(mag);
        }
        responseCurve.clear();
        
        const double outputMin = responseArea.getBottom();
        const double outputMax = responseArea.getY();
        
        auto map = [outputMin, outputMax](double input){
            return juce::jmap(input, -24.0, 24.0, outputMin, outputMax);
        };
        auto x = responseArea.getX();

        for(size_t i = 0; i < mags.size(); i++){
            if (InRange(mags[i])){
                if(responseCurve.isEmpty()) responseCurve.startNewSubPath(x+i,map(mags[i++]));
                responseCurve.lineTo(x+i, map(mags[i]));
            }
        }
        
    }
    void paint (juce::Graphics& g) override{
        auto bgColour = getLookAndFeel().findColour (juce::ResizableWindow::backgroundColourId);
        g.fillAll (bgColour);
        auto responseArea = getAnalysisArea();
        int x = responseArea.getX();
        int y = responseArea.getY();

//        g.drawImage(background, getLocalBounds().toFloat());
        g.setColour(bgColour.darker(0.3f));
        
        g.drawImage(background, responseArea.toFloat());
//        g.drawImageAt(background, x, y);
        drawBackGroundGrid(g);
        
        g.setColour (juce::Colours::white);
        g.setFont (15.0f);
        
        if(showFreqAnalysis){
            juce::Path FFTPath = leftPathGenerator.getPath();
//            FFTPath.applyTransform(juce::AffineTransform().translation(x,y));
            
            g.strokePath(FFTPath, juce::PathStrokeType(1.f));
            FFTPath = rightPathGenerator.getPath();
//            FFTPath.applyTransform(juce::AffineTransform().translation(x,y));
            g.strokePath(FFTPath, juce::PathStrokeType(1.f));
        }
        
        
        updateResponseCurve();
        g.setColour(juce::Colours::white);
        g.drawRoundedRectangle(responseArea.toFloat(), 4.f, 1.f);
        g.strokePath(responseCurve, juce::PathStrokeType(2.f));
        
        
//        juce::Path border;
//        border.setUsingNonZeroWinding(true);
//        border.addRoundedRectangle(getRenderArea(), 4);
//        g.setColour(juce::Colours::white);
//        g.fillPath(border);
        drawTextLabels(g);
        
        
    };
    void resized() override{
//        auto responseArea = getAnalysisArea();
////        background = juce::Image(juce::Image::PixelFormat::RGB, responseArea.getWidth(),responseArea.getHeight(),true);
////        juce::Image(<#PixelFormat format#>, <#int imageWidth#>, <#int imageHeight#>, <#bool clearImage#>)
//        juce::Graphics g(background);
//
////        auto bgLineColour = juce::Colour(123u,144u,196u).withAlpha(0.3f);
////        g.setColour(bgLineColour);
////        for(auto f : labelFreqs){
////            auto x = juce::mapFromLog10(f, 20.f, 20000.f);
////            g.drawVerticalLine(x * w, 0, h);
////        }
////        for(auto db : labelGains){
////            g.setColour((db==0)? (juce::Colours::white).withAlpha(0.8f) : bgLineColour);
////            auto y = juce::jmap(db,-24.f, 24.f, float(h), 0.f);
////            g.drawHorizontalLine(y, 0, w);
////        }
//        drawBackGroundGrid(g);
        responseCurve.preallocateSpace(getWidth()*3);
        updateResponseCurve();
    };
    void drawBackGroundGrid(juce::Graphics& g){
        
        auto responseArea = getAnalysisArea();
        auto h = responseArea.getHeight();
        auto w = responseArea.getWidth();
        int l = responseArea.getX();
        int r = responseArea.getRight();
        int t = responseArea.getY();
        int b = responseArea.getBottom();

        auto bgLineColour = juce::Colour(123u,144u,196u).withAlpha(0.3f);
        g.setColour(bgLineColour);
        for(auto f : labelFreqs){
            auto x = juce::mapFromLog10(f, 20.f, 20000.f);
//            g.drawVerticalLine(x * w, 0, h);
            g.drawVerticalLine(x * w + l, t, b);
        }
        for(auto db : labelGains){
            g.setColour((db==0)? (juce::Colours::white).withAlpha(0.5f) : bgLineColour);
            auto y = juce::jmap(db,-24.f, 24.f, float(b), float(t));
//            g.drawHorizontalLine(y, 0, w);
            g.drawHorizontalLine(y, l, r);
        }

        
    };
private:
    juce::Path responseCurve;
    bool showFreqAnalysis = true;
    const std::vector<float> labelFreqs{20,50,100,200,500,1000,2500,5000,10000,20000};
    const std::vector<float> labelGains{-24,-12,0,12,24};
    EQAudioProcessor& audioProcessor;
    juce::Image background;
    juce::Rectangle<int> getRenderArea(){
        auto bounds = getLocalBounds();
        bounds.removeFromTop(12);
        bounds.removeFromBottom(2);
        bounds.removeFromLeft(20);
        bounds.removeFromRight(20);
        return bounds;
    };
    juce::Rectangle<int> getAnalysisArea(){
        auto bounds = getRenderArea();
        bounds.removeFromTop(10);
        bounds.removeFromBottom(10);
        return bounds;
    };
    std::vector<float> getLogLabelPos(const std::vector<float> &labels,float left, float width,
                                      float minRange = 20.f, float maxRange = 20000.f){
        std::vector<float> pos;
        for(auto l: labels){
            auto normed = juce::mapFromLog10(l, minRange, maxRange);
            pos.push_back(left + width * normed);
        }
        return pos;
    };
    void drawTextLabels(juce::Graphics& g){
        g.setColour(juce::Colours::lightgrey);
        const int fontHeight = 10;
        g.setFont(fontHeight);
        
        auto area = getAnalysisArea();
        int left = area.getX();
        int top = area.getY();
        int bottom = area.getBottom();
        int width = area.getWidth();
        std::vector<float> pos = getLogLabelPos(labelFreqs, float(left), float(width));
        for(int i = 0; i < labelFreqs.size(); i++){
            float f = labelFreqs[i];
            float p = pos[i];
            bool addK = false;
            juce::String str;
            if(f > 999.f){
                addK = true;
                f /= 1000.f;
            }
            str << f << (addK? "k": "") << "Hz";
            
            juce::Rectangle<int> rec;
            rec.setSize(g.getCurrentFont().getStringWidth(str), fontHeight);
            rec.setCentre(p, top - fontHeight);
            
            g.drawFittedText(str, rec, juce::Justification::centred, 1);
        }
        for(float gain: labelGains){
            auto p = juce::jmap(gain, -24.f, 24.f, float(bottom), float(top));
            juce::String str;
            str << ( (gain > 0)? "+" : "" )  << gain;
            juce::Rectangle<int> rec;
            
            auto textWidth = g.getCurrentFont().getStringWidth(str);
            rec.setSize(textWidth, fontHeight);
            rec.setX(left + width + 1);
            rec.setCentre(rec.getCentreX(),p);
            if(gain == 0){
                rec.setCentre(rec.getCentreX() + g.getCurrentFont().getStringWidth(" "),rec.getCentreY());
                g.setColour( gain == 0.f ? juce::Colours::white : juce::Colours::lightgrey);
                
            }
            g.drawFittedText(str, rec, juce::Justification::centred, 1);
//            rec.setX(1);
        }
    };
    bool InRange(float val, float minVal = -24.f, float maxVal = 24.f){
        if (val > maxVal) return false;
        if(val < minVal) return false;
        return true;
    }
    SpectrumPathGenerator<juce::Path> leftPathGenerator, rightPathGenerator;
    

};



class EQAudioProcessorEditor  : public juce::AudioProcessorEditor
{
public:
    EQAudioProcessorEditor (EQAudioProcessor&);
    ~EQAudioProcessorEditor() override;

    //==============================================================================
    void paint (juce::Graphics&) override;
    void resized() override;
    //==============================================================================

private:
    // This reference is provided as a quick way for your editor to
    // access the processor object that created it.
    EQAudioProcessor& audioProcessor;
    
//    juce::Atomic<bool> paramChanged{false};
    // Sliders and Comboboxes
//    CustomRotarySlider peakFreqSlider[NUMPEAKFILTER], peakGainSlider[NUMPEAKFILTER], peakQSlider[NUMPEAKFILTER], lowCutFreqSlider,highCutFreqSlider;
    RotarySliderWithLabels peakFreqSlider[NUMPEAKFILTER], peakGainSlider[NUMPEAKFILTER], peakQSlider[NUMPEAKFILTER], lowCutFreqSlider,highCutFreqSlider;
//    std::unique_ptr<RotarySliderWithLabels> peakFreqSlider[NUMPEAKFILTER], peakGainSlider[NUMPEAKFILTER], peakQSlider[NUMPEAKFILTER], lowCutFreqSlider,highCutFreqSlider;
//    RotarySliderWithLabels* peakFreqSlider[NUMPEAKFILTER], peakGainSlider[NUMPEAKFILTER], peakQSlider[NUMPEAKFILTER], lowCutFreqSlider,highCutFreqSlider;
    CustomComboBox lowSlopeMenu, highSlopeMenu;
//    CustomRotarySlider lowSlopeMenu, highSlopeMenu;
    // Attachments
    using APVTS = juce::AudioProcessorValueTreeState;
    std::unique_ptr<APVTS::SliderAttachment> peakFreqSliderAttachment[NUMPEAKFILTER], peakGainSliderAttachment[NUMPEAKFILTER], peakQSliderAttachment[NUMPEAKFILTER],
        lowCutFreqSliderAttachment, highCutFreqSliderAttachment;
    std::unique_ptr<APVTS::ComboBoxAttachment> lowCutSlopeAttachment, highCutSlopeAttachment;
    
    void init_Attachments();
    
    std::vector<juce::Component*> getComps();
    void setPeakSliderBounds(juce::Rectangle<int> peakArea);
    
    ResponseCurveComponent responseCurveComponent;

    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EQAudioProcessorEditor)
};
