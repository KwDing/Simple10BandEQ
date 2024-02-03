/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
EQAudioProcessorEditor::EQAudioProcessorEditor (EQAudioProcessor& p)
    : AudioProcessorEditor (&p), audioProcessor (p), responseCurveComponent(p)
{
    // Make sure that before the constructor has finished, you've set the
    // editor's size to whatever you need it to be.
    
    for(int i = 0; i < NUMPEAKFILTER; i++){
        peakFreqSlider[i].labels.add({0.f, "20"});
        peakFreqSlider[i].labels.add({1.f, "20k"});
        peakQSlider[i].labels.add({0.f, "0.1"});
        peakQSlider[i].labels.add({1.f, "20"});
        peakGainSlider[i].labels.add({0.f, "-24dB"});
        peakGainSlider[i].labels.add({1.f, "+24dB"});
    }
    lowCutFreqSlider.labels.add({0.f, "20"});
    lowCutFreqSlider.labels.add({1.f, "20k"});
    highCutFreqSlider.labels.add({0.f, "20"});
    highCutFreqSlider.labels.add({1.f, "20k"});
    for(auto* comp:getComps()){
        addAndMakeVisible(comp);
    }
    init_Attachments();
//    addAndMakeVisible(lowSlopeMenu);
//    addAndMakeVisible(highSlopeMenu);
    
    setSize (1000, 500);
    
}

EQAudioProcessorEditor::~EQAudioProcessorEditor()
{
//    const auto& params = audioProcessor.getParameters();
//    for(auto param: params){param->removeListener(this);}
}

//==============================================================================
void EQAudioProcessorEditor::paint (juce::Graphics& g)
{
    // (Our component is opaque, so we must completely fill the background with a solid colour)
    g.fillAll(getLookAndFeel().findColour (juce::ResizableWindow::backgroundColourId));

}

void EQAudioProcessorEditor::resized()
{
    // This is generally where you'll want to lay out the positions of any
    // subcomponents in your editor..
    auto bounds = getLocalBounds();
    
    auto responseArea = bounds.removeFromTop(bounds.getHeight()*0.5);
    responseCurveComponent.setBounds(responseArea);
    auto lowCutArea = bounds.removeFromLeft(bounds.getWidth()*0.1);
    auto highCutArea = bounds.removeFromRight(bounds.getWidth()*0.1);
    lowCutFreqSlider.setBounds(lowCutArea.removeFromTop(lowCutArea.getHeight()*0.5));
//    lowCutFreqSlider->setBounds(lowCutArea.removeFromTop(lowCutArea.getHeight()*0.5));
    
    lowSlopeMenu.setBounds(lowCutArea.getTopLeft().x,lowCutArea.getTopLeft().y, lowCutArea.getWidth(), 20);
//    lowSlopeMenu.setBounds(lowCutArea);
    highCutFreqSlider.setBounds(highCutArea.removeFromTop(highCutArea.getHeight()*0.5));
//    highCutFreqSlider->setBounds(highCutArea.removeFromTop(highCutArea.getHeight()*0.5));
    
    highSlopeMenu.setBounds(highCutArea.getTopLeft().x,highCutArea.getTopLeft().y, highCutArea.getWidth(), 20);
//    highSlopeMenu.setBounds(highCutArea);
    setPeakSliderBounds(bounds);

    
    
}



std::vector<juce::Component*> EQAudioProcessorEditor::getComps(){
    std::vector<juce::Component*> comps;
    auto& vts = audioProcessor.apvts;
    for(int i = 0; i < NUMPEAKFILTER; i++){
        juce::String name;
        name << "Peak" << i+1 << " ";
        peakFreqSlider[i].set(vts.getParameter(name + "Freq"), "Hz");
        peakFreqSlider[i].sliderType = SliderType::Freq;
        peakQSlider[i].set(vts.getParameter(name + "Q"), "");
        peakQSlider[i].sliderType = SliderType::Q;
        peakGainSlider[i].set(vts.getParameter(name + "Gain"), "db");
        peakGainSlider[i].sliderType = SliderType::Gain;
        std::vector<juce::Component*> temp{&peakFreqSlider[i], &peakGainSlider[i], &peakQSlider[i]};
//        peakFreqSlider[i] = new RotarySliderWithLabels(*audioProcessor.apvts.getParameter("Peak1 Freq"), "Hz");
//        std::vector<juce::Component*> temp{peakFreqSlider[i], peakGainSlider[i], peakQSlider[i]};
        comps.insert(comps.end(), temp.begin(), temp.end());
    }
    lowCutFreqSlider.sliderType = SliderType::Freq;
    lowCutFreqSlider.set(vts.getParameter("LowCut Freq"),"Hz");
    highCutFreqSlider.sliderType = SliderType::Freq;
    highCutFreqSlider.set(vts.getParameter("HighCut Freq"), "Hz");
    
    comps.push_back(&lowCutFreqSlider);
    comps.push_back(&highCutFreqSlider);
    comps.push_back(&responseCurveComponent);
    comps.push_back(&lowSlopeMenu);
    comps.push_back(&highSlopeMenu);
    return comps;
}
void EQAudioProcessorEditor::setPeakSliderBounds(juce::Rectangle<int> peakArea){
    auto bounds = peakArea;
//    peakFreqSlider[0].setBounds(bounds.removeFromTop(bounds.getHeight()*0.33));
//    peakGainSlider[0].setBounds(bounds.removeFromTop(bounds.getHeight()*0.5));
//    peakQSlider[0].setBounds(bounds);
    auto widthPerBand = bounds.getWidth() / NUMPEAKFILTER;
    for(int i = 0; i < NUMPEAKFILTER; i++){
        auto peakBound = bounds.removeFromLeft(widthPerBand);
        peakFreqSlider[i].setBounds(peakBound.removeFromTop(peakBound.getHeight()*0.33));
        peakGainSlider[i].setBounds(peakBound.removeFromTop(peakBound.getHeight()*0.5));
        peakQSlider[i].setBounds(peakBound);
    }
}
void EQAudioProcessorEditor::init_Attachments(){
    auto & vts = audioProcessor.apvts;
    for(int i = 0; i < NUMPEAKFILTER; i++){
        juce::String name;
        name << "Peak" << i+1 << " ";
        peakFreqSliderAttachment[i].reset(new APVTS::SliderAttachment(vts, name + "Freq", peakFreqSlider[i]));
        peakGainSliderAttachment[i].reset(new APVTS::SliderAttachment(vts, name + "Gain", peakGainSlider[i]));
        peakQSliderAttachment[i].reset(new APVTS::SliderAttachment(vts, name + "Q", peakQSlider[i]));
    }
    lowCutFreqSliderAttachment.reset(new APVTS::SliderAttachment(vts, "LowCut Freq", lowCutFreqSlider));
    highCutFreqSliderAttachment.reset(new APVTS::SliderAttachment(vts, "HighCut Freq", highCutFreqSlider));
    lowCutSlopeAttachment.reset(new APVTS::ComboBoxAttachment(vts, "LowCut Slope", lowSlopeMenu));
    highCutSlopeAttachment.reset(new APVTS::ComboBoxAttachment(vts, "HighCut Slope", highSlopeMenu));
//    lowCutSlopeAttachment.reset(new APVTS::SliderAttachment(vts, "LowCut Slope", lowSlopeMenu));
//    highCutSlopeAttachment.reset(new APVTS::SliderAttachment(vts, "HighCut Slope", highSlopeMenu));
        
}
void LookAndFeel::drawRotarySlider (juce::Graphics& g, int x, int y, int width, int height,
                       float sliderPosProportional, float rotaryStartAngle,
                       float rotaryEndAngle, juce::Slider& slider) {
    auto bounds = juce::Rectangle<float>(x,y,width,height);
    g.setColour(juce::Colour(142u, 179u, 212u));
    g.fillEllipse(bounds);
    g.setColour(juce::Colour(118u, 145u, 202u));
    g.drawEllipse(bounds, 5.f);
    
    if(auto* ptr2slider = dynamic_cast<RotarySliderWithLabels*> (&slider)){
        auto center = bounds.getCentre();
        juce::Path p;
        
        juce::Rectangle<float> rec;
        rec.setLeft(center.getX()-2);
        rec.setRight(center.getX()+2);
        rec.setTop(bounds.getY());
//        rec.setBottom(center.getY() - ptr2slider->getTextHeight()*1.5);
//        rec.setBottom(center.getY());
        rec.setBottom(bounds.getY() + ptr2slider->getTextHeight()*1.5);
        p.addRoundedRectangle(rec, 2.f);
        jassert(rotaryStartAngle < rotaryEndAngle);
        
        g.setColour(juce::Colour(73u, 96u, 210u));
        auto sliderAngRad = juce::jmap(sliderPosProportional, 0.f, 1.f, rotaryStartAngle, rotaryEndAngle);
        p.applyTransform(juce::AffineTransform().rotated(sliderAngRad, center.getX(), center.getY()));
        g.fillPath(p);
        
        g.setFont(ptr2slider->getTextHeight());
        auto text = ptr2slider->getDisplayString();
        auto strWidth = g.getCurrentFont().getStringWidth(text);
        rec.setSize(strWidth+4, ptr2slider->getTextHeight()+2);
        rec.setCentre(bounds.getCentre());
        g.setColour(juce::Colours::white);
//        g.drawRect(rec);
//        g.setColour(juce::Colours::white);
        g.drawFittedText(text, rec.toNearestInt(), juce::Justification::centred, 1, 0.0f);
//        g.drawFittedText(const String &text, Rectangle<int> area, Justification justificationFlags, int maximumNumberOfLines);
        
    }
    
    
}
