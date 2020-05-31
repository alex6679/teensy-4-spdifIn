
#ifndef async_input_spdif3_h_
#define async_input_spdif3_h_
#include "Resampler.h"
#include "Quantizer.h"
#include "Arduino.h"
#include "AudioStream.h"
#include "DMAChannel.h"
#include <arm_math.h>

//#define DEBUG_SPDIF_IN	//activates debug output

class AsyncAudioInputSPDIF3 : public AudioStream
{
public:
	///@param attenuation target attenuation [dB] of the anti-aliasing filter. Only used if newFs<fs. The attenuation can't be reached if the needed filter length exceeds 2*MAX_FILTER_SAMPLES+1
	///@param minHalfFilterLength If newFs >= fs, the filter length of the resampling filter is 2*minHalfFilterLength+1. If fs y newFs the filter is maybe longer to reach the desired attenuation
	AsyncAudioInputSPDIF3(bool dither, bool noiseshaping,float attenuation, int32_t minHalfFilterLength);
	~AsyncAudioInputSPDIF3();
	virtual void update(void);
	void begin();
	void stop();
	double getBufferedTime() const;
	double getInputFrequency() const;
	bool isLocked() const;
	double getTargetLantency() const;
	double getAttenuation() const;
	int32_t getHalfFilterLength() const;
protected:	
	static DMAChannel dma;
	static void isr(void);
private:
	void resample(int16_t* data_left, int16_t* data_right, int32_t& block_offset);
	void monitorResampleBuffer();
	void configure();
	double getNewValidInputFrequ();
	void config_spdifIn();

	//accessed in isr ====
	static volatile int32_t buffer_offset;
	static int32_t resample_offset;
    static volatile uint32_t microsLast;
	//====================

	// spdif lock-changed interrupt
	static volatile bool locked;
	static volatile bool lockChanged;
	static volatile bool resetResampler;
  	static void spdif_interrupt();
	#ifdef MEASURE_FREQ
	static FrequencyMeasurement frequMeasure;	
	#endif
	//=============================
	float _attenuation;
	int32_t _minHalfFilterLength;
	Resampler _resampler;
	Quantizer* quantizer[2];
	arm_biquad_cascade_df2T_instance_f32 _bufferLPFilter;
	
	volatile double _bufferedTime;
	volatile double _lastValidInputFrequ;
	double _inputFrequency=0.;
	double _targetLatencyS;	//target latency [seconds]
	const double _blockDuration=AUDIO_BLOCK_SAMPLES/AUDIO_SAMPLE_RATE_EXACT; //[seconds] 
	double _maxLatency=2.*_blockDuration;

#ifdef DEBUG_SPDIF_IN
	static volatile bool bufferOverflow;
#endif
};

#endif