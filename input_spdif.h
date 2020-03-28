
#ifndef input_spdif_nativ_h_
#define input_spdif_nativ_h_
#include "Resampler.h"
#include "Quantizer.h"
#include "Arduino.h"
#include "AudioStream.h"
#include "DMAChannel.h"
#include <arm_math.h>

//#define DEBUG_SPDIF_IN	//activates debug output

class AudioInputSPDIF : public AudioStream
{
public:
	AudioInputSPDIF(void) : AudioStream(0, NULL) { begin(); }
	~AudioInputSPDIF();
	virtual void update(void);
	void begin(void);
	void stop();
	double getBufferedTime() const;
	double getInputFrequency() const;
	bool isLocked() const;
	double getTargetLantency() const;
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
	Resampler _resampler;
	Quantizer* quantizer[2];
	arm_biquad_cascade_df2T_instance_f32 _bufferLPFilter;
	
	volatile double _bufferedTime;
	volatile double _lastValidInputFrequ;
	double _inputFrequency;
	double _targetLatencyS;	//target latency [seconds]
	const double _blockDuration=AUDIO_BLOCK_SAMPLES/AUDIO_SAMPLE_RATE; //[seconds] 
	const double _maxLatency=2.*_blockDuration;

#ifdef DEBUG_SPDIF_IN
	static volatile bool bufferOverflow;
#endif
};

#endif