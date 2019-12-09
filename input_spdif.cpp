
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)

#include <Arduino.h>
#include "input_spdif.h"

//Parameters
double AudioInputSPDIF::targetLatencyS= 0.001;
#define SPDIF_RX_BUFFER_LENGTH AUDIO_BLOCK_SAMPLES
const int32_t bufferLength=12*SPDIF_RX_BUFFER_LENGTH;
//==========

#ifdef DEBUG_SPDIF_IN
volatile bool AudioInputSPDIF::bufferOverflow=false;
#endif

int32_t AudioInputSPDIF::maxLatency;

Resampler AudioInputSPDIF::resampler;
BiQuad AudioInputSPDIF::bufferLPFilter;
Quantizer AudioInputSPDIF::quantizer[2];

volatile double AudioInputSPDIF::bufferedTime;
double AudioInputSPDIF::inputFrequency=1.;
volatile uint32_t AudioInputSPDIF::inputFrequOld=0;
volatile bool AudioInputSPDIF::firstHalfProcessed=false;

static int32_t spdif_rx_buffer[SPDIF_RX_BUFFER_LENGTH];
static float bufferR[bufferLength];
static float bufferL[bufferLength];

// only read/written in resample and update, which have the same priority/ do not interrupt each other
audio_block_t* AudioInputSPDIF::block_left= NULL;
audio_block_t* AudioInputSPDIF::block_right = NULL;
int32_t AudioInputSPDIF::block_offset = 0;
//====================================================================================================

volatile int32_t AudioInputSPDIF::buffer_offset = 0;	// read by resample/ written in spdif input isr -> copied at the beginning of 'resmaple' protected by __disable_irq() in resample
int32_t AudioInputSPDIF::resample_offset = 0; // read/written by resample/ read in spdif input isr -> no protection needed?

volatile bool AudioInputSPDIF::lockChanged=false;
volatile bool AudioInputSPDIF::locked=false;
DMAChannel AudioInputSPDIF::dma(false);

void AudioInputSPDIF::begin(void)
{
	dma.begin(true); // Allocate the DMA channel first
	const uint32_t noByteMinorLoop=2*4;
	dma.TCD->SOFF = 4;
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
	dma.TCD->NBYTES_MLNO = DMA_TCD_NBYTES_MLOFFYES_NBYTES(noByteMinorLoop) | DMA_TCD_NBYTES_SMLOE | 
						DMA_TCD_NBYTES_MLOFFYES_MLOFF(-8);
	dma.TCD->SLAST = -8;
	dma.TCD->DOFF = 4;
	dma.TCD->CITER_ELINKNO = sizeof(spdif_rx_buffer) / noByteMinorLoop;
	dma.TCD->DLASTSGA = -sizeof(spdif_rx_buffer);
	dma.TCD->BITER_ELINKNO = sizeof(spdif_rx_buffer) / noByteMinorLoop;
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;	
	dma.TCD->SADDR = (void *)((uint32_t)&SPDIF_SRL);
	dma.TCD->DADDR = spdif_rx_buffer;
	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SPDIF_RX);

	SPDIF_SCR |=SPDIF_SCR_DMA_RX_EN;		//DMA Receive Request Enable
	dma.enable();
	dma.attachInterrupt(isr);
	config_spdifIn();
#ifdef DEBUG_SPDIF_IN
	while (!Serial);
#endif
	resampler.init();
	bufferLPFilter=BiQuad(BiQuad::LOW_PASS, 0., 1., AUDIO_SAMPLE_RATE_EXACT/AUDIO_BLOCK_SAMPLES, 0.7);
}
void AudioInputSPDIF::spdif_interrupt(){
	if(SPDIF_SIS & SPDIF_SIS_LOCK){
		if (!locked){
			locked=true;
			lockChanged=true;
		}
	}
	else if(SPDIF_SIS & SPDIF_SIS_LOCKLOSS){
		if (locked){
			locked=false;
			lockChanged=true;
		}
	}
	SPDIF_SIC |= SPDIF_SIC_LOCKLOSS;//clear SPDIF_SIC_LOCKLOSS interrupt
	SPDIF_SIC |= SPDIF_SIC_LOCK;	//clear SPDIF_SIC_LOCK interrupt
}


void AudioInputSPDIF::resample(){
	if(!resampler.initialized() || block_offset>=AUDIO_BLOCK_SAMPLES){
		return;
	}
	__disable_irq();
	if(!locked || !resampler.initialized()){
		__enable_irq();
		return;
	}
	int32_t bOffset=buffer_offset;
	int32_t resOffset=resample_offset;
	__enable_irq();
		
	uint16_t inputBufferStop = bOffset >= resOffset ? bOffset-resOffset : bufferLength-resOffset;
	if (inputBufferStop==0){
		return;
	}
	uint16_t processedLength;
	uint16_t outputCount=0;
	uint16_t outputLength=AUDIO_BLOCK_SAMPLES-block_offset;
	
	float resampledBufferL[AUDIO_BLOCK_SAMPLES];
	float resampledBufferR[AUDIO_BLOCK_SAMPLES];
	resampler.resample(&bufferL[resOffset],&bufferR[resOffset], inputBufferStop, processedLength, resampledBufferL, resampledBufferR, outputLength, outputCount);
	
	quantizer[0].quantize(resampledBufferL, block_left->data+block_offset, outputCount);
	quantizer[1].quantize(resampledBufferR, block_right->data+block_offset, outputCount);

	resOffset=(resOffset+processedLength)%bufferLength;
	block_offset+=outputCount;	

	if (bOffset > resOffset && block_offset< AUDIO_BLOCK_SAMPLES){
		inputBufferStop= bOffset-resOffset;
		outputLength=AUDIO_BLOCK_SAMPLES-block_offset;
		resampler.resample(&bufferL[resOffset],&bufferR[resOffset], inputBufferStop, processedLength, resampledBufferL, resampledBufferR, outputLength, outputCount);
		
		quantizer[0].quantize(resampledBufferL, block_left->data+block_offset, outputCount);
		quantizer[1].quantize(resampledBufferR, block_right->data+block_offset, outputCount);
	
		resOffset=(resOffset+processedLength)%bufferLength;
		block_offset+=outputCount;
	}
	__disable_irq();
	resample_offset=resOffset;
	__enable_irq();	
}

void AudioInputSPDIF::isr(void)
{
	const int32_t *src, *end;
	uint32_t daddr = (uint32_t)(dma.TCD->DADDR);
	dma.clearInterrupt();

	if (daddr < (uint32_t)spdif_rx_buffer + sizeof(spdif_rx_buffer) / 2) {
		// DMA is receiving to the first half of the buffer
		// need to remove data from the second half
		src = (int32_t *)&spdif_rx_buffer[SPDIF_RX_BUFFER_LENGTH/2];
		end = (int32_t *)&spdif_rx_buffer[SPDIF_RX_BUFFER_LENGTH];
		firstHalfProcessed=false;
		//if (AudioInputSPDIF::update_responsibility) AudioStream::update_all();
	} else {
		// DMA is receiving to the second half of the buffer
		// need to remove data from the first half
		src = (int32_t *)&spdif_rx_buffer[0];
		end = (int32_t *)&spdif_rx_buffer[SPDIF_RX_BUFFER_LENGTH/2];
		firstHalfProcessed=true;
	}
	if (buffer_offset >=resample_offset ||
		(buffer_offset + SPDIF_RX_BUFFER_LENGTH/4) < resample_offset) {
		float *destR = &(bufferR[buffer_offset]);
		float *destL = &(bufferL[buffer_offset]);
		const float factor= pow(2., 23.)+1;
		do {			
			int32_t n=(*src) & 0x800000 ? (*src)|0xFF800000  : (*src) & 0xFFFFFF;
			*destL++ = (float)(n)/factor;
			++src;

			n=(*src) & 0x800000 ? (*src)|0xFF800000  : (*src) & 0xFFFFFF;
			*destR++ = (float)(n)/factor;
			++src;
		} while (src < end);
		buffer_offset=(buffer_offset+SPDIF_RX_BUFFER_LENGTH/4)%bufferLength;
	}
#ifdef DEBUG_SPDIF_IN
	else {
		bufferOverflow=true;
	}
#endif
}
double AudioInputSPDIF::getInputFrequ(){
	//page 2129: FrequMeas[23:0]=FreqMeas_CLK / BUS_CLK * 2^10 * GAIN
	if (SPDIF_SRPC & SPDIF_SRPC_LOCK){
		const uint32_t freqMeas=(SPDIF_SRFM & 0xFFFFFF);
		if (inputFrequOld != freqMeas){//frequency not stable yet;
			inputFrequOld=freqMeas;
			return -1.;	
		}
		const double f=(float)F_BUS_ACTUAL/(1024.*1024.*24.*128.);// bit clock = 128 * sampling frequency
		const double freqMeas_CLK= (float)(freqMeas)*f;
		return freqMeas_CLK;
	}
	return -1.;
}
void AudioInputSPDIF::allocateBlocks(){
	if(block_left){
		release(block_left);
		block_left=NULL;
	}
	if(block_right){
		release(block_right);
		block_right=NULL;
	}
	block_left= allocate();
	if (block_left!= NULL) {
		block_right = allocate();
		if (block_right == NULL) {
			release(block_left);
			block_left = NULL;
		}
	}
}

double AudioInputSPDIF::getBufferedTime() const{
	__disable_irq();
	double n=bufferedTime;
	__enable_irq();
	return n;
}

void AudioInputSPDIF::configure(){
	__disable_irq();
	if(!locked){
		__enable_irq();
#ifdef DEBUG_SPDIF_IN
		Serial.println("lock lost");
#endif
		return;
	}	
#ifdef DEBUG_SPDIF_IN
	const bool bOverf=bufferOverflow;
	bufferOverflow=false;
#endif
	const bool lc=lockChanged;
	__enable_irq();

#ifdef DEBUG_SPDIF_IN
	if (bOverf){
		Serial.print("buffer overflow, buffer offset: ");
		Serial.print(buffer_offset);
		Serial.print(", resample_offset: ");
		Serial.println(resample_offset);
		if (!resampler.initialized()){
			Serial.println("resampler not initialized. ");
		}
	}
#endif
	if (lc || !resampler.initialized()){
		const double inputF=getInputFrequ();	//returns: -1 ... invalid frequency
		if (inputF > 0.){
			__disable_irq();
			lockChanged=false;	//only reset lockChanged if a valid frequency was received (inputFrequ > 0.)
			__enable_irq();
			//we got a valid sample frequency
			const double frequDiff=inputF/inputFrequency-1.;
			if (abs(frequDiff) > 0.001 || !resampler.initialized()){
				//the new sample frequency differs from the last one -> configure the resampler again
				inputFrequency=inputF;		
				const int32_t targetLatency=round(targetLatencyS*inputF);
				maxLatency=AUDIO_BLOCK_SAMPLES*inputF/AUDIO_SAMPLE_RATE_EXACT+1;	//number of input samples needed to fill a block
				block_offset=0;
				__disable_irq();
				resample_offset =  targetLatency <= buffer_offset ? buffer_offset - targetLatency : bufferLength -(targetLatency-buffer_offset);
				__enable_irq();
				resampler.configure(inputF, AUDIO_SAMPLE_RATE_EXACT);
		#ifdef DEBUG_SPDIF_IN
				Serial.print("maxLatency: ");
				Serial.println(maxLatency);
				Serial.print("targetLatency: ");
				Serial.println(targetLatency);
				Serial.print("relative frequ diff: ");
				Serial.println(frequDiff, 8);
				Serial.print("configure resampler with frequency ");
				Serial.println(inputF,8);			
		#endif			
			}
		}
	}
}

void AudioInputSPDIF::monitorResampleBuffer(){
	__disable_irq();	
	if(!locked || !resampler.initialized()){
		__enable_irq();
		return;
	}
	int32_t dmaOffset = ((int32_t*)(dma.TCD->DADDR)-spdif_rx_buffer);
	if (dmaOffset >= SPDIF_RX_BUFFER_LENGTH/2 && firstHalfProcessed){
		dmaOffset-=SPDIF_RX_BUFFER_LENGTH/2;
	}
	dmaOffset/=2;

	int32_t noBufferedSamples = resample_offset <= buffer_offset ? buffer_offset-resample_offset+dmaOffset : bufferLength-resample_offset +buffer_offset+dmaOffset;
	const double inputFrequency=AUDIO_SAMPLE_RATE_EXACT*resampler.getStep();
	double neededSamplesForBlock=resampler.getStep()*AUDIO_BLOCK_SAMPLES;
	if (noBufferedSamples > maxLatency+neededSamplesForBlock || noBufferedSamples <= neededSamplesForBlock) {		
		const double targetLatency=targetLatencyS*inputFrequency;	
		noBufferedSamples=round(targetLatency+neededSamplesForBlock+resampler.getXPos());
		resample_offset=buffer_offset-noBufferedSamples+dmaOffset;
		while (resample_offset<0){
			resample_offset+=bufferLength;
		}	
		__enable_irq();
		bufferLPFilter.reset();
		resampler.fixStep();
#ifdef DEBUG_SPDIF_IN
		Serial.print("bufferRed, noBufferedSamplesDebug: ");
		Serial.println(noBufferedSamples);
#endif		
	}
	else {
		__enable_irq();
	}	
	double diff = (noBufferedSamples-resampler.getXPos())/inputFrequency- (AUDIO_BLOCK_SAMPLES/(double)AUDIO_SAMPLE_RATE_EXACT+ targetLatencyS);	//seconds
	diff= bufferLPFilter.process(diff);	//it's better to filter diff instead of e.g. sib, since all internal members of bufferLPFilter are set to 0 at a reset -> the filter settles faster 
	bool settled=resampler.addToSampleDiff(diff);
	if (settled && abs(diff) > 25* 1e-6){
		const double targetLatency=targetLatencyS*inputFrequency;
		__disable_irq();
		resample_offset=buffer_offset-round(targetLatency+neededSamplesForBlock)+dmaOffset;
		while (resample_offset<0){
			resample_offset+=bufferLength;
		}
		__enable_irq();
		bufferLPFilter.reset();

#ifdef DEBUG_SPDIF_IN
		Serial.println("step settled + latency fixed");
#endif
	}
	bufferedTime=targetLatencyS+diff;
}

void AudioInputSPDIF::update(void)
{
	configure();
	monitorResampleBuffer();	//important first call 'monitorResampleBuffer' then 'resample'
	allocateBlocks();
	if (block_left && block_right) {
		block_offset = 0;
		resample();
		if(block_offset < AUDIO_BLOCK_SAMPLES){
			memset(block_left->data+block_offset, 0, (AUDIO_BLOCK_SAMPLES-block_offset)*sizeof(int16_t)); 
			memset(block_right->data+block_offset, 0, (AUDIO_BLOCK_SAMPLES-block_offset)*sizeof(int16_t)); 
			block_offset = AUDIO_BLOCK_SAMPLES;
#ifdef DEBUG_SPDIF_IN	
			Serial.println("filled block with zeros");
#endif
		}
		block_offset = 0;// indicates that the blocks were transmitted
		transmit(block_left, 0);
		release(block_left);
		block_left=NULL;
		transmit(block_right, 1);
		release(block_right);
		block_right=NULL;	
	}
#ifdef DEBUG_SPDIF_IN
	else {		
		Serial.println("no filled blocks available");
	}
#endif
}

void AudioInputSPDIF::config_spdifIn(){
	//CCM Clock Gating Register 5, imxrt1060_rev1.pdf page 1145
	CCM_CCGR5 |=CCM_CCGR5_SPDIF(CCM_CCGR_ON); //turn spdif clock on - necessary for receiver!
	
	SPDIF_SCR |=SPDIF_SCR_RXFIFO_OFF_ON;	//turn receive fifo off 1->off, 0->on

	SPDIF_SCR&=~(SPDIF_SCR_RXFIFO_CTR);		//reset rx fifo control: normal opertation

	SPDIF_SCR&=~(SPDIF_SCR_RXFIFOFULL_SEL(3));	//reset rx full select
	SPDIF_SCR|=SPDIF_SCR_RXFIFOFULL_SEL(2);	//full interrupt if at least 8 sample in Rx left and right FIFOs

	SPDIF_SCR|=SPDIF_SCR_RXAUTOSYNC; //Rx FIFO auto sync on

	SPDIF_SCR&=(~SPDIF_SCR_USRC_SEL(3));	//No embedded U channel

    CORE_PIN15_CONFIG  = 3;  //pin 15 set to alt3 -> spdif input

	/// from eval board sample code
	//   IOMUXC_SetPinConfig(
	//       IOMUXC_GPIO_AD_B1_03_SPDIF_IN,        /* GPIO_AD_B1_03 PAD functional properties : */
	//       0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
	//                                                  Drive Strength Field: R0/6
	//                                                  Speed Field: medium(100MHz)
	//                                                  Open Drain Enable Field: Open Drain Disabled
	//                                                  Pull / Keep Enable Field: Pull/Keeper Enabled
	//                                                  Pull / Keep Select Field: Keeper
	//                                                  Pull Up / Down Config. Field: 100K Ohm Pull Down
	//                                                  Hyst. Enable Field: Hysteresis Disabled */
	CORE_PIN15_PADCONFIG=0x10B0;
	SPDIF_SCR &=(~SPDIF_SCR_RXFIFO_OFF_ON);	//receive fifo is turned on again


	SPDIF_SRPC &= ~SPDIF_SRPC_CLKSRC_SEL(15);	//reset clock selection page 2136
	//SPDIF_SRPC |=SPDIF_SRPC_CLKSRC_SEL(6);		//if (DPLL Locked) SPDIF_RxClk else tx_clk (SPDIF0_CLK_ROOT)
	//page 2129: FrequMeas[23:0]=FreqMeas_CLK / BUS_CLK * 2^10 * GAIN
	SPDIF_SRPC &=~SPDIF_SRPC_GAINSEL(7);	//reset gain select 0 -> gain = 24*2^10
	//SPDIF_SRPC |= SPDIF_SRPC_GAINSEL(3);	//gain select: 8*2^10
	//==============================================

	//interrupts
	SPDIF_SIE |= SPDIF_SIE_LOCK;	//enable spdif receiver lock interrupt
	SPDIF_SIE |=SPDIF_SIE_LOCKLOSS;

	lockChanged=true;
	attachInterruptVector(IRQ_SPDIF, spdif_interrupt);
	NVIC_SET_PRIORITY(IRQ_SPDIF, 208); // 255 = lowest priority, 208 = priority of update
	NVIC_ENABLE_IRQ(IRQ_SPDIF);

	SPDIF_SIC |= SPDIF_SIC_LOCK;	//clear SPDIF_SIC_LOCK interrupt
	SPDIF_SIC |= SPDIF_SIC_LOCKLOSS;//clear SPDIF_SIC_LOCKLOSS interrupt
	locked=(SPDIF_SRPC & SPDIF_SRPC_LOCK);
}
#endif

