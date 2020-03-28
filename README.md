# teensy-4-spdifIn
This library implements an asynchronous spdif input for the Teensy 4. The main class 'AudioInputSPDIF' is compatible with the teensy audio library and inherits from teensy's AudioStream object. However an AudioInputSPDIF will never have the 'update responsibility', but resamples data from an input buffer, when another object of the audio pipline calls the update_all- function. In practice this just means that an object like e.g. the spdif-output or i2s output must be present in the audio pipeline to make the input work. 'example_spdif.cpp' is a minimalistic example at which the spdif-output triggers the audio pipline. The spdif-input is tested with input frequencies: 44.1kHz, 48Khz, 88.2Khz, 96KHz 177.4kHz and 192kHz. 

The two other classes of the library are the Resampler and the Quantizer. Both classes are used by the spdif-input.

The resampler:

The resampler implements the algorithm described in the paper https://ccrma.stanford.edu/~jos/resample/resample.pdf. 'example_resampler.cpp' shows how it can be used without the spdif-input or the Teensy audio library. Here a 4kHz wave is resampled from 48kHz to 44.1kHz. The result can be copied from the serial monitor to the pyhton script eval_example_resampler.py, that generates the following figure. It shows the amplitude of the fourier tranform of the resampled signal.

![img0](https://github.com/alex6679/teensy-4-spdifIn/blob/master/imgs/4kHz_resampled.png)

The Quantizer:

The spdif-input uses the Quantizer to get the 16bit integer signal from the floating point output of the resampler. It can be used to add trianlge-shaped dither to the signal and to apply noise-shaping. Currently the noise-shaping filter is only available for 44.1kHz (Teensy audio lib!) and 48kHz. It's stand-alone usage is shown in example_noise_shaping.cpp. Here a 4kHz 32bit floating point signal is quantized to 16 bit integer. eval_example_noiseshaping.py can be used to visualize the results. The following image show the influence of the different configurations of the algorithm:

no noise-shaping and no dithering
![img1](https://github.com/alex6679/teensy-4-spdifIn/blob/master/imgs/noiseShapingOff_ditherOff.png)


no noise-shaping but dithered
![img2](https://github.com/alex6679/teensy-4-spdifIn/blob/master/imgs/noiseShapingOff_ditherOn.png)


noise-shaping and dithering applied:
![img3](https://github.com/alex6679/teensy-4-spdifIn/blob/master/imgs/noiseShapingOn_ditherOn.png)
