Espressif ESP32 program to generate a 1kHz modulation waveform for the HP8733B PIN modulator
===================================================================================================

The hardware is an Espressif ESP32-WROVER connected to an I2S PCM5102A digital to analog converter and then a voltage to current circuit.

Use of the HP 415E SWR meter and slotted line, require the test signal be AM modulated at 1kHz. (the crystal detector in the slotted line detects this modulation).

If the signal source does not generate this modulation, the HP8733B PIN modulator may be used to apply it.
The HP8733B requires a forward current to attenuate the signal. 

The attenuation of the HP8733B is logarithmic with diode current (approx 3dB per 0.1mA)

In order to amplitude modulate a signal, a corrective pre-distortion is applied to the modulating sinewave.

- ESP32 IO 22 - PCM5102A I2S BCK
- ESP32 IO 5  - PCM5102A I2S LRCK
- ESP32 IO 21 - PCM5102A I2S DIN

PCM5102A settings: 
- FLT  - Normal Latency (gnd)
- DEMP - Off (gnd)
- XMST - Soft un-mute (high)
- FMT  - I2S (gnd)
- PCM5102A SCK (gnd) (not used)

