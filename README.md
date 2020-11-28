# EE445L-ESP32-Tests

micSpeakerTest requires no configuration. It will pass ADC samples at 8k hz directly to the DAC. 

udpVOIP requires the RPORT and SPORT to be configured for each device near the top of the program. The RPORT for device 1 should be the SPORT on device 2 and vice versa. Wifi settings should also be set in the sdkconfig. This program will continuosly transmit adc samples and output received samples to the DAC.
