# Multi-Method Acoustic Leak Detection System

A complete acoustic pipeline leak detection system implemented on an ESP32-C6 microcontroller with a PCM5102A DAC for transmission and analog ADC for reception. The system uses Zadoff–Chu (ZC) chirp excitation and fuses multiple signal processing methods to achieve robust and repeatable leak localization in short pipelines.

This project is designed as a full end-to-end system: embedded signal generation, synchronized acquisition, binary data transfer, advanced MATLAB signal processing, and confidence-based fusion.

---

## System Summary

The system transmits a deterministic Zadoff–Chu chirp into a pipe using an acoustic transducer. Reflections and leak-generated echoes are captured by a hydrophone, digitized by the ESP32 ADC, and streamed as raw binary data to a host computer. MATLAB reconstructs the transmitted reference locally using a shared random seed and applies multiple detection methods whose results are fused into a final distance estimate.

Key goals of the design are reproducibility, timing determinism, and robustness against noise and distortion.

---

## Hardware Architecture

Main components:

- ESP32-C6 (FireBeetle 2) as the real-time controller
    
- PCM5102A I2S DAC for accurate waveform transmission
    
- ESP32 12-bit ADC (with option for external PCM180x series ADC)
    
- Piezo or underwater acoustic transducer for transmission
    
- Hydrophone or piezo sensor for reception
    

The DAC is driven in I2S master mode. ADC sampling is performed with microsecond-level timing control to maintain a fixed sampling rate.

Critical wiring constraints:

- PCM5102A FMT and XMT pins must be tied to ground
    
- Analog input must be AC-coupled using a DC blocking capacitor
    
- Analog wiring must be kept short to reduce noise pickup
    

---

## Communication Protocol

A lightweight command-based serial protocol is used at 921600 baud. All received samples are transmitted as raw binary for speed and determinism.

Supported commands:

- HELLO: connection test
    
- ZC : generate ZC chirp parameters
    
- TX: transmit chirp and capture ADC response
    
- STATUS: show current configuration
    
- TONE: output a 1 kHz test tone
    
- ADC: basic ADC health check
    

---

## Signal Design

The excitation signal is a real-valued Zadoff–Chu chirp with a sinusoidal carrier. The same seed is used on both ESP32 and MATLAB to ensure phase coherence between transmitted and reference signals.

Critical constraint:  
The center frequency must satisfy the Nyquist criterion. At a 20 kHz sampling rate, the center frequency is fixed at 5 kHz. Earlier versions using 30 kHz were invalid due to aliasing and phase distortion.

---

## MATLAB Processing Pipeline

Each received signal undergoes the following processing stages:

- DC offset removal
    
- High-pass filtering to remove low-frequency noise
    
- Amplitude normalization
    

Three independent detection methods are then applied:

1. Cross-correlation with ZC reference
    
2. Energy envelope detection
    
3. Phase-only frequency-domain correlation
    

Each method produces a time-delay estimate. Results are clustered, weighted by SNR, and fused based on agreement consistency.

---

## Fusion Strategy

Fusion confidence is determined by the standard deviation between method estimates:

- Low variance: weighted fusion
    
- Medium variance: simple averaging
    
- High variance: fallback to cross-correlation only
    

The final distance estimate accounts for round-trip propagation and the speed of sound in the selected medium.

---

## Key Parameters

Sampling rate: 20 kHz  
ZC length: 1000 samples  
Receive window: 2000 samples  
Center frequency: 5 kHz  
Measurements per run: 10  
Minimum valid measurements: 8

Speed of sound:

- Water: 1480 m/s
    
- Air: 343 m/s
    

---

## Validation Features

- Multi-measurement acquisition with statistical filtering
    
- SNR-based validity checks
    
- Cluster-based outlier rejection
    
- Automatic uncertainty estimation
    

Results include time delay, distance estimate, uncertainty bounds, method weights, and confidence level.

---

## Known Limitations

- ADC timing uses software delay loops and may jitter under heavy load
    
- Analog front-end lacks gain control and anti-alias filtering
    
- System assumes a single dominant reflection path
    

These limitations are intentional to keep the system minimal and transparent.

---

## File Structure

project/  
├── leak_detection_esp32.ino  
├── multi_method_leak_detection_fixed.m  
└── README.md
