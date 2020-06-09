# ECLIB: Encrypted Controller Library

A C++ implementation for linear controllers to operate over encrypted data, using LWE-based cryptosystem.

## Overview

**ECLIB** library provides modules for encrypted controller.

The main goal of this library is to give designers an easy way to implement an encrypted control system from their original linear SISO controller and simulate its performance.

Homomorphic Encryption refers to a type of encryption technology that allows computation to be directly on encrypted data, without requiring any decryption in the process.

For a control system, if the controller is homomorphically encrypted, all the control operations are performed in encrypted state. So the controller doesn't have to know the secret key and plaintext of signals, thus the system attacker can't get informantion from controller access.

<img src="https://github.com/KimMinryoung/Controller-Encryption/blob/master/controlsystem_eng.png" width="75%"></img>

This library is composed of **three main modules**.
- Encrypter
- Decrypter
- Encrypted Controller

You can embed these modules into your devices to implement an encrypted controller.

Also, to help system implementation and test simulation, **ECLIB** provides auxiliary modules.
- System Builder
- Plant
- Sensor
- Actuator

## How to Use It

[**Documentation link**](https://github.com/KimMinryoung/Controller-Encryption/raw/master/Instruction_Manual_of_HECS.pdf)

### Building a simulation

**Windows environment**
- Open the project solution file `EncryptedSystem.sln` with Visual Studio(version above 2017).
- To build this project, choose **Build Solution**(`F7`) from the **Build** menu.
- To run the code, on the menu bar, choose **Debug** -> **Start without debugging**(`Ctrl+F5`).

You can immediately start simulation of an example control system with `main.cpp`.
 
**For settings**
- **To set controller**: Revise `parameters.txt`.

- **To set plant**: Revise plant parameters and operation codes in `Plant.cpp`.

- **To set cryptosystem**: Revise `parameters.txt`.

### Implementing a real physical control system



## Input file: parameters.txt

 Following parameters are entries of the input file `parameters.txt`. Each entry in a line should be separated by a tab(\t).

 - F,G,H,J: state space matrices in following form of controller
 
 > x(t+1) = Fx(t) + Gy(t),
 u(t) = Hx(t) + Jy(t).
 
 - T_s: sampling time(second)
 
 - r_y: plant output sensor resolution
 
 - r_u: actuator resolution
 
 - U: size of controller range
 
 - sigma: standard deviation of Gaussian noise
 
 - degrade: desired upper bound of performance degradation
 
 (ex: if degrade=1.8, then degradation ratio is under 10^(-1.8)=0.0158)
