# Controller-Encryption

A C++ implementation of homomorphic encryption of linear control system.

## Overview

Controller-Encryption library provides modules of LWE-based encrypted control system.

The main goal of this library is to give designers an easy way to implement an encrypted control system from their original linear SISO controller and simulate its performance.

This library is composed of encrypted system builder, LWE-encrypter, encrypted controller, sensor, actuator, and system simulation codes.

Homomorphic Encryption refers to a type of encryption technology that allows computation to be directly on encrypted data, without requiring any decryption in the process.

For a control system, if the controller is homomorphically encrypted, all the control operations are performed in encrypted state. So the controller doesn't have to know the secret key and plaintext of signals, thus the system attacker can't get informantion from controller access.

## How to Use It

Documentation

Open this project with Visual Studio and build an executable file.

You can immediately start simulation of an example control system with `main.cpp`.

**To set controller**: Revise parameters.txt.

**To set plant**: Revise plant parameters and operation codes in `Plant.cpp`.

**To set cryptosystem**: Revise parameters.txt.

## Input file: parameters.txt

 These are entries of the input file `parameters.txt`. Each entry in a line should be separated by a tab(\t).

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
