# ECLIB: Encrypted Controller Library

A C++ implementation for linear controllers to operate over encrypted data, using LWE-based cryptosystem.

## Documentation

[**Download Instruction Guide**](https://github.com/KimMinryoung/Controller-Encryption/raw/master/Instruction_Manual_of_HECS.pdf)

## Overview

**ECLIB** library provides modules for encrypted controller.

The main goal of this library is to give designers an easy way to implement an encrypted controller from their original linear SISO controller and simulate its performance.

**LWE-based cryptosystem** allows homomorphic encryption and arithmetics. **Homomorphic Encryption** refers to a type of encryption technology that allows computation to be directly on encrypted data, without requiring any decryption in the process. For a control system, if the controller is homomorphically encrypted, all the control operations are performed in encrypted state. So the controller doesn't have to know the secret key and plaintext of signals, thus the system attacker can't get informantion from controller access.

<img src="https://github.com/KimMinryoung/Controller-Encryption/blob/master/controlsystem_eng.png"></img>

This library is composed of **three main modules**.
- `Encrypter`: Encrypt signals(and also encrypt controller matrices while building system)
- `Decrypter`: Decrypt signals
- `Encrypted Controller`: Performs control operation with encrypted data.

You can embed these modules into your devices to implement an encrypted controller.

Also, to help system implementation and test simulation, **ECLIB** provides auxiliary modules.
- `System Builder`: With user input, decide proper parameters for system and build it.
- `Plant`: Receives signal from actuator, updates plant state with the signal, and sends the plant output to the sensor.
- `Sensor`: Receives the plant output signal, encrypts it, and then sends it to the encrypted controller.
- `Actuator`: Receives controller output signal, decrypts it, and then sends it to the plant.

## How to Use It

[**Download Instruction Guide**](https://github.com/KimMinryoung/Controller-Encryption/raw/master/Instruction_Manual_of_HECS.pdf)

### Building a simulation

You can immediately start simulation of an example control system with `main.cpp`.

**Windows environment**
- Open the project solution file `EncryptedSystem.sln` with Visual Studio(version above 2017).
- To build this project, choose **Build Solution**(`F7`) from the **Build** menu.
- To run the code, on the menu bar, choose **Debug** -> **Start without debugging**(`Ctrl+F5`).

If you want to change the simulation model, please edit the corresponding files.

**For settings**
- **To set controller**: Revise `parameters.txt`.

- **To set plant**: Revise plant parameters and operation codes in `Plant.cpp`.

- **To set cryptosystem**: Revise `parameters.txt`.

### Implementing a real physical control system

There are codes of modules in \textbf{ECLIB} project folder. You can put them in your project or physical device by including each header file(.h) and source file(.cpp).

## Input File: parameters.txt

 Following parameters are entries of the input file `parameters.txt`. Each entry in a line should be separated by a tab(\t).

**Controller Parameters**
 - **F,G,H,J**: state space matrices in following form of controller
 
 > x(t+1) = Fx(t) + Gy(t), 
  u(t) = Hx(t) + Jy(t).
 
 (x: controller state, u: controller output, y: plant output)
 
 - **T_s**: sampling time(second)
 
 - **r_y**: resolution of plant output sensor(e.g. if y=1.46, then 1/r_y=100)
 
 - **r_u**: resolution of actuator(e.g. if u=4.821, then 1/r_u=1000)
 
 - **U**: size bound of controller output(e.g. if possible output of controller range from -32 to 17, then U=32)
 
 - **sigma**: standard deviation of Gaussian noise to inject to the ciphertext **(highly recommend 1.0 as value)**
 
 - **degrade_bound**: desired upper bound of performance degradation ratio due to injected noise **(recommend 0.01 as value)**
(e.g. if degrade_bound=0.05, then degradation ratio will be under 5%)

## Output File: result.txt
