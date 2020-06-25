# ECLIB: Encrypted Controller Library

A C++ implementation for linear controllers to operate over encrypted data, using LWE-based cryptosystem.

## Documentation

[**Download Instruction Guide**](https://github.com/KimMinryoung/ECLIB/raw/master/Instruction_Manual_for_ECLIB.pdf)

## Overview

**ECLIB** provides modules for encrypted controller.

The main goal of this library is to give designers an easy way to implement an encrypted controller from their original linear SISO controller and simulate its performance. **ECLIB** decides proper parameters to satisfy user's desired performance and promise security, and build cryptosystem and encrypted controller according to the parameters.

**LWE-based cryptosystem** allows homomorphic encryption and arithmetics. **Homomorphic Encryption** refers to a type of encryption technology that allows computation to be directly on encrypted data, without requiring any decryption in the process. For a control system, if the controller is homomorphically encrypted, all the control operations are performed in encrypted state. So the controller doesn't have to know the secret key and plaintext of signals, thus the system attacker can't get informantion from controller access.

The structure of the control system that **ECLIB** will construct is depicted in the next image.

<img src="https://github.com/KimMinryoung/ECLIB/blob/master/controlsystem_eng.png"></img>

This library is composed of **three main modules**.
- `Encrypter`: Encrypt signals
- `Decrypter`: Decrypt signals
- `Encrypted Controller`: Performs control operation with encrypted data.

You can embed these modules into your devices to implement an encrypted controller.

Also, to help system implementation and test simulation, **ECLIB** provides **auxiliary modules**.
- `System Builder`: With user input, decide proper parameters for system and build it.
- `Plant`: Receives signal from actuator, updates plant state with the signal, and sends the plant output to the sensor.
- `Sensor`: Receives the plant output signal, encrypts it, and then sends it to the encrypted controller.
- `Actuator`: Receives controller output signal, decrypts it, and then sends it to the plant.

## How to Use It

### Implementing encrypted control system with real plant

There are codes of modules in **ECLIB** project folder. You can put them in your project or physical device by including each header file(.h) and source file(.cpp).

Please read the guide documentation above for detailed instructions.

[**Download Instruction Guide**](https://github.com/KimMinryoung/ECLIB/raw/master/Instruction_Manual_for_ECLIB.pdf)

You can also refer to the virtual code for plant, sensor, and actuator.

### Building a simulation

You can immediately start simulation of an example control system with `main.cpp`.

**Windows environment**
- Install **Visual Studio 2019**. In the Installing Visual Studio screen, choose the **Desktop development with C++** workload. We need to set up `MSVC v142 toolset` and `Windows 10 SDK(10.0.17134.0)` to build project.(If you missed that step, you should re-open Visual Studio Installer again, and **modify** Visual Studio 2019. Then you can install workload we need.)
- Open the project solution file `EncryptedSystem.sln` with Visual Studio 2019.
- To build this project, choose **Build Solution**(`F7`) from the **Build** menu.
- To run the code, on the menu bar, choose **Debug** -> **Start without debugging**(`Ctrl+F5`).

If you want to change the simulation model, please edit the corresponding files.

**For settings**
- **To set controller**: Revise `parameters.txt`.

- **To set plant**: Revise plant parameters and operation codes in `Plant.cpp`.

- **To set cryptosystem**: Revise `parameters.txt`.

### Input File: parameters.txt

 Following parameters are entries of the input file `parameters.txt`. Each entry in a line should be separated by a tab(\t).

**Controller Parameters**
 - **F,G,H,J**: state space matrices in following form of controller
 
 > x(t+1) = Fx(t) + Gy(t), 
  u(t) = Hx(t) + Jy(t).
 
 (x: controller state, u: controller output, y: plant output)
 
 - **T_s**: sampling time(second)
 
 - **r_y**: resolution of plant output sensor(e.g. if y=1.46, then 1/r_y=100)
 
 - **r_u**: resolution of actuator(e.g. if u=4.821, then 1/r_u=1000)
 
 - **U**: size bound of controller output(e.g. if possible output of controller range from -32 to 17(i.e. -32<=u(t)<=17), then U=32.)
 
 - **sigma**: standard deviation of Gaussian noise to inject to the ciphertext **(recommend 1.0 as value)**
 
 - **degrade_bound**: desired upper bound of performance degradation ratio due to injected noise
(e.g. if degrade_bound=0.05, then degradation ratio will be under 5%.)
