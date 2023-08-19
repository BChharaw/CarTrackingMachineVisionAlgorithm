# High Precision Car Tracking Machine Vision Algorithm

## Introduction
The aim of this project was to to develop a vehicle tracking system for the Toyota Innovation Challenge. This repository contains the submission of our group (Group 17). You can check out the final results of this code at [https://bchharaw.github.io/#/projects/cardetection].

## Requirements
The starting point of this project for all groups competeting under the C++ category was an Astra depth camera and the Astra SDK library. From there the groups were challenged to develop a system that could track moving cars at varying levels of difficulty. As a result to use this code, you will need the following:
- An Astra SDK compatible depth camera
- The Astra SDK and the associated desktop drivers

The unmodified SDK and drivers can be found at [https://drive.google.com/drive/folders/1ybOR5NLRfRJM09OUw7Nv3fQ8K34Sdg6A?usp=sharing].

## Installation
To install and run the code, follow these steps:
1. Clone the repository: `git clone https://github.com/BChharaw/CarTrackingMachineVisionAlgorithm.git`
2. Install the Astra SDK and the associated desktop drivers from the link provided above.
3. Replace the `main.cpp` file in the `AstraSDKvs2015-win64-silas (1)/AstraSDKvs2015-win64-silas/samples/sfml/SimpleDepthViewer-SFML` directory with the modified version from this repository.
4. Build and run the project according to the instructions in the original SDK documentation.


## Acknowledgements
This repository contains a modified version of the Depth Viewer library of the Orbbec Astra SDK [https://orbbec3d.com]. It is used in compliance with the Apache License, Version 2.0 as a part of the Toyota Innovation Challenge hosted by the University of Waterloo.

