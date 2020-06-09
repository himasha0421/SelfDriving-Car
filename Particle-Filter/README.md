![](images/Screenshot%20from%202020-06-09%2017-14-00.png)
## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Particle Filter


## Filter initialization


First we need to define the initial state of the state vector using the initial measurement . In this project I used initial measurement from the simulator to initialize the filter  x,Y,yaw_angle with additional gaussian mean noise .


For real car situations this is just like initialization using the GPS measurements initially to initialize the particle  filter.


First we initialize the number of particles we need  for robust localization and define each initial state using the initial gpu location and gpus location noise .


## Predict


In the predicted state we predict the particle's motion using a simple motion model . This prediction state predicts the particle motion based on the vehicle motion command . Then simply integrate the state transition information to all the particles . 


First stage updates the orientation angle with the given theta angle . After orientation update use appropriate motion equations to update the state of x and y. Add random gaussian noise due to the uncertainty associated with the real world motion not exactly mimicking the motion command as it is .


It’s important to make sure we are within 0-2*pi  limit for the yaw angle for that  i have used some if conditions .


## Update


In the update step we mainly focus on the weight update assigned to each and every particle to distinguish whether that particle is relevant or not . Initially we assign that uniform likelihood for every particle but with measurement updates we adjust the weights. Update step consists of mainly  three steps :


* Transform measurements from local coordinate system to map coordinate system


* Find likely matching map landmarks using neighbourhood search




* Using multivariate gaussian probability function we measure the weight of each particle


Step 01:
As the car moves it takes measurements using sensors but all the sensor measurements are  in the vehicle coordinate system . For landmarker matching we need to transform from local to map coordinate frame . For that we use below equation 

Step 02:
After transforming into the map coordinate frame we need to measure the most likely landmark from the map using neighbour search . Now each particle has it’s more likely measurements from the map .



Step 03:
using multivariate gaussian we can measure the likelihood of the measurements for the particle . If the measurements and the likely landmarks  with low euclidean distance error then that particle is more likely to represent the object . Hence weight goes high . Below equation shows the weight updating equation :




## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)



# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```


## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

