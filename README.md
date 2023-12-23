# COURSE: ECE-GY-6483-REAL TIME EMBEDDED SYSTEMS
# Embedded Challenge Name: The Embedded Gyrometer
## Team-(Group 14)
## Team Members: 
1. SRIRAM NARAYAN KOUSHIK CHITRAPU (sc9948) 
2. Weining Wu (ww2644)
3. Daichong Meng (dm5449)
4. Devashish Gawde (dg4015)

## Preparation/Required Parts:
1. Laptop/Computer with VS Code installed along with PlatformIO, C Compiler and Teleplot installation.
2. Link STM32F429 Discovery Board with built in gyroscope to the computer/laptop.
3. USB power bank for the board. (Not used in our demo, Laptop has been used for the power supply)
4. A way to store 20 seconds of velocity data, sampled at 0.5 second intervals => Stored the results in `linearVelocity[][]`

# STEPS/PROCEDURE:
## STEP 1: Create a new PlatformIO project
### GitHub Link: https://github.com/devashish-11/EmbeddedGyrometer_Group14.git
- Download the folder/zip file with name: `Embedded_Challenge_Group14`
- Unzip the `zip` file, ensure that the `proj.cpp` file and the entire `drivers` folder is present the `src` folder. 
- Include the `mbed_app.json` file in the home library of this project to print the float values.

## Step 2: Build, Upload and Execute
- (Design Description provided in the pdf report submitted along with it.)
- ***Note: Fix the board right under the knee, then start moving after uploading the build to measure distance.***
- Execute the proj.cpp file by 1st building it and then uploading the build onto the board.

# Results
### Video Link: https://www.youtube.com/watch?v=Vf7crkMIVsM
- You can view the results on the VS CODE TERMINAL printed at the Serial Monitor and also display of the same results on the Board's LCD Screen.
- Values Displayed
  1. Distance travelled during that 20 second limit, Overall Total distance travelled after the 20 second limit. 
  2. Step Counts made by the person while the gyrometer was moving as the person travelled.