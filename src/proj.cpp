//=======================================================================================
// REQUIRED HEADER FILES:
//=======================================================================================
#include <mbed.h>                                           //IMPORTING THE MBED HAL FRAMEWORK INTO THE CODE
#include <stdio.h>                                          //IMPORTING THE STDIO.H HEADER FILE
#include <math.h>                                           //IMPORTING THE MATH.H HEADER FILE
#include <chrono>                                           //IMPORTING THE CHRONO HEADER FILE
#include "stm32f4xx_hal.h"                                  //IMPORTING THE STM32F4 HEADER FILE (Additional Features of HAL).
#include "drivers/LCD_DISCO_F429ZI.h"                       //IMPORTING the STM32F29 LCD-DISPLAY FILE.
#include <stdlib.h>                                         //IMPORTING THE STDLIB HEADER FILE
#include <float.h>                                          //IMPORTING THE FLOAT HEADER FILE                                


//=======================================================================================
// GYROSCOPE CONFIGURATION ESSENTIALS:
//=======================================================================================

// OUTPUT REGISTER MAPPING FOR GYROSCOPE: 
#define CTRL_REG1 0x20                                       // CTRL_REG1 REGISTER ADDRESS
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1                     // OUTPUT DATA RATE (ODR) SELECTION = 01  [190 Hz]
                                                             // BANDWIDTH SELECTION = 10  [Cut-Off 50]
                                                             // POWER DOWN MODE = 1 [NORMAL MODE]
                                                             // Z-AXIS ENABLE = 1
                                                             // Y-AXIS ENABLE = 1
                                                             // X-AXIS ENABLE = 1

#define CTRL_REG4 0x23                                       // CTRL_REG4 REGISTER ADDRESS
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0                     // BLOCK DATA UPDATE (BDU) = 0  [CONTINUOUS UPDATE]
                                                             // BIG/LITTLE ENDIAN SELECTION = 0  [DATA AT LSB; LOWER ADDRESS]
                                                             // FULL SCALE SELECTION = 01 [500DPS]
                                                             // NO SELECTION = 000
                                                             // SPI SERIAL INTERFACE MODE SELECTION = 0 [4-WIRE INTERFACE]
                                                            
#define CTRL_REG3 0x22                                       // CTRL_REG3 REGISTER ADDRESS
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000                     // INTERRUPT ENABLE FOR INT1 = 0  [DISABLED]
                                                             // BOOT STATUS FOR INT1 = 0  [DISABLED]
                                                             // INTERRUPT ACTIVE CONFIG FOR INT1 = 0  [HIGH]
                                                             // PUSH-PULL/OPEN DRAIN CONFIG = 0 [PUSH-PULL]
                                                             // DATA READY ON INT2 = 1 [ENABLED]
                                                             // FIFO watermark interrupt on DRDY/INT2 = 0 [DISABLED]
                                                             // FIFO overrun interrupt on DRDY/INT2 = 0 [DISABLED]
                                                             // FIFO empty interrupt on DRDY/INT2 = 0 [DISABLED]

#define OUT_X_L 0x28                                         // X-AXIS ANGULAR DATA RATE ADDRESS 


//=======================================================================================
//GYROSCOPE STATES:
#define IDLE 0                                           // WHEN IDLE
#define MOVING_DATARECORD 1                              // WHEN MOVING [WALK/JOGG/RUN]
//=======================================================================================


//=======================================================================================
// GYROSCOPE INTERRUPT CONFIGURATION:
//=======================================================================================
//We are using the Ticker Timer Interrupt in the code to check at every 0.5seconds.
//Additionally the designer can also configure these registers of the gyroscope if needed to be more specific, which are as follows (From the I3G4250D datasheet):
// 1. INT1_CFG (30h)				
// 2. INT1_SRC (31h)				
// 3. INT1_THS_XH (32h)			
// 4. INT1_THS_XL (33h)			
// 5. INT1_THS_YH (34h)			
// 6. INT1_THS_YL (35h)			
// 7. INT1_THS_ZH (36h)			
// 8. INT1_THS_ZL (37h)			
// 9. INT1_DURATION (38h)			
//We have taken the readings of these 3 dimensions and used a particular threshold  for each in order to transition in between the states.


//=======================================================================================
// CUSTOMIZABLE CONSTANTS FOR THE CODE: [USER-DEFINED CONSTANTS]
//=======================================================================================
#define ScalingFactor (1.0f* 0.017453292519943295769236907684886f / 1000.0f)            // SCALING FACTOR FOR ANGULAR TO DEGREE CONVERSION
#define Radius 0.5f                                                                     // RADIUS IN METERS
#define WINDOW_SIZE 6                                                                   // MAXIMUM INTEGRATING LIMIT FOR EACH X,Y,Z CO-ORDINATE LINEAR VELOCITY
#define DUR_SAMPLE_COUNT 40                                                             // TOTAL SAMPLE COUNT = 40 (SAMPLE AT 0.5s FOR 20s)
#define DIM_COUNT 3                                                                     // DIMENSIONS COUNT = 3 [X,Y,Z]
#define RESET_TIMERLIMIT 20                                                             // RESET TIMER CONFIG (RESET FOR EVERY 20s)
#define TICKER_LIMIT 500ms                                                              // TICKER LIMIT = 0.5s (SAMPLE GYROSCOPE VALUES AT EVERY 0.5s)
#define GRYOMULFACTOR1 1000000                                                          // GYROSCOPE MULTIPLICATION FACTOR 1 FOR PROPER SCALING OF INDIVIDUAL CO-ORDINATE DISTANCE VALUE 
#define GRYOMULFACTOR2 100                                                              // GYROSCOPE MULTIPLICATION FACTOR 2 FOR PROPER SCALING OF RESULTANT DISTANCE VALUE 
#define GYRO_THRESHOLD 104.85f                                                          // GYROSCOPE THRESHOLD DISTANCE TO TRANSITION FROM THE IDLE STATE INTO MOVING_DATARECORD STATE


//=======================================================================================
// INITIALIZING THE CODE VARIABLES
//=======================================================================================
int i_cnt=0;                                                          // Global variable for iteratively storing the angular velocity of each co-ordinate into 'angularVelocity' memory array
int j_cnt=0;                                                          // Global variable for iteratively storing the linear velocity of each co-ordinate into 'linearVelocity' memory array
float angularVelocity[DUR_SAMPLE_COUNT][DIM_COUNT];                   // Angular Velocity Memory array 40x3 (Data points over 20s duration with 0.5s sampling rate = 40, Dimensions = 3)
float linearVelocity[DUR_SAMPLE_COUNT][DIM_COUNT];                    // Linear Velocity Memory array 40x3 (Data points over 20s duration with 0.5s sampling rate = 40, Dimensions = 3)
volatile int8_t state_chk = IDLE;                                     // Global declaration for state variable
volatile float totalDist = 0.0f;                                      // Global variable declaration for total distance travelled so far
int8_t step_cnt=0;                                                    // Global variable declaration for total step count so far
float window_gx[WINDOW_SIZE] = {0};                                   // Temporary register to store current x co-ordinate linear velocity value. Initializing the register with all values as '0'
float window_gy[WINDOW_SIZE] = {0};                                   // Temporary register to store current y co-ordinate linear velocity value. Initializing the register with all values as '0'
float window_gz[WINDOW_SIZE] = {0};                                   // Temporary register to store current z co-ordinate linear velocity value. Initializing the register with all values as '0'
int window_index = 0;                                                 // Global index variable declaration for the temporary register
float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;     // Global variables to store filtered linear velocity values if we use LPF Low Pass Filter
volatile float gX_ref=0.0f, gY_ref=0.0f, gZ_ref=0.0f;                 // Global variables for intial distance reference points for all 3 co-ordinates. Distance in meters 


//=======================================================================================
//  DECLARING A FILE TO OUTPUT THE STORED LINEAR-VELOCITIES ONTO A CSV FILE:
//FILE *file = fopen("output.csv", "w");
//=======================================================================================


//=======================================================================================
// DECLARING SEMAPHORE:
//Semaphore sem(1); //1 for available to share      //wait -> acquire,  signal -> release
//=======================================================================================


//=======================================================================================
// TIME HAL DEFINITIONS:
//=======================================================================================

Ticker t;             // TICKER HAL TO CALL FUNCTION BLOCK AT EACH SAMPLE POINT OF 0.5s
Timer resetTimer;     // TIMER HAL TO RESET THE CODE LOGIC ONCE AFTER 20s DURATION 


//=======================================================================================
// SPI INTERFACE PIN CONFIGURATIONS:
//=======================================================================================
EventFlags flags;                        // FLAG DECLARATION
#define MOSI_PIN PF_9                    // MASTER OUT SLAVE IN PIN CONFIG  
#define MISO_PIN PF_8                    // MASTER IN SLAVE OUT PIN CONFIG
#define SCLK_PIN PF_7                    // SERIAL CLOCK PIN CONFIG
#define CS_PIN  PC_1                     // CHIP SELECT PIN CONFIG [DEFAULT:0]
#define SPI_FLAG 1
#define DATA_READY_FLAG 2                // FLAG DEFINITIONS
#define WRITELIMIT_SIZE 2                // SPI TRANSFER WRITE LIMIT IN BYTES
#define READLIMIT_SIZE 2                 // SPI TRANSFER READ LIMIT IN BYTES
#define WRITELIMIT_SIZE1 7
#define READLIMIT_SIZE1 7


uint8_t write_buf[32];                   // WRITE BUFFER ARRAY
uint8_t read_buf[32];                    // READ BUFFER ARRAY
volatile int flag = 0;                   // VOLATILE FLAG DECLARATION
float result[3];                         // ARRAY FOR STORING LINEAR VELOCITIES [X,Y,Z] FOR EACH SAMPLE


//SPI INITIALIZATION:
SPI spi(MOSI_PIN, MISO_PIN, SCLK_PIN, CS_PIN, use_gpio_ssel);  //MOSI, MISO, SCLK, CS, SEL_CONFIG (CAN BE CONFIGURED IN CODE USING "spi.select()" and "spi.deselect()")


// CALL BACK FUNCTION TO SET SPI FLAG (TO INDICATE SPI COMMENCEMENT / SPI TRANSFER COMPLETE)
void spi_cb(int event)
{
    //sem.acquire();
    flags.set(SPI_FLAG);                // SET SPI FLAG
    //sem.release();
}


//CALLBACK FUNCTION FOR THE TICKER HAL:
void cb()
{
    spi_cb(0); // CALL 'spi_cb' WITH DEFAULT EVENT VALUE
}


// DATA READY CALLBACK FUNCTION TO SET DATA READY FLAG:
void data_cb()
{
    //sem.acquire();
    flags.set(DATA_READY_FLAG);
    //sem.release();
}


//=======================================================================================
// LCD ESSENTIALS:
//=======================================================================================
#define BACKGROUND 1
#define FOREGROUND 0

// LCD OBJECT:
LCD_DISCO_F429ZI lcd;

// SETUP OF LCD DISPLAY BACKGROUND LAYER 
void setup_background_layer(){
  lcd.SelectLayer(BACKGROUND);             // Function to set up the background layer on an LCD display  
  lcd.Clear(LCD_COLOR_BLACK);              // Clear the background layer with black color
  lcd.SetBackColor(LCD_COLOR_BLACK);       // Set the background color to black
  lcd.SetTextColor(LCD_COLOR_GREEN);       // Set the text color to green for the background layer
  lcd.SetLayerVisible(BACKGROUND,ENABLE);  // Make the background layer visible
  lcd.SetTransparency(BACKGROUND,0x7Fu);   // Set the transparency of the background layer (0x7F is a common value for semi-transparency)
}

// SETUP OF LCD DISPLAY FOREGROUND LAYER 
void setup_foreground_layer(){
    lcd.SelectLayer(FOREGROUND);            // Select the foreground layer
    lcd.Clear(LCD_COLOR_BLACK);             // Clear the foreground layer with black color
    lcd.SetBackColor(LCD_COLOR_BLACK);      // Set the background color to black for the foreground layer
    lcd.SetTextColor(LCD_COLOR_LIGHTGREEN); // Set the text color to light green for the foreground layer
}

// Function Prototype/Declaration:
void Initial_ScreenDisp();
void CALC_ScreenDisp(float totalDist);

// UI CONFIGURATION:
// Function to display the initial screen on an LCD
void Initial_ScreenDisp()
{
  lcd.Clear(LCD_COLOR_BLACK);                                                    // Clear the LCD screen with a black color
  lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"ECE 6483", CENTER_MODE);           // Display the string "ECE 6483" at line 4, centered on the screen
  lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Final Challenge", CENTER_MODE);    // Display the string "Final Challenge" at line 6, centered on the screen
  lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"Embedded Gyrometer", CENTER_MODE); // Display the string "Embedded Gyrometer" at line 7, centered on the screen
  lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"Need for Speed", CENTER_MODE);     // Display the string "Need for Speed" at line 8, centered on the screen
  lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"Group: 14", CENTER_MODE);         // Display the string "Group: 14" at line 10, centered on the screen
  lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"Auto Starting...", CENTER_MODE);  // Display the string "Auto Starting..." at line 15, centered on the screen
  HAL_Delay(1500);                                                               // Pause for 1500 milliseconds (1.5 seconds) using HAL_Delay
}


// Function to display current distance and step count on an LCD while in moving state
void CALC_ScreenDisp(float totalDist, int8_t stepcnt)      
{
//HAL_Delay(20);
float distance = totalDist;      // Pass to Local variable 'distance'
lcd.Clear(LCD_COLOR_BLACK);

char distance_buf[50];           // Declare a character array to hold the formatted distance string with size 50
char stepcnt_buf[50];            // Declare a character array to hold the formatted step count string with size 50


// Display a message indicating that computation is in progress for 20 seconds on LCD
lcd.DisplayStringAt(LINE(0),LINE(10), (uint8_t *)"Computing for 20 sec...", CENTER_MODE);
snprintf(distance_buf, 50, "Current Calc: %.3f m", distance);   // To display current distance message and store the string into 'distance_buf'
snprintf(stepcnt_buf, 50, "Current Step Cnt: %d", stepcnt);     // To display current step count message and store the string into 'stepcnt_buf'


// Display the current distance and current step count string at specific postion on the LCD screen
lcd.DisplayStringAt(LINE(0),LINE(10), (uint8_t *)distance_buf, CENTER_MODE); //DISPLAYING Distance Calculation.
lcd.DisplayStringAt(LINE(0),LINE(11), (uint8_t *)stepcnt_buf, CENTER_MODE); // Displaying the Step Cnt Taken while moving.
}


void CALC_Final_ScreenDisp(float totalDist, int8_t stepcnt)     // Function to display final calculated total distance and final total step count on the LCD covered for 20s duration.
{
float distance = totalDist;                                     // Passing by value to the local variable 'distance'

char distance_buf[50];                                          // Declare a character array to hold the formatted distance string
char stepcnt_buf[50];                                           // Declare a character array to hold the formatted step count string

// Display a message on LCD to show final total distance value and final step count covered in 20s duration
snprintf(distance_buf, 50, "Distance: %.2f m", distance); //Distance String
snprintf(stepcnt_buf, 50, "Step Count: %d", stepcnt); //Step Cnt String

lcd.Clear(LCD_COLOR_BLACK);

// Display the final total distance and final step count string at specific postion on the LCD screen
lcd.DisplayStringAt(LINE(0),LINE(8), (uint8_t *)distance_buf, CENTER_MODE); //Distance Display on LCD.
lcd.DisplayStringAt(LINE(0),LINE(9), (uint8_t *)stepcnt_buf, CENTER_MODE); //Step Cnt Display on LCD.

//Additional Data Display on the screen to make it more user friendly:
lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"[Result in 20 sec]", CENTER_MODE);

//Additional Instruction for reset to be displayed on LCD after covering the 20s duration:
lcd.DisplayStringAt(0, LINE(13), (uint8_t *)"Press Black Button", CENTER_MODE); 
lcd.DisplayStringAt(0, LINE(14), (uint8_t *)"To Restart", CENTER_MODE);
}


//=======================================================================================
// FUNCTION TO RECEIVE GYROSCOPE DATA AND CALCULATE THE LINEAR DISTANCE OF 3 CO-ORDINATES
//=======================================================================================
float* getGyroData(int32_t StartingAddress)          
{
    int16_t raw_gx, raw_gy, raw_gz;                                         // Declare the variables to get raw angular velocities from Gyroscope
    float gx, gy, gz;                                                       // Declared variables to store the filtered angular velocities data


    flags.wait_all(DATA_READY_FLAG);                                        // Wait for the Data Ready flag to be set up to commence the SPI transfer

    write_buf[0] = StartingAddress | 0x80 | 0x40;                           // Providing SPI Protocol for SDI [Read=1, Master=1]
                                                                            // SDI --> R/Wbar  Master/Slavebar AD5 AD4 AD3 AD2 AD1 AD0 
                                                                            //           1           1          0   0   0   0   0   0                [0x80 | 0x40]

    spi.transfer(write_buf, WRITELIMIT_SIZE1, read_buf, READLIMIT_SIZE1, spi_cb,SPI_EVENT_COMPLETE);  // SPI Transfer protocol with different byte limits for write and read buffers. WRITELIMIT_SIZE1 = 7, READLIMIT_SIZE1 = 7
    flags.wait_all(SPI_FLAG);                                                 // Setup 'SPI_flag' flag to indicate the transfer is complete


    // Processing the raw data from the Gyroscope
    raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);        // Store first 2 bytes from read_buf into raw_gx which is of 16bits (2 bytes)
    raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);        // Store further 2 bytes from read_buf into raw_gy which is of 16bits (2 bytes)
    raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);        // Store further 2 bytes from read_buf into raw_gz which is of 16bits (2 bytes)


    //Storing the Filtered Angular Velocity values for each sample into the 'angularVelocity' memory array:
    angularVelocity[i_cnt][0]=raw_gx;
    angularVelocity[i_cnt][1]=raw_gy;
    angularVelocity[i_cnt][2]=raw_gy;

    // Calculating and displaying the current filtered angular velocity values of all 3 co-ordinates as well as the Average angular velocity:
    float avg_AngVel= angularVelocity[i_cnt][0]+angularVelocity[i_cnt][1]+angularVelocity[i_cnt][2]/DIM_COUNT;
    printf("\nFiltered Angular Velocity:-> \tgx_AngVel: %f \t gy_AngVel: %f \t gz_AngVel: %f\t Avg_AngVel:%f\n",angularVelocity[i_cnt][0],angularVelocity[i_cnt][1],angularVelocity[i_cnt][2], avg_AngVel );
    i_cnt++;

    if(i_cnt==DUR_SAMPLE_COUNT)       // Checking if i_cnt == 40 for the Array Index
    {
        i_cnt=0;                      // Set i_cnt to '0' as the 'angularVelocity' memory is full, as it covered all the values for 20s duarion! [20/0.5 = 40]
    }


    //Conversion of Filtered Angular Velocity to Linear Velocity using the scaling factor of all 3 co-ordinates:
    gx = ((float)raw_gx) * ScalingFactor;       //X-Dimension
    gy = ((float)raw_gy) * ScalingFactor;       //Y-Dimension
    gz = ((float)raw_gz) * ScalingFactor;       //Z-Dimension

    // Temporary registers to store current linear velocity values of all 3 co-ordinates:
    window_gx[window_index]  = gx;              //X-Dimension
    window_gy[window_index]  = gy;              //Y-Dimension
    window_gz[window_index]  = gz;              //Z-Dimension


    // LPF LOW PASS FILTER: (This can also be used to the replacement of the Moving Average Filter, but might not be accurate enough)
    // filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
    // filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
    // filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;


    // Introducing the Moving Average Filter to Filter the linear velocity values of all 3 co-ordinates. 
    // Moving Average Filter will smoothen the graph for linear velocity values of all 3 co-ordinates when we observe it on TelePlot. 
    float avg_gx = 0.0f, avg_gy = 0.0f, avg_gz = 0.0f;   // Declare local variables to store the Average values of all 3 co-ordinates.
    for (int i = 0; i < WINDOW_SIZE; i++)                // Averaging Algorithm TO SMOOTHEN the coefficients of the gyroscope readings, reducing noise impacts
    {
        avg_gx += window_gx[i];                          // Integrating the x co-ordinate linear velocity values for each sample time
        avg_gy += window_gy[i];                          // Integrating the y co-ordinate linear velocity values for each sample time
        avg_gz += window_gz[i];                          // Integrating the z co-ordinate linear velocity values for each sample time
    }
    avg_gx /= WINDOW_SIZE;                               // Averaged x co-ordinate linear velocity value for the current sample time
    avg_gy /= WINDOW_SIZE;                               // Averaged y co-ordinate linear velocity value for the current sample time
    avg_gz /= WINDOW_SIZE;                               // Averaged z co-ordinate linear velocity value for the current sample time

    // Storing the Filtered Linear Velocity values for each sample into the 'linearVelocity' memory array:
    linearVelocity[j_cnt][0]=avg_gx;
    linearVelocity[j_cnt][1]=avg_gy;
    linearVelocity[j_cnt][2]=avg_gz;

    // Calculating and displaying the current filtered linear velocity values of all 3 co-ordinates as well as the Average linear velocity:
    float avg_LinVel= linearVelocity[j_cnt][0]+linearVelocity[j_cnt][1]+linearVelocity[j_cnt][2]/DIM_COUNT;
    printf("\nFiltered Linear Velocity:-> \tgx_LinVel: %f \t gy_LinVel: %f \t gz_LinVel: %f\t Avg_LinVel:%f\n",linearVelocity[j_cnt][0],linearVelocity[j_cnt][1],linearVelocity[j_cnt][2], avg_LinVel );
    j_cnt++;

    if(j_cnt==DUR_SAMPLE_COUNT)       // if j_cnt == 40
    {
        j_cnt=0;                      // Set j_cnt to '0' as the 'linearVelocity' memory is full, as it covered all the values for 20s duarion!  [20/0.5 = 40]
    }



    // The filtered linear velocity readings can also be outputted onto the file:
    //Commented this section, as we have used arrays to store the readings and display them.
    // if (file == NULL) {
    //     fprintf(stderr, "Error opening file for writing.\n");
       
    // }
    // fprintf(file, "%f, %f, %f", avg_gx, avg_gy, avg_gz);
    // fflush(file);
   
   
    // Individual co-ordinates distance Determination:  
    // Single co-ordinate distance = (Filtered/Averaged) Linear Velocity * Radius * Time; where Time = 0.5s
    float gx_length = (avg_gx)* (Radius) * 0.5; //Length in Dimension acorss-X
    float gy_length = (avg_gy)* (Radius) * 0.5; //Length in Dimension acorss-Y
    float gz_length = (avg_gz)* (Radius) * 0.5; //Length in Dimension acorss-Z

    //WE can also display the above readings onto the serial monitor, as follows:
    //printf("\nFiltered LENGTH:-> \tgx: %f \t gy: %f \t gz: %f\n", gx_length, gy_length, gz_length);
    
    //Thread Sleep -Optional Section: Can be omitted as well
    // thread_sleep_for(500); //When added, the displaying of the values can be seen comfortably.

    result[0]=gx_length;               // Storing the x co-ordinate distance in the 'result' register final array
    result[1]=gy_length;               // Storing the y co-ordinate distance in the 'result' register final array
    result[2]=gz_length;               // Storing the z co-ordinate distance in the 'result' register final array


    //Resetting the window index for the Moving Average Filter to zero, to repeat the averaging on new set of readings:
    window_index = (window_index + 1) % WINDOW_SIZE;

    //Consolidated array return at the state machine:
    return result;                    // Returning the 'result' register which holds individual co-ordinate distance values.
}


//Function to calculate the resultant distance using all the 3 individual co-ordinate distances by passing the 'result' register
float calculateDist3Dim(float gyroDimDegRes[])    
{
    //Distance Calculation using the 3 Dimensional Distance Formula:
    float xSqr=(gyroDimDegRes[0]-gX_ref)*(gyroDimDegRes[0]-gX_ref);   // Squaring the x co-ordinate distance.
    float ySqr=(gyroDimDegRes[1]-gY_ref)*(gyroDimDegRes[1]-gY_ref);   // Squaring the y co-ordinate distance.
    float zSqr=(gyroDimDegRes[2]-gZ_ref)*(gyroDimDegRes[2]-gZ_ref);   // Squaring the z co-ordinate distance.
    float distcalc=sqrt(xSqr+ ySqr + zSqr);                           // The resultant distance is calculated by taking squareroot of all three integrated squared co-ordinate distances.

    printf("\nCurrent Input Calc: \t%f\n", distcalc);                 // Statement to print the current resultant distance while moving.

    //Updating the Reference points to the newly moved point as the person moves:
    gX_ref=gyroDimDegRes[0];                                          // Updating x-cordinate reference point for resultant distance calculation
    gY_ref=gyroDimDegRes[1];                                          // Updating y-cordinate reference point for resultant distance calculation
    gZ_ref=gyroDimDegRes[2];                                          // Updating z-cordinate reference point for resultant distance calculation


    //Logic for incrementing the Step Count:
    if(distcalc>=0.00280)   //Post multiple trials, we observed this threshold value to efficient in determining the step count
    {
        ++step_cnt;                                                   // if the resultant distance is greater than 0.0028m, the person has moved by one step.
    }
    
    // else
    // {
    //     step_cnt=step_cnt;
    // }

   return distcalc;                                                   // Returning the resultant distance value.

   //Also the code can be modififed to return the stepcnt with the distance, but not doing it as it was not critical part of the requirement of challenge.
}

//======================================================================
//MAIN FUNCTION:
//======================================================================
int main()
 {  
    // Setting up the Background of the LCD layer to Black:
    setup_background_layer();
   
    // Setting up the foreground of the LCD layer to Green:
    setup_foreground_layer();

    // Interrupt Initialization:
    InterruptIn int2(PA_2, PullDown);           // Declaring a PullDown event centric Interrupts
    int2.rise(&data_cb);                        // Configuring the rise transition interrupt to trigger 'data_cb' callback function to implicitly set the data ready 'DATA_READY_FLAG' flag.

    // Sampling the data every 0.5seconds using the Ticker HAL Interrupts:
    t.attach(&cb, TICKER_LIMIT);                // TICKER_LIMIT = 500ms
   
    // Setting up the SPI format and frequency
    spi.format(8, 3);                           // Transferable bits = 8, SPI MODE = 3 [Clock starting position = High, Data received on rising edge of the clock]
    spi.frequency(1'000'000);                   // SPI frequency set to 1MHz


    // Writing the Control Register 1 address and configuration onto write buffer
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, WRITELIMIT_SIZE, read_buf, READLIMIT_SIZE, spi_cb,SPI_EVENT_COMPLETE);   // SPI transfer using WRITELIMIT_SIZE = 2 and READLIMIT_SIZE = 2
    flags.wait_all(SPI_FLAG);                                                                        // Setup 'SPI_flag' flag to indicate the transfer is complete

    // Writing the Control Register 4 address and configuration onto write buffer
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, WRITELIMIT_SIZE, read_buf, READLIMIT_SIZE, spi_cb,SPI_EVENT_COMPLETE);   // SPI transfer using WRITELIMIT_SIZE = 2 and READLIMIT_SIZE = 2
    flags.wait_all(SPI_FLAG);                                                                        // Setup 'SPI_flag' flag to indicate the transfer is complete

    // Writing the Control Register 1 address and configuration onto write buffer
    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, WRITELIMIT_SIZE, read_buf, READLIMIT_SIZE, spi_cb,SPI_EVENT_COMPLETE);  // SPI transfer using WRITELIMIT_SIZE = 2 and READLIMIT_SIZE = 2
    flags.wait_all(SPI_FLAG);                                                                       // Setup 'SPI_flag' flag to indicate the transfer is complete

    write_buf[1] = 0xFF;                                                                            // To mark end of writing. 0xFF = Reserved


    //Polling data ready flag:
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1))
    {
        flags.set(DATA_READY_FLAG);
    }


    //Initial welcome message on LCD:
    Initial_ScreenDisp();

    //Commencing the reset timer:
    resetTimer.start();


    //while(1){} => for infinite duration if its to be implemented.


    //While loop to execute only for 20 second duration:
    while (chrono::duration_cast<chrono::seconds>(resetTimer.elapsed_time()).count() <= RESET_TIMERLIMIT)          // RESET_TIMERLIMIT = 20
    {
        float *gyroCurrDimData=getGyroData(OUT_X_L);                                                               // Get the Gyroscope Data (individual x,y,z co-ordinate distance) onto the local variable 'gyroCurrDimData'
        printf("\nInput Gyro Data: %f\t%f\t%f", gyroCurrDimData[0], gyroCurrDimData[1], gyroCurrDimData[2] );      // Print the x, y, z co-ordinate distance onto the terminal
        //thread_sleep_for(500); //Can be added to see the results slowly at the monitor
       
        //sem.acquire();
       
       //----------------------------------FSM Implementation for the Distance Calculation:----------------------------------------------
        // State Transitioning Logic:
        switch(state_chk)
        {
            case IDLE:                                  // IDLE State if the person is at rest.
            {
                //sem.acquire();

                //These volatile global declarations have been made initalized to zero at the beginning of the code at the gyroscope section:
                totalDist=totalDist;                    // If the person is at rest, no change in the total distance computed within this 20 sec timer.
                step_cnt=step_cnt;                      // If the person is at rest, no change in the step_cnt computed within this 20 sec timer.

                //Will move into the next state only when the value read is greater than or equal to the threshold.
                //Post multiple trials, we have declared the value in the macro: GYRO_THRESHOLD
                //The threshold can be also be changed but that will impact tremendously on the accuracy of the measurements.
                //We have used a multiplication factor just to increase the magnitude scale of low intensity gyroscope readings:
                if((abs(gyroCurrDimData[0]*GRYOMULFACTOR1)>=GYRO_THRESHOLD) || (abs(gyroCurrDimData[1]*GRYOMULFACTOR1)>=GYRO_THRESHOLD) || (abs(gyroCurrDimData[2]*GRYOMULFACTOR1)>=GYRO_THRESHOLD))     // Multiplying the individual x,y,z co-ordinate distance with GYROMULFACTOR1 for proper scaling of all 3 individual co-ordinates distance
                {
                    state_chk=MOVING_DATARECORD;        // If any of the current distance of x or y or z co-ordinate is equal to or greater than the threshold distance 'GYRO_THRESHOLD' then the state transitions to 'MOVING_DATARECORD' state
                }
                else
                {
                    state_chk=IDLE;                     // Else remain in the same state with zero readings to have no impact on the distance calculations:
                    gyroCurrDimData[0]=0.0f;            // Updating the x co-ordinate distance to 0.0m
                    gyroCurrDimData[1]=0.0f;            // Updating the y co-ordinate distance to 0.0m
                    gyroCurrDimData[2]=0.0f;            // Updating the z co-ordinate distance to 0.0m
                }
                //sem.release();

            }
            break;

            case MOVING_DATARECORD:                     //In this state, the values beyond threshold are used in reocrding of total distance calculation along with stepcnt.
            {
                //sem.acquire();

                totalDist=totalDist+ (calculateDist3Dim(gyroCurrDimData)*GRYOMULFACTOR2);        // Calculate the resultant distance by passing the local variable 'gyroCurrDimData' along with stepcnt estimation.
                printf("\nTotal Distance Travelled So Far:%f\t", totalDist);                     // Print Current total distance travelled so far within 20s in the terminal
                printf("\nTotal Step Counts So Far:\t %d",step_cnt);                             // Print Current total step count so far within 20s in the terminal
                //thread_sleep_for(5000);                                                        // Optional to use.
                
                //Post determining ht
                state_chk=IDLE;                                                                  // Transition back to IDLE state

                //sem.release()
            }
            break;
        }

        //sem.release();

        CALC_ScreenDisp(totalDist, step_cnt);                                                    // Function to display current total distance travelled and current total step count within 20s duration onto the LCD screen
    }

    CALC_Final_ScreenDisp(totalDist, step_cnt);                                                  // Function to display final total distance travelled and final total step count covered for the 20s duration onto the LCD screen

    resetTimer.stop();                                                                           // Stop the reset timer to indicate end of 20s duration                               

    //state_chk=IDLE;                                                                              // Transition back to IDLE state after the 20 second duration to restart if needed.

    //The following statement is with regards to file which had the velocity values outputted:
    //fclose(file);                                                                              // Close the file if the current and final total distance and total step count are streaming onto a csv file located in the project working directory

 }