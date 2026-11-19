/* 
 *  
 *  Ver. 13.1 2026-11-19
 *  - Updtated KB_RBrain.h used, some methods renamed
 *  
 *  Ver. 13 2026-04-29 with Neural Network for Arduino Nano
 *  Features:
 *  - Finite state machine implementation;
 *  - Class NN - Neural network for Arduino;
 *  - Neural Network, can be saved to EEPROM and loaded from EEPROM on power-on;
 *  - "Context"/"Memory" neurones (kind of recurrent neural network implementation);
 *  - Class RBrain - "Interface" between robot sensors, motor drivers and Neural Network;
 *  - RBrain v4 allows to manage arbitary quantity and type of sensors, motor programm parameters, motor programm steps and context neurones;
 *  - Class Timer_R - "free run" timer to measure time robot moves between obstacles;
 *  - display_freeram() function to check Arduino RAM usage;
 *  
 *  Robot:
 *  - Two DC motors chassis;
 *  - Ultrasonic sonar mounted on servo - scans distance in three directions;
 *  - Four IR bumper sensors;
 *  
 *  
 *  Algorithm:
 *  loop(){
 *    Robot moves forward; 
 *    if it encounters an obstacle{
 *      Repeat untill it escapes the obstacle{
 *        Collect sensors readings before attempting to escape;
 *        Generate programs for motors;
 *        Predict and propose a programm for motors that will let the Robot to escape the obstacle;
 *        Try to scape the obstacle using proposed programm;
 *        Collect sensors readings after escaping attempt;
 *        Train NN with result{ 
 *          sensor readings and motor programm before escape -> propagate NN -> predicted sensors readings after escape; 
 *          predicted VS actual sensors readings after escape -> backpropagate NN;
 *        }
 *      }
 *    }
 *  }
 *  
 *  
 *  Refs:
 *  [1] J. Tani, вЂњModel-based learning for mobile robot navigation from the dynamical systems perspective,вЂќ 
 *      IEEE Trans. Syst., Man, Cybern. B, vol. 26, no. 3, pp. 421вЂ“436, Jun. 1996, doi: 10.1109/3477.499793.
 *  [2] D. JanglovГЎ, вЂњNeural Networks in Mobile Robot Motion,вЂќ International Journal of Advanced Robotic Systems, 
 *      vol. 1, no. 1, p. 2, Mar. 2004, doi: 10.5772/5615.
 *  [3] J. L. Elman, вЂњFinding Structure in Time,вЂќ Cognitive Science, vol. 14, no. 2, pp. 179вЂ“211, Mar. 1990, doi: 10.1207/s15516709cog1402_1.
 *      Accesed: April 03, 2026. [Online]. Available: 
 *      https://onlinelibrary.wiley.com/doi/10.1207/s15516709cog1402_1
 *  [4] Arduino Docs, вЂњPROGMEM,вЂќ Arduino Docs. Accessed: Apr. 20, 2026. [Online]. Available: 
 *      https://docs.arduino.cc/language-reference/en/variables/utilities/PROGMEM/#the-f-macro     
 *  [5] H. Talk, вЂњSave SRAM with the F() macro,вЂќ Tech Explorations. Accessed: Apr. 20, 2026. [Online]. 
 *      Available: https://techexplorations.com/guides/arduino/programming/f-macro/     
 *  [6] M. Girish, вЂњWhat are the traditional ways to optimize program memory usage?,вЂќ Arduino Stack Exchange. Accessed: Apr. 20, 2026. [Online]. 
 *      Available: https://arduino.stackexchange.com/questions/76619/what-are-the-traditional-ways-to-optimize-program-memory-usage     
 *  [7] Arduino, J. Bagur, and T. Chung, вЂњArduino Memory Guide,вЂќ Arduino Docs. Accessed: Apr. 20, 2026. [Online]. 
 *      Available: https://docs.arduino.cc/learn/programming/memory-guide/
 *  [8] Arduino Learning, вЂњState Machines in Arduino Projects Explained (With Examples),вЂќ Arduino Learning. Accessed: Apr. 20, 2026. 
 *      [Online]. Available: https://www.arduinolearning.com/learning/state-machines-in-arduino-projects-explained-with-examples.php
 *  [9] K.-S. (KC) Chang and D. Zhu, вЂњFinite State Machine (FSM) Concept and Implementation,вЂќ Standford University. Accessed: Apr. 20, 2026. 
 *      [Online]. Available: https://web.stanford.edu/class/cs123/
 *  [10] C. Terman, вЂњFinite State Machines,вЂќ Computation Structures. Accessed: Apr. 20, 2026. [Online]. 
 *       Available: https://computationstructures.org/lectures/fsm/fsm.html
 *  [11] Arduino, вЂњA guide to EEPROM,вЂќ Arduino Docs. Accessed: Apr. 24, 2026. [Online]. 
 *       Available: https://docs.arduino.cc/learn/programming/eeprom-guide/
 *
 *  
 */

#include <EEPROM.h>           // To save NN
#include "KB_Sonar2.h"        // !!! Set Arduino pins for ultrasonic sensor and servo here
#include "KB_Chassis_DC2.h"   // !!! Set motor shield connection to motors here
#include "KB_IR_Bumper.h"     // !!! Set Arduino IR sensor pins here
#include "Mantis_NN.h"          // !!! Set NN params here
#include "KB_RBrain.h"        // !!! Set robot behavior and how it use NN here
#include "KB_Timer_R.h"       // timer with pause


// --------------------
// Define working modes
// --------------------
#define MIN_REPORT_MODE       // write to UART only minimal info
//#define REPORT_MODE           // write more info to UART for debugging purposes
#define USE_SONAR             // use sonar
#define USE_IRBUMPER          // use IR bumper sensors
#define USE_RESTING           // allows robot to rest periodically (in order to cool down power supply module)
#define USE_MOTORS            // allows robot to use motors to move
#define USE_TIMER             // use timer to measure time robot move between obstacles
//#define RND_AWOID_DIR       // Allows the robot to turn in random directions while avoiding obstacles (by default, the robot only turns to the right)
//#define RND_AFTER_REST      // Allows the robot to turn in random directions on waking-up after resting
#define SAVE_NN_TO_EEPROM     // save NN and related data to EEPROM
#define LOAD_NN_FROM_EEPROM   // load NN and related data from EEPROM at power up



// ----------------
// General settings
// ----------------
const int rbt_runtime_min = 5000;          // mS, robot min working time between resting 
const int rbt_runtime_max = 15000;         // mS, robot max working time between resting 
const int rbt_resttime = 4000;             // mS, robot resting time (in order to cool-down motors and power supply)
const int rbt_idletime = 1000;             // mS, robot startup idle time

// --------------
// Neural network
// -------------- 
const byte rbt_NNL_cnt = 3;    // number of NN layers 
const byte rbt_L0N_cnt = 10;   // number of nodes in input layer
const byte rbt_L1N_cnt = 7;    // number of layers in NN
const byte rbt_L2N_cnt = 5;    // number of nodes in output layer

// -------
// Sensors
// -------
const byte rbt_s_cnt = 5;       // number of sensors inputs - 3 for sonar (F, L and R), 1 for array IR bumper, 1 for "free run" timer
const byte s_row_irb = 0;       // idx of IR bumper sensor signal in table S(!) (in table S, not in NN's in/out data arrays)
const byte s_row_usf = 1;       // idx of US forward distance readings in table S(!) (in table S, not in NN's in/out data arrays)
const byte s_row_usl = 2;       // idx of US left distance readings in table S(!) (in table S, not in NN's in/out data arrays)
const byte s_row_usr = 3;       // idx of US right distance readings in table S(!) (in table S, not in NN's in/out data arrays)
const byte s_row_tmr = 4;       // idx of "free run" timer readings in table S(!) (in table S, not in NN's in/out data arrays)
const double s_usf_thr = 15.0;    // cm, threshold distance for obstacle in forward direction
const double s_uslr_thr = 15.0;    // cm, threshold distance for obstacle in left/right direction

// Table S:
// holds data related to sensors
// it can be arbitary number of rows
// columns description:
// [][0] holds index of a sensor in  NN's input/output data arrays;
// [][1] holds min value detectable by sensor;
// [][2] holds max value detectable by sensor;
// [][3] holds lower bound of sensor readings acceptable for robot;
// [][4] holds upper bound of sensor readings acceptable for robot;
// [][5] holds "weight" or "importance" of a sensor - its value will be added to environment rating if the readings of this sensor is in range;
// [][6] holds actual sensors readings
// [][7] holds temporal/virtual sensors raadings
// +------------------------------------------------------------------------------------------------+
// | X,Y[idx] | min val | max val | lower bound | upper bound | weight | actual val | temp/virt val | 
// +------------------------------------------------------------------------------------------------+
double rbt_Table_S[rbt_s_cnt][BRN_S_COL_CNT] = 
{{0,           0.0,                     15.0,         0.0,                     0.2,  50.0,  0.0, 0.0},     // row#0 IR bumpers array - encoded binary value (one bit for each of four sensors, DEC 0...15)
 {1,  SNR_DIST_MIN,             SNR_DIST_MAX,   s_usf_thr,                 10000.0,  20.0,  0.0, 0.0},     // row#1 ultrasonic sensor - readings in forward direction, cm
 {2,  SNR_DIST_MIN,             SNR_DIST_MAX,  s_uslr_thr,                 10000.0,  10.0,  0.0, 0.0},     // row#2 ultrasonic sensor - readings in left direction, cm
 {3,  SNR_DIST_MIN,             SNR_DIST_MAX,  s_uslr_thr,                 10000.0,  10.0,  0.0, 0.0},     // row#3 ultrasonic sensor - readings in right direction, cm 
 {4,           0.0,  (double)rbt_runtime_max,         0.0, (double)rbt_runtime_max,  10.0,  0.0, 0.0}};    // row#4 "free-run" timer, mS


// ---------------------------------
// Motor programm generator/selector
// ---------------------------------
const byte rbt_mpparam_cnt = 3;                     // number of motor programm parameters (left and right motors speed, motors working time)
const byte rbt_mpsteps_cnt = 1;                     // number of motor programm steps 
const int rbt_mpgenatt_cnt = 300;                   // number of attempts motor programm generator generates a motor programm
const double rbt_env_thr1 = 60;                     // environment rating threshold to accept generated motor programm as candidate (see sensor weights in Table S)
const double rbt_env_thr2 = 80;                     // environment rating threshold to accept motor programm (see sensor weights in Table S)
const int rbt_mtrspd_min = -DC2_MAX_SPD_MV;         // min motor speed allowed for motor programm (0...255)
const int rbt_mtrspd_max = DC2_MAX_SPD_MV;          // max motor speed allowed for motor programm (0...255)
const int rbt_mtrwt_min = 100;                      // mS, min motor working time to be proposed by motor programm generator
const int rbt_mtrwt_max = 3000;                     // mS, max motor working time to be proposed by motor programm generator
const byte m_row_lms = 0;                           // idx of left motor speed parameter in tables M, MPGEN, MPSEl(!! not in NN's in/out data arrays)
const byte m_row_rms = 1;                           // idx of right motor speed parameter in tables M, MPGEN, MPSEl(!! not in NN's in/out data arrays)
const byte m_row_mwt = 2;                           // idx of motor working time parameter in tables M, MPGEN, MPSEl(!! not in NN's in/out data arrays)

// Table M - motor programm settings
// it can be arbitary number of rows
// columns description:
// [][0] holds index of a motor programm paramenter in X NN's input data array
// [][1] holds min value of a parameter to be generated
// [][2] holds max value of a parameter to be generated
// +----------------------------+
// | X[idx] | min val | max val |
// +----------------------------+
int rbt_Table_M[rbt_mpparam_cnt][BRN_M_COL_CNT] = 
{{5, rbt_mtrspd_min, rbt_mtrspd_max},           // row#0 Left motor programm settings
 {6, rbt_mtrspd_min, rbt_mtrspd_max},           // row#1 Right motor programm settings
 {7,  rbt_mtrwt_min,  rbt_mtrwt_max}};          // row#2 otors working time programm settings


// ----------------
// Context neurones
// ----------------
const byte rbt_c_cnt = 2;   // number of context neurones    
const double c_init_val = 0.5; // init value for context neurone

// Table C columns description:
// it can be arbitary number of rows
// [][0] holds idx of an element in X data array and in NN's input layer to feedback context neurone   
// [][1] holds idx of NN's layer with a context neurone
// [][2] holds idx of context neuron node inside a layer
// +------------------------+
// | X[idx] | NN L# | NN N# |
// +------------------------+                
byte rbt_Table_C[rbt_c_cnt][BRN_C_COL_CNT] = 
{{8, 1, 0},           // row#0 context neuron #0 indexes
 {9, 1, 1}};          // row#1 context neuron #1 indexes


// ---------------------------------------------
// Variables and timers to control state machine
// ---------------------------------------------
double env_rating = 0;                  // stores environment rating
byte rbt_motprogstep_idx = 0;           // counts motor programm steps accomplished by the robot
bool rbt_motprog_accomplished = false;  // flag to be set after robot accomplished all motor programm steps

unsigned long tmr_wt = 0;     // Timer to control robot working time and resting 
Timer_R tmr_freerun;          // Timer with pause to measure "free run" time robot moves between obstacles


// --------
// Counters
// --------
unsigned int obstacle_alerts_cnt = 0;    // counts encountered obstacles
unsigned int learnings_total_cnt = 0;    // counts all learning attempts through all encountered obstacle


// --------------
// EEPROM related
// --------------
const byte eepromControl1 = 1;
const byte eepromControl2 = 2;
const byte eepromControl3 = 3;
const byte eepromControl4 = 4;
bool isNNLoadedFromEEPROM = false;  // flag wether NN has been loaded from EEPROM


// ------------
// Sonar object
// ------------
SNR Robo_Sonar(A0, A1, 9);


// ---------------------------
// NN Object and RBrain object 
// ---------------------------
RBrain Robo_Brain;
NN Robo_NN;


// -------------
// State machine
// -------------
enum State {
  EVADE_OBST,          // Try to evade an obstacle
  FULL_SCAN,           // Full environment scan
  INIT_MOVE_FW,        // Moving forward initialization
  MOVE_FW,             // Moving forward 
  PREP_TO_START,       // Prepare to start
  PROPOSE_MOTPROG,     // Propose motor programm
  QUICK_SCAN,          // Bumpers check and sonar scan in forward direction
  RESTING,             // Resting
  SAVE_INIT_COND,      // Encoding initial conditions into NN's input array X
  SAVE_RESULT_COND,    // Encoding outcome conditions into NN's target/outcome array Y
  SAVE_NN,             // Saving NN weights and related data to EEPROM
  STOP_OBST,           // Stop robot because of obstacle
  TRAIN                // Train NN 
};
State robo_state = PREP_TO_START;


// ---------------------
// FUNCTION DECLARATIONS
// ---------------------

// Behavior
void rbt_takeARestIfNeed();
void rbt_fullScan();

// Aux
void rbt_reportSensorsReadings();
void rbt_reportDecodedDataArr(byte arr_idx, bool endline);
void rbt_reportContX(bool endline);
void rbt_reportContH(bool endline);

// EEPROM Related
int saveToEEPROM_double2D(double** Arr, int r_cnt, int c_cnt, int add);
int readFromEEPROM_double2D(double** Arr, int r_cnt, int c_cnt, int add);
int saveNNToEEPROM(unsigned long cnt_alerts, unsigned long cnt_learn);
int loadNNFromEEPROM(unsigned long& cnt_alerts, unsigned long& cnt_learn);

// RAM control
void display_freeram();
int freeRam();


// ------------------------------------------------------------------------------------------------------------
// SETUP() AND LOOP()
// ------------------------------------------------------------------------------------------------------------
void setup() {

  // For reporting and troubleshooting
  #if defined(MIN_REPORT_MODE) || defined(REPORT_MODE)
    Serial.begin(9600); 
    delay(100);  
  #endif
  
  #ifdef MIN_REPORT_MODE
    Serial.println();
    Serial.print(F("RAM @Setup() start"));
    display_freeram();
  #endif

  // -----------
  // Seup NN
  // -----------

  // Set NN layers number and sizes
  Robo_NN.setNumOfLayers(rbt_NNL_cnt);
  Robo_NN.setNumOfNodes(0, rbt_L0N_cnt);
  Robo_NN.setNumOfNodes(1, rbt_L1N_cnt);
  Robo_NN.setNumOfNodes(2, rbt_L2N_cnt);

  #ifdef MIN_REPORT_MODE
    Serial.println();
    Serial.print(F("Estimated RAM usage for NN Layer Objects (In="));
    Serial.print(rbt_L0N_cnt);
    Serial.print(F(", H1="));
    Serial.print(rbt_L1N_cnt);
    Serial.print(F(", O="));
    Serial.print(rbt_L2N_cnt);
    Serial.print(F("): "));
    Serial.println(Robo_NN.estimateTotalLayersRAM());
  #endif

  // initialise empty NN according to layers sizes set
  Robo_NN.initEmptyNN();   

  #ifdef MIN_REPORT_MODE
    Serial.println();
    Serial.println(F("Empty NN created"));   
  #endif

  #ifdef MIN_REPORT_MODE
    Serial.println();
    Serial.print(F("RAM after Empty NN created"));
    display_freeram();
  #endif



  // -------------
  // Setup "Brain"
  // -------------

  // set parameters and initialise
  Robo_Brain.setCntXY(rbt_L0N_cnt, rbt_L2N_cnt);                // set sizes of data arrays X(Same as NN In layer), Y(Same as NN Out layer)   
  Robo_Brain.setMotProgGenerator(rbt_mpparam_cnt, rbt_mpsteps_cnt);                    // sent quantity of motor programm parameters and quantity of motor programm steps
  Robo_Brain.setMotProgSelector(rbt_env_thr1, rbt_env_thr2, rbt_mpgenatt_cnt);    // set thresholds to evaluate motor programm and number of attempts to generate a motor programm
  Robo_Brain.init(false);                                          // Init and allocate memory for Robo_Brain
  Robo_Brain.addNN(&Robo_NN);                                     // Attach NN to Robo_Brain
  Robo_Brain.addTableS(rbt_Table_S, rbt_s_cnt);
  Robo_Brain.addTableM(rbt_Table_M, rbt_mpparam_cnt);
  Robo_Brain.addTableC(rbt_Table_C, rbt_c_cnt);

  #ifdef MIN_REPORT_MODE
    Robo_Brain.dispBRN();
    Serial.println();
    Serial.print(F("RAM After initialising of BRN"));
    display_freeram();
  #endif
  
  #ifdef MIN_REPORT_MODE
    Serial.println();
    Serial.println(F("BRN set"));  
    Robo_Brain.dispBRN(); 
  #endif

  #ifdef MIN_REPORT_MODE
    Serial.println();
    Serial.print(F("RAM after BRN set"));
    display_freeram();
  #endif


  // ------------------------------------
  // Load data from EEPROM or Init random
  // ------------------------------------
  
  // Load NN weighta and context neurones data from EEPROM
  #ifdef LOAD_NN_FROM_EEPROM
    loadNNFromEEPROM(obstacle_alerts_cnt, learnings_total_cnt);
  #endif

  // If NN is not loaded from EEPROM
  // - randomize weights
  // - initialise context neurones with defined value
  if(isNNLoadedFromEEPROM == false)
  {
    Robo_NN.randomizeWeights(0); // init NN weights with random values

    for(int i = 0; i < Robo_Brain.getCntC(); i++)
    {
      Robo_Brain.setValC(c_init_val, i);
    }
    
      #ifdef MIN_REPORT_MODE
        Serial.println();
        Serial.println(F("New NN init OK")); 
      #endif 
    }
    else
    {
      #ifdef MIN_REPORT_MODE
        Serial.println();
        Serial.println(F("EEPROM->NN OK")); 
      #endif
    }

    #ifdef MIN_REPORT_MODE
      Serial.println();
      Serial.print(F("RAM after NN init"));
      display_freeram();
    #endif

  // -------------
  // Setup Sonar
  // -------------
  #ifdef USE_SONAR  
    Robo_Sonar.setupSnr();   
    Robo_Sonar.setThreshold(s_usf_thr); 

    #ifdef MIN_REPORT_MODE
      Serial.println();
      Serial.println(F("SNR set"));   
    #endif

    #ifdef MIN_REPORT_MODE
      Serial.println();
      Serial.print(F("RAM after SNR set"));
      display_freeram();
    #endif
  #endif


  // -------------
  // IR Bumpers
  // -------------
  #ifdef USE_IRBUMPER
    IRB_setup();

    #ifdef MIN_REPORT_MODE
      Serial.println();
      Serial.println(F("IR set"));   
    #endif

    #ifdef MIN_REPORT_MODE
      Serial.println();
      Serial.print(F("RAM after IR set"));
      display_freeram();
    #endif
  #endif

  #ifdef MIN_REPORT_MODE
    Serial.println();    
    Serial.println(F("NN and histoty before start:")); 
    Serial.print(F("Obs#/Tot: ")); 
    Serial.print(obstacle_alerts_cnt); 
    Serial.print(F("/")); 
    Serial.println(learnings_total_cnt);
    Robo_NN.disp_NN();
  #endif    

  // Report memory usage
  #ifdef MIN_REPORT_MODE
    Serial.println();
    Serial.print(F("RAM @setup() end"));
    display_freeram();
  #endif


  // -----------------
  // Starting sequence
  // -----------------

  #ifdef MIN_REPORT_MODE
    Serial.println();
    Serial.println(F("Go!"));
    Serial.println();
  #endif
   
  // Delay
  delay(rbt_idletime);

  // Return sonar in forward position
  #ifdef USE_SONAR
    Robo_Sonar.forwardPos();
    delay(100);
    Robo_Sonar.forwardPos();
  #endif

  // Wait after sonar is in forward position
  delay(2*rbt_idletime);

}


void loop() {

  // -------------------------
  // State machine realisation
  // -------------------------
  
  switch(robo_state)
  {
    
   // ----------------------------
    case EVADE_OBST:
    {
      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: EVADE_OBST"));
      #endif

      Robo_Brain.encMotProgToX(rbt_motprogstep_idx);

      // Restart "move timer" and move the robot according to
      // proposed programm
      tmr_freerun.clearTimer();
      tmr_freerun.startTimer();

      #ifdef REPORT_MODE
        Serial.print(F("MPG step = "));
        Serial.println(rbt_motprogstep_idx);
      #endif
          
      #ifdef USE_MOTORS
        #ifdef REPORT_MODE
          Serial.println(F("Move"));
        #endif

        DC2_moveA(Robo_Brain.getMotProgParam(m_row_lms, rbt_motprogstep_idx), Robo_Brain.getMotProgParam(m_row_rms, rbt_motprogstep_idx), DC2_MIN_SPD_MV, DC2_MAX_SPD_MV, DC2_SPD_INCR_STEP_MV, DC2_SPD_INCR_TIME_MV);
        delay(Robo_Brain.getMotProgParam(m_row_mwt, rbt_motprogstep_idx));
        DC2_Stop();
      #endif

      // Update motor programm step index and motor programm accomplished flag
      rbt_motprogstep_idx++;

      if(rbt_motprogstep_idx >= rbt_mpsteps_cnt)
      {
        rbt_motprog_accomplished = true;
        rbt_motprogstep_idx = 0;
        #ifdef REPORT_MODE
          Serial.println(F("MPG accomplished!"));        
        #endif
      }
      else
      {
        rbt_motprog_accomplished = false; 
        #ifdef REPORT_MODE
          Serial.println(F("MPG not accomplished yet"));        
        #endif
      }
      
      tmr_freerun.stopTimer();

      // Next state
      robo_state = FULL_SCAN;
      
      #ifdef REPORT_MODE
        Serial.println(F("Next state: FULL_SCAN"));
        Serial.println();
      #endif
      
      break;
    }
    
    // ----------------------------
    case FULL_SCAN:
    {  
      // Scanning of environment
      // and calculation of "environmet rating"

      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: FULL_SCAN"));
      #endif

      // Scan using all sensors (it will update table S)
      rbt_fullScan(); 

      #ifdef REPORT_MODE                
        rbt_reportSensorsReadings();
      #endif

      // Clear timer
      tmr_freerun.clearTimer();

      // Calc environment rating
      env_rating = Robo_Brain.evalActRawSensData();
      
      #ifdef REPORT_MODE
        Serial.print(F("Environment rating = "));
        Serial.println(env_rating);
      #endif

      // Next state
      // Check "free run" timer
      if(Robo_Brain.getRecS(s_row_tmr) <= 0)
      {
        // if robot didn't move
        // 
        robo_state = SAVE_INIT_COND;
              
        #ifdef REPORT_MODE
          Serial.println(F("Next state: SAVE_INIT_COND"));
          Serial.println();
        #endif
      }
      else
      {
        // if robot already moved
        robo_state = SAVE_RESULT_COND;
        
        #ifdef REPORT_MODE
          Serial.println(F("Next state: SAVE_RESULT_COND"));
          Serial.println();
        #endif
      }
      break;
    }


    // ----------------------------
    case INIT_MOVE_FW:
    {
      // Moving forward initialization
              
      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: INIT_MOVE_FW"));
      #endif

      // Save and encode "move forward" programm to input array X
      Robo_Brain.setMotProgParam(rbt_mtrspd_max, m_row_lms, 0);
      Robo_Brain.setMotProgParam(rbt_mtrspd_max, m_row_rms, 0);
      Robo_Brain.setMotProgParam(rbt_mtrwt_max, m_row_mwt, 0);
      Robo_Brain.encMotProgToX(0); 

      // Clear motor programm accomplished flag
      rbt_motprog_accomplished = false;

      // Report
      #ifdef REPORT_MODE
        rbt_reportMPG(true);
      #endif

      // Start/resume "free run" timer
      tmr_freerun.startTimer();

      // Reset worktime timer
      tmr_wt = millis();

      // Send programm to motor driver
      #ifdef USE_MOTORS
        DC2_moveA(Robo_Brain.getMotProgParam(m_row_lms, 0), Robo_Brain.getMotProgParam(m_row_rms, 0), DC2_MIN_SPD_MV, DC2_MAX_SPD_MV, DC2_SPD_INCR_STEP_MV, DC2_SPD_INCR_TIME_MV);
      #endif

      // Next state
      robo_state = MOVE_FW;
      
      #ifdef REPORT_MODE
        Serial.println(F("Next state: MOVE_FW"));
        Serial.println();
      #endif
      
      break;
    }


    // ----------------------------
    case MOVE_FW:
    {
      // check working time timer while moving forward

      // Next state
      if(millis() - tmr_wt < rbt_runtime_max)
      {
        // working time is OK
        robo_state = QUICK_SCAN;
      }
      else
      {
        // working time too long
        robo_state = RESTING;
      }
      
      break;
    }


    // ----------------------------
    case PREP_TO_START:
    {
      // Prepare robot to Start

      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: PREP_TO_START"));
      #endif

      
      // Reset timers
      tmr_wt = millis();
      tmr_freerun.stopTimer();
      tmr_freerun.clearTimer();

      // Clear sensors readings
      //Robo_Brain.clearSensorsReadings();
      Robo_Brain.encSensRdsToX();

      // Set motprog_accomplished flag
      rbt_motprog_accomplished = true;


      // Save "stop" programm and encode it to input array X
      // (robot is stopped before start)
      Robo_Brain.setMotProgParam(0, m_row_lms, 0);
      Robo_Brain.setMotProgParam(0, m_row_rms, 0);
      Robo_Brain.setMotProgParam(0, m_row_mwt, 0);
      Robo_Brain.encMotProgToX(0); 

      // set motor programm status
      rbt_motprog_accomplished = true;

      // Copy values of context neurones to input array X
      Robo_Brain.copyContNodesToX();

      // Next state
      robo_state = FULL_SCAN;

      // Report
      #ifdef REPORT_MODE
        rbt_reportDecodedDataArr(BRN_X_ID, false);
        rbt_reportContX(true);
        rbt_reportMPG(true);
      #endif
      
      #ifdef REPORT_MODE
        Serial.println(F("Next state: FULL_SCAN"));
        Serial.println();
      #endif
      
      break;
    }
  

    // ----------------------------
    case PROPOSE_MOTPROG:
    {
      // Propose motor programm
      
      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: PROPOSE_MOTPROG"));
      #endif

      #ifdef REPORT_MODE
        Serial.println(F("Before proposing motor programm"));
        rbt_reportMPG(true);
        Robo_Brain.dispMPGEN();
        Robo_Brain.dispMPSEL();
      #endif

      Robo_Brain.proposeMotProg();
      rbt_motprogstep_idx = 0;         
      rbt_motprog_accomplished = false; 

      #ifdef REPORT_MODE
        Serial.println(F("After proposing motor programm"));
        rbt_reportMPG(true);
        Robo_Brain.dispMPGEN();
        Robo_Brain.dispMPSEL();
      #endif

      // Next state
      robo_state = EVADE_OBST;  

      #ifdef REPORT_MODE
        Serial.println(F("Next state: EVADE_OBST"));
        Serial.println();
      #endif
      
      break;
    }


    // ----------------------------
    case QUICK_SCAN:
    {
      // Quick bumpers check and scan in forward direction

      // Next state (may be changed by below checks)
      robo_state = MOVE_FW;

      // Check IR sensors
      #ifdef USE_IRBUMPER 
        if(IRB_checkSensors())
        {

          // EMERGENVY STOP
          DC2_Stop();
          
          // environment rating to zero
          env_rating = 0;
         
          #ifdef REPORT_MODE
            Serial.print(F("STATE: QUICK_SCAN: IR!b:"));
            Serial.println(IRB_getBits(), BIN);
            Serial.println(F("Next state: STOP_OBST"));
            Serial.println();
          #endif

          // Next state
          robo_state = STOP_OBST;    

          break;
        }
      #endif

      // Quick check Sonar in forward direction
      #ifdef USE_SONAR
        if(Robo_Sonar.quickCheckF())
        {
          // EMERGENVY STOP
          DC2_Stop();
          
          // environment rating to zero
          env_rating = 0;
      
          #ifdef REPORT_MODE
            Serial.println(F("QUICK_SCAN: SNR!F!"));
            Serial.println(F("Next state: STOP_OBST"));
            Serial.println();
          #endif 

          // Next state
          robo_state = STOP_OBST;        

          break;
        }
      #endif
    
      break;
    }


    // ----------------------------
    case RESTING:
    {
      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: RESTING"));
      #endif

      #ifdef USE_RESTING
        tmr_freerun.pauseTimer();
        rbt_takeARestIfNeed();
      #endif

      // Next state
      robo_state = INIT_MOVE_FW;
      //robo_state = FULL_SCAN;
  
      #ifdef REPORT_MODE
        Serial.println(F("Next state: INIT_MOVE_FW"));
        Serial.println();
      #endif
      
      break;
    }


    // ----------------------------
    case SAVE_INIT_COND:
    {
      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: SAVE_INIT_COND"));
      #endif

      // convert sensors readings to NN input data
      Robo_Brain.encSensRdsToX();

      // Report
      #ifdef REPORT_MODE
        rbt_reportDecodedDataArr(BRN_X_ID, false);
        rbt_reportContX(true);
        rbt_reportMPG(true);
      #endif


      // Next state
      // env_rating was updating during scan
      if(env_rating >= rbt_env_thr1)
      {
        // If environment is OK
        robo_state = INIT_MOVE_FW;

        #ifdef REPORT_MODE
          Serial.print(F("RAM before exit current state"));
          display_freeram();
        #endif
        
        #ifdef REPORT_MODE
          Serial.println(F("Next state: INIT_MOVE_FW"));
          Serial.println();
        #endif
      }
      else
      {
        // If environment is bad
        // AND
        // if all current motor programm steps are accomplished
        if(rbt_motprog_accomplished)
        {
          robo_state = PROPOSE_MOTPROG;

          #ifdef REPORT_MODE
            Serial.println(F("Next state: PROPOSE_MOTPROG"));
            Serial.println();
          #endif
        }
        else
        {
          robo_state = EVADE_OBST;
      
          #ifdef REPORT_MODE
            Serial.println(F("Next state: EVADE_OBST"));
            Serial.println();
          #endif
        }
      }
      
      break;
    }


    // ----------------------------
    case SAVE_RESULT_COND:
    {
      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: SAVE_RESULT_COND"));
      #endif

      // Convert sensors readings to NN's target data
      Robo_Brain.encSensRdsToY();
      
      // Report
      #ifdef REPORT_MODE
        rbt_reportDecodedDataArr(BRN_Y_ID, true);
      #endif

      // Next state
      robo_state = TRAIN;

      #ifdef REPORT_MODE
        Serial.println(F("Next state: TRAIN"));
        Serial.println();
      #endif
      
      break;
    }


    // ----------------------------
    case SAVE_NN:
    {
      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: SAVE_NN"));
      #endif

      // Save NN to EEPROM
      #ifdef SAVE_NN_TO_EEPROM
        saveNNToEEPROM(obstacle_alerts_cnt, learnings_total_cnt);
        
        #ifdef REPORT_MODE
          Serial.println(F("NN->EEPROM OK"));
        #endif  
      #endif

      // Next state
      robo_state = SAVE_INIT_COND;

      #ifdef REPORT_MODE
        Serial.println(F("Next state: SAVE_INIT_COND"));
        Serial.println();
      #endif
      
      break;
    }


    // ----------------------------
    case STOP_OBST:  
    {
      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: STOP_OBST"));
      #endif

      DC2_Stop();

      // Stop timer
      tmr_freerun.stopTimer();

      // Set motprog_accomplished flag and reset counter
      rbt_motprog_accomplished = true;
      rbt_motprogstep_idx = 0;

      // ncrement obstacles counter
      obstacle_alerts_cnt++; 
      
      #ifdef REPORT_MODE
        Serial.print(F("Obstacle #"));
        Serial.println(obstacle_alerts_cnt);
      #endif

      // Next state
      robo_state = FULL_SCAN;
      
      #ifdef REPORT_MODE
        Serial.println(F("Next state: FULL_SCAN"));
        Serial.println();
      #endif

      break;
    }


    // ----------------------------
    case TRAIN: 
    {
      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: TRAIN"));
      #endif

      Robo_Brain.learn();
      Robo_Brain.copyContNodesToX();
      learnings_total_cnt++;

      #ifdef MIN_REPORT_MODE
        Serial.println(F("-----"));    
        Serial.println(F("NN and histoty after training:")); 
        Serial.print(F("Obs#/Tot: ")); 
        Serial.print(obstacle_alerts_cnt); 
        Serial.print(F("/")); 
        Serial.println(learnings_total_cnt);
        Robo_NN.disp_NN();
        Serial.println(F("-----")); 
      #endif  

      // Next state
      robo_state = SAVE_NN;

      #ifdef REPORT_MODE
        Serial.println(F("Next state: SAVE_NN"));
        Serial.println();
      #endif
      
      break;
    }


    // ----------------------------
    default:
    {
      #ifdef REPORT_MODE
        Serial.println();
        Serial.println(F("STATE: N/A"));
      #endif

      // Next state - restart
      robo_state = PREP_TO_START;

      #ifdef REPORT_MODE
        Serial.println(F("Next state: PREP_TO_START"));
        Serial.println();
      #endif
      
      break;
    }

  } // end switch

} // End loop()


// ------------------------------------------------------------------------------------------------------------
// FUNCTION DEFINITIONS
// ------------------------------------------------------------------------------------------------------------

void rbt_reportDecodedDataArr(byte arr_idx, bool endline)
{
  #ifdef REPORT_MODE
    Serial.print(F("BRN ["));

    switch(arr_idx)
    {
      case BRN_X_ID:
        Serial.print(F("X]: "));
      break;

      case BRN_Y_ID:
        Serial.print(F("Y]: "));
      break;

      case BRN_O_ID:
        Serial.print(F("O]: "));
      break;

      case BRN_V_ID:
        Serial.print(F("V]: "));
      break;
    }
    
    Serial.print(F("IR b:"));
    Serial.print((int)Robo_Brain.decodeSTypeVal(arr_idx, 0), BIN);   // IR bumper is #0 sensor in table S
    Serial.print(F("; SNR F="));
    Serial.print(Robo_Brain.decodeSTypeVal(arr_idx, 1));
    Serial.print(F("; L="));
    Serial.print(Robo_Brain.decodeSTypeVal(arr_idx, 2));
    Serial.print(F("; R="));
    Serial.print(Robo_Brain.decodeSTypeVal(arr_idx, 3));
    Serial.print(F("; fwt="));
    Serial.print((unsigned long)Robo_Brain.decodeSTypeVal(arr_idx, 4));
    Serial.print(F("; score="));
    Serial.print(Robo_Brain.evalEncodedSensorsData(arr_idx)); 
    if(endline)
    {
      Serial.println();
    }
    else
    {
      Serial.print(F("; "));
    }
  #endif    
}



// ------------------------------------------------------------------------------------------------------------
void rbt_reportMPG(bool endline)
{
  #ifdef REPORT_MODE
    Serial.println(F("BRN MPSEL:"));
    for(byte mpp_idx = 0; mpp_idx < rbt_mpparam_cnt; mpp_idx++)
    {
      for(byte mps_idx = 0; mps_idx < rbt_mpsteps_cnt; mps_idx++)
      {
        Serial.print(Robo_Brain.getMotProgParam(mpp_idx, mps_idx));
        Serial.print(F("; "));
      }
      Serial.println();
    }
  #endif 
}


// ------------------------------------------------------------------------------------------------------------
void rbt_reportContX(bool endline)
{
  #ifdef REPORT_MODE
    Serial.print(F("BRN [X]"));
    for(byte i = 0; i < Robo_Brain.getCntC(); i++)
    {
      Serial.print(F(" C"));
      Serial.print(i);
      Serial.print(F("="));
      Serial.print(Robo_Brain.getelX(Robo_Brain.getDestIdxC(i)));
    }
    
    if(endline)
    {
      Serial.println();
    }
    else
    {
      Serial.print(F("; "));
    }
  #endif  
}

// ------------------------------------------------------------------------------------------------------------
void reportContH(bool endline)
{
  #ifdef REPORT_MODE
    Serial.print(F("BRN [H]"));
    for(byte i = 0; i < Robo_Brain.getCntC(); i++)
    {
      Serial.print(F(" C"));
      Serial.print(i);
      Serial.print(F("="));
      Serial.print(Robo_Brain.getValC(i));
    }
    
    if(endline)
    {
      Serial.println();
    }
    else
    {
      Serial.print(F("; "));
    }
  #endif  
}


// ------------------------------------------------------------------------------------------------------------
void rbt_reportSensorsReadings()
{
  
  #ifdef REPORT_MODE
    Serial.print(F("Sensors readings from table S[][]: "));
    Serial.print(F("IR b:"));
    Serial.print((byte)Robo_Brain.getRecS(s_row_irb), BIN);
    Serial.print(F("; SNR F="));
    Serial.print(Robo_Brain.getRecS(s_row_usf));
    Serial.print(F("; L="));
    Serial.print(Robo_Brain.getRecS(s_row_usl));
    Serial.print(F("; R="));
    Serial.print(Robo_Brain.getRecS(s_row_usr));
    Serial.print(F("; fwt="));
    Serial.println((unsigned long)Robo_Brain.getRecS(s_row_tmr));
  #endif
  
}



// ------------------------------------------------------------------------------------------------------------
void rbt_fullScan()
{

  #ifdef USE_IRBUMPER
    Robo_Brain.setRecS((double)IRB_checkSensors(), s_row_irb);  
  #else
    Robo_Brain.setRecS(0.0, s_row_irb);  // fake readings
  #endif

  #ifdef USE_SONAR
    Robo_Sonar.fullScan();
    Robo_Brain.setRecS(Robo_Sonar.getDistF(), s_row_usf);   // Sonar forward is a #1 sensor in rbt_S_table
    Robo_Brain.setRecS(Robo_Sonar.getDistL(), s_row_usl);   // Sonar left is a #2 sensor in rbt_S_table
    Robo_Brain.setRecS(Robo_Sonar.getDistR(), s_row_usr);   // Sonar right is a #3 sensor in rbt_S_table
  #else
    Robo_Brain.setRecS(SNR_DIST_MAX, s_row_usf);   // fake readings
    Robo_Brain.setRecS(SNR_DIST_MAX, s_row_usr);   // fake readings
    Robo_Brain.setRecS(SNR_DIST_MAX, s_row_usl);   // fake readings
  #endif

  #ifdef USE_TIMER
    Robo_Brain.setRecS((double)tmr_freerun.getWorkTime(false), s_row_tmr);   // "free timer" readings is a #4 sensor in rbt_S_table
  #else
    Robo_Brain.setRecS((double)rbt_runtime_max, s_row_tmr);   // fake readings
  #endif
}

// ------------------------------------------------------------------------------------------------------------
void rbt_takeARestIfNeed()
{
   /*
   * This function is to take a rest to cool down
   * 
   * Arguments
   *  - None
   * 
   * Depends/Updates:
   *  - "KB_Chassis_DC2.h"
   *  
   *  - directive #define REPORT_MODE
   *  - directive #define RND_AFTER_REST
   *  - directive #define USE_MOTORS
   *  
   *  - const int dc_wt_min
   *  - const int dc_wt_max
   *  - const int dc_resting_time
   *  
   * 
   */

    #ifdef REPORT_MODE
        Serial.println(F("Rest"));
    #endif

    // Stop motors for defined resting time
    #ifdef USE_MOTORS
      DC2_Stop();
      delay(rbt_resttime);
    #endif

    #ifdef REPORT_MODE
      Serial.println(F("Waikup"));
    #endif
    
    // decide turn direction (In a case of ambiguity) 
    #ifdef RND_AFTER_REST
      byte turnDir = random(0,5);
    #else
      byte turnDir = 4;
    #endif

    // move according to direction decided on waking up
    switch(turnDir)
    {
      case 0:
        #ifdef REPORT_MODE
          Serial.println(F("Trn L"));
        #endif

        #ifdef USE_MOTORS
          DC2_moveA(-rbt_mtrspd_max, rbt_mtrspd_max, DC2_MIN_SPD_MV, DC2_MAX_SPD_MV, DC2_SPD_INCR_STEP_TRN, DC2_SPD_INCR_TIME_TRN);
          delay(3*rbt_runtime_min);
        #endif
      break;

      case 1:
        #ifdef REPORT_MODE
          Serial.println(F("Trn R"));
        #endif

        #ifdef USE_MOTORS
          DC2_moveA(rbt_mtrspd_max, -rbt_mtrspd_max, DC2_MIN_SPD_MV, DC2_MAX_SPD_MV, DC2_SPD_INCR_STEP_TRN, DC2_SPD_INCR_TIME_TRN);
          delay(3*rbt_runtime_min);
        #endif
      break;

      case 2:
        #ifdef REPORT_MODE
          Serial.println(F("Go F"));
        #endif

        #ifdef USE_MOTORS
          DC2_moveA(rbt_mtrspd_max, rbt_mtrspd_max, DC2_MIN_SPD_MV, DC2_MAX_SPD_MV, DC2_SPD_INCR_STEP_MV, DC2_SPD_INCR_TIME_MV);
          delay(3*rbt_runtime_min);
        #endif
      break;

      case 3:
        #ifdef REPORT_MODE
          Serial.println(F("Go B"));
        #endif

        #ifdef USE_MOTORS
          DC2_moveA(-rbt_mtrspd_max, -rbt_mtrspd_max, DC2_MIN_SPD_MV, DC2_MAX_SPD_MV, DC2_SPD_INCR_STEP_TRN, DC2_SPD_INCR_TIME_TRN);
          delay(3*rbt_runtime_min);
        #endif
      break;

      case 4:
        #ifdef REPORT_MODE
          Serial.println(F("~act"));
        #endif
        // Do nothing
      break;

      default:
        #ifdef REPORT_MODE
          Serial.println(F("N/A"));
        #endif
        // Do nothing
      break;
    }
}

// ------------------------------------------------------------------------------------------------------------
int saveToEEPROM_double2D(double** Arr, int r_cnt, int c_cnt, int add)
{
  /*
   * Save 2D Array passed as double pointer to EEPROM
   * Based on Arduino documentation on EEPROM
   * https://docs.arduino.cc/learn/programming/eeprom-guide
   * 
   * Arguments:
   * - double** Arr - 2D array as double pointer
   * - int r_cnt - number of rows
   * - int c_cnt - number of columns
   * - int add - EEPROM adress to start with
   * 
   * Return:
   * - int add - next EEPROM adress just after saved array 
   * 
   * Depends/Updates
   *  - library <EEPROM.h>
   * 
   */
  for(int r = 0; r < r_cnt; r++)
  {
    for(int c = 0; c < c_cnt; c++)
    {
      EEPROM.put(add, Arr[r][c]);
      add += sizeof(double);
    }
  } 

  return add;
}

// ------------------------------------------------------------------------------------------------------------
int readFromEEPROM_double2D(double** Arr, int r_cnt, int c_cnt, int add)
{
  /*
   * Read data from EEPROM into 2D Array (passed as double pointer)
   * Based on Arduino documentation on EEPROM
   * https://docs.arduino.cc/learn/programming/eeprom-guide
   * 
   * Arguments:
   * - double** Arr - 2D array as double pointer
   * - int r_cnt - number of rows
   * - int c_cnt - number of columns
   * - int add - EEPROM adress to start with
   * 
   * Return:
   * - int add - next EEPROM adress just after finishing reading array Arr 
   * 
   * Depends/Updates
   *  - library <EEPROM.h>
   */
   
  for(int r = 0; r < r_cnt; r++)
  {
    for(int c = 0; c < c_cnt; c++)
    {
      EEPROM.get(add, Arr[r][c]);
      add += sizeof(double);
    }
  } 

  return add;
}

// ------------------------------------------------------------------------------------------------------------
int saveNNToEEPROM(unsigned int cnt_alerts, unsigned int cnt_learn)
{

  // Save the robot's NN weights and some related data to EEPROM

  // reset starting address
  int eeprom_addr = 0; 

  // Save control bytes */
  EEPROM.put(eeprom_addr, eepromControl1);
  eeprom_addr += sizeof(byte);
  EEPROM.put(eeprom_addr, eepromControl2);
  eeprom_addr += sizeof(byte);
  EEPROM.put(eeprom_addr, eepromControl3);
  eeprom_addr += sizeof(byte);
  EEPROM.put(eeprom_addr, eepromControl4);
  eeprom_addr += sizeof(byte);

  // Save each layer nodes biases, bias weights and input weights (excluding input layer)   
  // for each layer (excluding input layer
  for(byte l_idx = 1; l_idx < Robo_NN.getCntL(); l_idx++)
  {
    // for each node
    for(byte n = 0; n < Robo_NN.getCntN(l_idx); n++)
    {
      // save biases
      EEPROM.put(eeprom_addr, Robo_NN.getB(l_idx, n));
      eeprom_addr += sizeof(double);  
        
      // save biases weights
      EEPROM.put(eeprom_addr, Robo_NN.getWb(l_idx, n));
      eeprom_addr += sizeof(double);  

      // for each input weight
      for(byte w = 0; w < Robo_NN.getCntW(l_idx); w++)
      {
        // save inputs weights
        EEPROM.put(eeprom_addr, Robo_NN.getW(l_idx, n , w));
        eeprom_addr += sizeof(double);    
      } // end looping input weights  
    } // end looping nodes    
  } // end looping layers
   
  // save context neurones values
  for(byte i = 0; i < Robo_Brain.getCntC(); i++)
  {
    EEPROM.put(eeprom_addr, Robo_Brain.getValC(i)); 
    eeprom_addr += sizeof(double); 
  }
        
  // save counters
  EEPROM.put(eeprom_addr, cnt_alerts);
  eeprom_addr += sizeof(unsigned int);
  EEPROM.put(eeprom_addr, cnt_learn);
  eeprom_addr += sizeof(unsigned int);

  return eeprom_addr;
  
}

int loadNNFromEEPROM(unsigned int& cnt_alerts, unsigned int& cnt_learn)
{

  // Loads the robot's NN weights and some related data from EEPROM
  // Arguments:
  // - unsigned int cnt_alerts - counnter, stores quantity of obstackes encountered by robot
  // - unsigned int cnt_learn - counnter, stores quantity of single learnings
  // 
  // Return:
  // - int eeprom_addr - next EEPROM adress just after finishing reading NN data

  // set starting address
  int eeprom_addr = 0; 

  // temp variable
  double temp = 0;

  // Load eeprom control bytes and set succes flag if they are OK
  byte controlCheck1;
  byte controlCheck2;
  byte controlCheck3;
  byte controlCheck4;
  EEPROM.get(eeprom_addr, controlCheck1);
  eeprom_addr += sizeof(byte);
  EEPROM.get(eeprom_addr, controlCheck2);
  eeprom_addr += sizeof(byte);
  EEPROM.get(eeprom_addr, controlCheck3);
  eeprom_addr += sizeof(byte);
  EEPROM.get(eeprom_addr, controlCheck4);
  eeprom_addr += sizeof(byte);

  if((controlCheck1==eepromControl1) && (controlCheck2==eepromControl2) && (controlCheck3==eepromControl3) && (controlCheck4==eepromControl4))
  {
     isNNLoadedFromEEPROM = true;
  }
  else
  {
     isNNLoadedFromEEPROM = false;
  }

  // Load each layer nodes biases, bias weights and input weights (excluding input layer)   
  // for each layer (excluding input layer
  for(byte l_idx = 1; l_idx < Robo_NN.getCntL(); l_idx++)
  {
    // for each node
    for(byte n = 0; n < Robo_NN.getCntN(l_idx); n++)
    {
      // load biases
      EEPROM.get(eeprom_addr, temp);
      Robo_NN.setB(temp, l_idx, n);
      eeprom_addr += sizeof(double);
        
      // load biases weights
      EEPROM.get(eeprom_addr, temp);
      Robo_NN.setWb(temp, l_idx, n);
      eeprom_addr += sizeof(double);

      // for each input weight
      for(byte w = 0; w < Robo_NN.getCntW(l_idx); w++)
      {
        // load inputs weights
        EEPROM.get(eeprom_addr, temp);
        Robo_NN.setW(temp, l_idx, n, w);
        eeprom_addr += sizeof(double);            
       } // end looping weights   
    } // end looping nodes   
  } // end looping layers

  // load context neurones
  for(byte i = 0; i < Robo_Brain.getCntC(); i++)
  {
    EEPROM.get(eeprom_addr, temp);
    Robo_Brain.setValC(temp, i);
    eeprom_addr += sizeof(double);
  }
  
  // load counters
  EEPROM.get(eeprom_addr, cnt_alerts);
  eeprom_addr += sizeof(unsigned int);
  EEPROM.get(eeprom_addr, cnt_learn);
  eeprom_addr += sizeof(unsigned int);

  return eeprom_addr;  
}


// ------------------------------------------------------------------------------------------------------------
void display_freeram() 
{
  #ifdef MIN_REPORT_MODE
    Serial.print(F(" - SRAM left: "));
    Serial.println(freeRam());
  #endif
}

int freeRam() 
{
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0  ? (int)&__heap_start : (int) __brkval);  
}
 
