/*
 *
 * pnpControl.h - declarations for pick and place machine controller
 *
 * Platform: Any POSIX compliant platform
 * Intended for and tested on: Cygwin 64 bit
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <termios.h>
#include <math.h>

#define MANUAL_CONTROL 1
#define AUTONOMOUS_CONTROL 2

#define MEMORY_MAPPED_FILE "pnp_shared_file"
#define CENTROID_FILE "centroid.txt"

#define MAX_NUMBER_OF_COMPONENTS_TO_PLACE 100
#define NUMBER_OF_FIELDS_IN_PLACEMENT_INFO 7

#define CENTROID_FILE_PRESENT_AND_READ 0
#define CENTROID_FILE_NOT_PRESENT -1
#define CENTROID_FILE_PRESENT_BUT_CONTENT_ISSUE -2
#define CENTROID_FILE_HAS_TOO_MANY_COMPONENTS -3

#define HOME_X 0.0
#define HOME_Y 0.0
#define MIN_X -200.0
#define MIN_Y -200.0
#define MAX_X +1000.0
#define MAX_Y +1000.0

#define NUMBER_OF_FEEDERS 10
#define NO_PICKED_PART -1
#define NO_TAPE_FEEDER_AT_THIS_LOCATION -1
#define FDR_0_X +50.0
#define FDR_1_X +150.0
#define FDR_2_X +250.0
#define FDR_3_X +350.0
#define FDR_4_X +450.0
#define FDR_5_X +550.0
#define FDR_6_X +650.0
#define FDR_7_X +750.0
#define FDR_8_X +850.0
#define FDR_9_X +950.0
#define FDR_0_Y -100.0
#define FDR_1_Y -100.0
#define FDR_2_Y -100.0
#define FDR_3_Y -100.0
#define FDR_4_Y -100.0
#define FDR_5_Y -100.0
#define FDR_6_Y -100.0
#define FDR_7_Y -100.0
#define FDR_8_Y -100.0
#define FDR_9_Y -100.0

#define LOOKUP_CAMERA_X -100
#define LOOKUP_CAMERA_Y +100
#define PHOTO_LOOKUP 0
#define PHOTO_LOOKDOWN 1

#define POLL_LOOP_RATE 50          // poll loops per second - DANGER, changing this can result in unstable or incorrect operation

#define TRUE 1
#define FALSE 0

#define NO_KEY 0

#define NUMBER_OF_NOZZLES 3
#define LEFT_NOZZLE 0
#define CENTRE_NOZZLE 1
#define RIGHT_NOZZLE 2

#define NOZZLE_X_SEPARATION 20

#define NO_INSTRUCTION 0
#define MOVE_HEAD 1
#define ROTATE_NOZZLE 2
#define LOWER_NOZZLE 3
#define RAISE_NOZZLE 4
#define APPLY_VACUUM 5
#define RELEASE_VACUUM 6
#define TAKE_PHOTO 7
#define AMEND_HEAD_POSITION 8

typedef struct
{
    int ready_for_next_instruction;
    double sim_time;
    double theta_pick_error[NUMBER_OF_NOZZLES];
    double x_preplace_error;
    double y_preplace_error;
    int instruction_to_execute;
    double instruction_argument_1;
    double instruction_argument_2;
    int instruction_argument_3;
    int quit;

} PnP;

typedef struct
{
    char component_designation[10];
    char component_footprint[10];
    double component_value;
    double x_target;
    double y_target;
    double theta_target;
    int feeder;

} PlacementInfo;

struct termios setTerminalSettings();

void resetTerminalSettings(struct termios);

int getCentroidFileContents(int*, int*, PlacementInfo[MAX_NUMBER_OF_COMPONENTS_TO_PLACE]);

void setTargetPos(double, double);

void amendPos(double, double);

void lowerNozzle(int);

void raiseNozzle(int);

void rotateNozzle(int, double);

void applyVacuum(int);

void releaseVacuum(int);

void takePhoto(int);

void pnpOpen();

void pnpClose();

double getSimTime();

double getSimulationTime();

double getPreplaceErrorX();

double getPreplaceErrorY();

double getPickErrorX(int);

double getPickErrorY(int);

double getPickErrorTheta(int);

int isSimulatorReadyForNextInstruction();

char getKey();

int isPnPSimulationQuitFlagOn();

void sleepMilliseconds(long);

