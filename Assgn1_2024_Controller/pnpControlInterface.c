/*
 *
 * pnpControlInterface.c - the interface routines for pick and place machine control, which simplify
 * interfacing to the simulator
 *
 * This program creates a shared memory segment with the simulator via a memory mapped file
 *
 * Platform: Any POSIX compliant platform
 * Intended for and tested on: Cygwin 64 bit
 *
 */

#include "pnpControl.h"

PnP *pnp;
int fd;
struct termios old_term;
pthread_t key_thread;
char key_pressed;

/*
 Function: setTerminalSettings
 -----------------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 sets the terminal settings to disable
 character echoing and line buffering
 Argument(s): none
 Return Value:
 a termios struct representing the original
 terminal settings for future restoration
 Usage: struct termios old_term = setTerminalSettings();
 */
struct termios setTerminalSettings() {

    struct termios old_term, new_term;

     /* Get old terminal settings for future restoration */
    tcgetattr(0, &old_term);

    /* Copy the settings to the new value */
    new_term = old_term;

    /* Disable echo of the character and line buffering */
    new_term.c_lflag &= (~ICANON & ~ECHO);

    /* Set new settings to the terminal */
    tcsetattr(0, TCSANOW, &new_term);

    return old_term;
}

/*
 Function: resetTerminalSettings
 -------------------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose: resets the terminal settings
 Argument(s):
 struct termios old_term - terminal settings to restore
 Return Value: none
 Usage: resetTerminalSettings(old_term);
 */
void resetTerminalSettings(struct termios old_term) {

    /* Restore old settings */
    tcsetattr(0, TCSANOW, &old_term);

}

/*
 Function: getCentroidFileContents
 ---------------------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 gets the contents of the centroid file (including placement info of components) if it exists in the
 current working directory and if its contents are valid.
 Argument(s):
 The following arguments are passed by reference and so are available to the calling function:
 int *operation_mode - a pointer to an integer variable representing the operation mode (manual or auto)
 int *number_of_components_to_place - a pointer to an integer variable representing the number of components to place
 PlacementInfo pi[] - a pointer to an array of structures, with each structure representing the placement info of one component
 Return Value:
 one of:
 CENTROID_FILE_PRESENT_AND_READ (0)
 CENTROID_FILE_NOT_PRESENT (-1)
 CENTROID_FILE_PRESENT_BUT_CONTENT_ISSUE (-2)
 CENTROID_FILE_HAS_TOO_MANY_COMPONENTS (-3)
 Usage:
 int res = getCentroidFileContents(&operation_mode, &number_of_components_to_place, placementInfo);
 */
int getCentroidFileContents(int *operation_mode, int *number_of_components_to_place, PlacementInfo pi[MAX_NUMBER_OF_COMPONENTS_TO_PLACE])
{

    char dummy_char = 'z';
    char * operation_mode_char = &dummy_char;

    FILE *fp = fopen(CENTROID_FILE, "r");

    if (fp == NULL) return CENTROID_FILE_NOT_PRESENT;

    if (fscanf(fp, "%c", operation_mode_char) != 1) {fclose(fp); return CENTROID_FILE_PRESENT_BUT_CONTENT_ISSUE;}

    if (*operation_mode_char == 'm' || *operation_mode_char == 'M') *operation_mode = MANUAL_CONTROL;
    else if (*operation_mode_char == 'a' || *operation_mode_char == 'A') *operation_mode = AUTONOMOUS_CONTROL;
    else {fclose(fp); return CENTROID_FILE_PRESENT_BUT_CONTENT_ISSUE;}

    if (fscanf(fp, "%i", number_of_components_to_place) != 1) {fclose(fp); return CENTROID_FILE_PRESENT_BUT_CONTENT_ISSUE;}
    if (*number_of_components_to_place > MAX_NUMBER_OF_COMPONENTS_TO_PLACE) {fclose(fp); return CENTROID_FILE_HAS_TOO_MANY_COMPONENTS;}

    for (int i = 0; i < *number_of_components_to_place; i++)
    {
        if (fscanf(fp, "%s %s %lf %lf %lf %lf %i", &pi[i].component_designation[0], &pi[i].component_footprint[0], &pi[i].component_value, &pi[i].x_target, &pi[i].y_target, &pi[i].theta_target, &pi[i].feeder) != NUMBER_OF_FIELDS_IN_PLACEMENT_INFO) {fclose(fp); return CENTROID_FILE_PRESENT_BUT_CONTENT_ISSUE;};
    }
    fclose(fp);
    return CENTROID_FILE_PRESENT_AND_READ;

}

/*
 Function: setTargetPos
 ----------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 instructs the simulator to move the gantry head to the specified target position
 Argument(s):
 double x_target - the target x-coordinate of the gantry head
 double y_target - the target y-coordinate of the gantry head
 Return Value:
 None, the instruction to move the head to the specified target position will always be passed to the simulator, check the simulator
 display output to see whether or not the simulator acted upon the instruction
 Usage:
 setTargetPos(x_target, y_target);
 */
void setTargetPos(double x_target, double y_target)
{

    pnp -> instruction_argument_1 = x_target;
    pnp -> instruction_argument_2 = y_target;
    pnp -> instruction_argument_3 = 0; // instruction_argument_3 is not used with the MOVE_HEAD instruction
    pnp -> instruction_to_execute = MOVE_HEAD;

}

/*
 Function: amendPos
 ------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 instructs the simulator to finely tune the gantry head position to eliminate alignment errors
 Argument(s):
 double del_x - the requested positive or negtive change in the x-coordinate of the gantry head
 double del_y - the requested positive or negtive change in the y-coordinate of the gantry head
 Return Value:
 None, the instruction to finely tune the head position by the specified amount will always be passed to the simulator, check the simulator
 display output to see whether or not the simulator acted upon the instruction
 Usage:
 amendPos(del_x, del_y);
 */
void amendPos(double del_x, double del_y)
{

    pnp -> instruction_argument_1 = del_x;
    pnp -> instruction_argument_2 = del_y;
    pnp -> instruction_argument_3 = 0; // instruction_argument_3 is not used with the AMEND_HEAD instruction
    pnp -> instruction_to_execute = AMEND_HEAD_POSITION;

}

/*
 Function: lowerNozzle
 ---------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 instructs the simulator to lower the specified nozzle
 Argument(s):
 int nozzle - the nozzle to lower
 Return Value:
 None, the instruction to lower the specified nozzle will always be passed to the simulator, check the simulator
 display output to see whether or not the simulator acted upon the instruction
 Usage:
 lowerNozzle(nozzle);
 */
void lowerNozzle(int nozzle)
{

    pnp -> instruction_argument_1 = 0.0;    // instruction_argument_1 is not used with the LOWER_NOZZLE instruction
    pnp -> instruction_argument_2 = 0.0;    // instruction_argument_2 is not used with the LOWER_NOZZLE instruction
    pnp -> instruction_argument_3 = nozzle;
    pnp -> instruction_to_execute = LOWER_NOZZLE;

}

/*
 Function: raiseNozzle
 ---------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 instructs the simulator to raise the specified nozzle
 Argument(s):
 int nozzle - the nozzle to raise
 Return Value:
 None, the instruction to raise the specified nozzle will always be passed to the simulator, check the simulator
 display output to see whether or not the simulator acted upon the instruction
 Usage:
 raiseNozzle(nozzle);
 */
void raiseNozzle(int nozzle)
{

    pnp -> instruction_argument_1 = 0.0;    // instruction_argument_1 is not used with the RAISE_NOZZLE instruction
    pnp -> instruction_argument_2 = 0.0;    // instruction_argument_2 is not used with the RAISE_NOZZLE instruction
    pnp -> instruction_argument_3 = nozzle;
    pnp -> instruction_to_execute = RAISE_NOZZLE;

}

/*
 Function: rotateNozzle
 ----------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 instructs the simulator to rotate the specified nozzle by a specified positive or negative angle in degrees
 Argument(s):
 int nozzle - the nozzle to rotate
 double angleInDegrees - the positive or negative angle of rotation in degrees
 Return Value:
 None, the instruction to rotate the specified nozzle will always be passed to the simulator, check the simulator
 display output to see whether or not the simulator acted upon the instruction
 Usage:
 rotateNozzle(nozzle, angleInDegrees);
 */
void rotateNozzle(int nozzle, double angleInDegrees)
{

    pnp -> instruction_argument_1 = angleInDegrees;
    pnp -> instruction_argument_2 = 0.0;    // instruction_argument_2 is not used with the ROTATE_NOZZLE instruction
    pnp -> instruction_argument_3 = nozzle;
    pnp -> instruction_to_execute = ROTATE_NOZZLE;

}

/*
 Function: applyVacuum
 ---------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 instructs the simulator to apply vacuum suction to the specified nozzle
 Argument(s):
 int nozzle - the nozzle to apply the vacuum suction to
 Return Value:
 None, the instruction to apply vacuum suction to the specified nozzle will always be passed to the simulator, check the simulator
 display output to see whether or not the simulator acted upon the instruction
 Usage:
 applyVacuum(nozzle);
 */
void applyVacuum(int nozzle)
{

    pnp -> instruction_argument_1 = 0.0;    // instruction_argument_1 is not used with the APPLY_VACUUM instruction
    pnp -> instruction_argument_2 = 0.0;    // instruction_argument_2 is not used with the APPLY_VACUUM instruction
    pnp -> instruction_argument_3 = nozzle;
    pnp -> instruction_to_execute = APPLY_VACUUM;

}

/*
 Function: releaseVacuum
 -----------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 instructs the simulator to release vacuum suction from the specified nozzle
 Argument(s):
 int nozzle - the nozzle to release the vacuum suction from
 Return Value:
 None, the instruction to release vacuum suction from the specified nozzle will always be passed to the simulator, check the simulator
 display output to see whether or not the simulator acted upon the instruction
 Usage:
 releaseVacuum(nozzle);
 */
void releaseVacuum(int nozzle)
{

    pnp -> instruction_argument_1 = 0.0;    // instruction_argument_1 is not used with the RELEASE_VACUUM instruction
    pnp -> instruction_argument_2 = 0.0;    // instruction_argument_2 is not used with the RELEASE_VACUUM instruction
    pnp -> instruction_argument_3 = nozzle;
    pnp -> instruction_to_execute = RELEASE_VACUUM;

}

/*
 Function: takePhoto
 -------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 instructs the simulator to take a photo using the specified camera
 Argument(s):
 int camera - the camera with which to take the photo (should be the lookup or lookdown camera)
 Return Value:
 None, the instruction to take the photo with the specified camera will always be passed to the simulator, check the simulator
 display output to see whether or not the simulator acted upon the instruction
 Usage:
 takePhoto(camera);
 */
void takePhoto(int camera)
{

    pnp -> instruction_argument_1 = 0.0;    // instruction_argument_1 is not used with the TAKE_PHOTO instruction
    pnp -> instruction_argument_2 = 0.0;    // instruction_argument_2 is not used with the TAKE_PHOTO instruction
    pnp -> instruction_argument_3 = camera;
    pnp -> instruction_to_execute = TAKE_PHOTO;

}

/*
 Function: getKeyPress
 ---------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 thread function to handle keyboard input, called as part
 of creation of a new thread
 Argument(s):  None
 Return Value: none
 Usage: not called directly but via pthread_create()
 */
void *getKeyPress(void *arguments)
{
    do {

        key_pressed = getchar();

    } while ((key_pressed != 'q') && (key_pressed != 'Q'));

    pnp -> quit = TRUE;
    return NULL;
}

/*
 Function: pnpOpen
 -------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose: sets the terminal settings, creates a separate thread to handle
 keyboard input, initializes and memory maps a file so that a shared memory
 segment is created with the simulator
 Argument(s): none
 Return Value: none
 Usage: pnpOpen();
 */
void pnpOpen()
{
    /* disable character echoing and line buffering */
    old_term = setTerminalSettings();

    /* create separate thread to handle keyboard input */
    int res = pthread_create(&key_thread, NULL, getKeyPress, NULL);
    if (res != 0)
    {
        perror("Problem creating thread to handle user input");
        exit(1);
    }

    /* initialize file */
    fd = open(MEMORY_MAPPED_FILE, (O_CREAT | O_RDWR), 0666);
    if (fd < 0)
    {
        perror("creation/opening of file failed");
        exit(1);
    }
    ftruncate(fd, sizeof(PnP));

    /* map the file to memory */
    pnp = (PnP *)mmap(0, sizeof(PnP), (PROT_READ | PROT_WRITE),  MAP_SHARED, fd, (off_t)0);
    if (pnp == MAP_FAILED)
    {
        perror("memory mapping of file failed");
        close(fd);
        exit(2);
    }
}

/*
 Function: pnpClose
 ------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose: indicates to the simulator that the controller is quitting,
 unmaps the memory mapped file, closes the associated file descriptor
 and resets the terminal settings
 Argument(s): none
 Return Value: none
 Usage: pnpClose();
 */
void pnpClose()
{
    pnp -> quit = TRUE;
    munmap(pnp, sizeof(PnP));
    close(fd);

    /* reset terminal settings to original values */
    resetTerminalSettings(old_term);
}

/*
 Function: getSimTime
 --------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 gets the current simulation time from the simulator in seconds,
 this is not necessarily real time
 Argument(s):
 none
 Return Value:
 a double representing current simulation time
 Usage:
 double simTime = getSimTime();
 */
double getSimTime()
{
    return round(10.0 * pnp -> sim_time)/10.0;
}

/*
 Function: getSimulationTime
 ---------------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 gets the current simulation time from the simulator in seconds,
 this is not necessarily real time
 Argument(s):
 none
 Return Value:
 a double representing current simulation time
 Usage:
 double simTime = getSimulationTime();
 */
double getSimulationTime()
{
    return pnp -> sim_time;
}

/*
 Function: getPreplaceErrorX
 ---------------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 gets the error in the x-positioning of the gantry head when about to place a part (assuming a lookdown photo has already been taken)
 Argument(s):
 none
 Return Value:
 a double representing the positive or negative error in the x-positioning of the gantry head
 Usage:
 double x_preplace_error = getPreplaceErrorX();
 */
double getPreplaceErrorX()
{
    return pnp -> x_preplace_error;
}

/*
 Function: getPreplaceErrorY
 ---------------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 gets the error in the y-positioning of the gantry head when about to place a part (assuming a lookdown photo has already been taken)
 Argument(s):
 none
 Return Value:
 a double representing the positive or negative error in the y-positioning of the gantry head
 Usage:
 double y_preplace_error = getPreplaceErrorY();
 */
double getPreplaceErrorY()
{
    return pnp -> y_preplace_error;
}

/*
 Function: getPickErrorTheta
 ---------------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 gets the error in the angular rotation of the picked part on the specified nozzle (assuming a lookup photo has already been taken)
 Argument(s):
 int nozzle - the nozzle for which the pick error must be determined
 Return Value:
 a double representing the positive or negative angular error in degrees of the picked part on the specified nozzle
 Usage:
 double theta_pick_error[nozzle] = getPickErrorTheta(nozzle);
 */
double getPickErrorTheta(int nozzle)
{
    return pnp -> theta_pick_error[nozzle];
}

/*
 Function: isSimulatorReadyForNextInstruction
 --------------------------------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 provides information on whether the simulator has finished executing the previous instruction
 Argument(s):
 none
 Return Value:
 an int representing whether the simulator has finished executing the previous instruction (1) or not (0)
 Usage:
 int simulatorIsReadyForNextInstruction = isSimulatorReadyForNextInstruction();
 */
int isSimulatorReadyForNextInstruction()
{
    return pnp -> ready_for_next_instruction;
}

/*
 Function: getKey
 -------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 gets the most recent key press by the user which has not already been handled, then resets
 the key to NO_KEY to prevent handling the same key press more than once
 Argument(s):
 none
 Return Value:
 the most recent key press by the user which has not already been handled as a char,
 otherwise NO_KEY (0)
 Usage:
 char c = getKey();
 */
char getKey()
{
    char c;
    c = key_pressed;
    key_pressed = NO_KEY;
    return c;
}

/*
 Function: isPnPSimulationQuitFlagOn
 -------------------------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose:
 determines whether the simulator is quitting after receiving a 'q' key press
 Argument(s):
 none
 Return Value:
 one of:
 FALSE (0) - quit flag off
 TRUE (1) - quit flag on
 Usage:
 int quiteFlagOn = isPnPSimulationQuitFlagOn();
 */
int isPnPSimulationQuitFlagOn()
{
    return pnp -> quit;
}

/*
 Function: sleepMilliseconds
 ---------------------------
 Written by Jason Brown
 Date: 30/03/2024
 Version 1.0
 Purpose: put the calling thread to sleep
 for a certain number of ms
 Argument(s):
 long ms - the number of ms to sleep
 Return Value: none
 Usage: sleepMilliseconds(20);
 */
void sleepMilliseconds(long ms)
{
    struct timespec ts;

    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
}
