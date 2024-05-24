/*
 *
 * pnpControl.c - the controller for the pick and place machine in manual and autonomous mode
 *
 * Platform: Any POSIX compliant platform
 * Intended for and tested on: Cygwin 64 bit
 *
 */

#include "pnpControl.h"

// state names and numbers
#define HOME                0
#define MOVE_TO_FEEDER      1
#define WAIT_1              2
#define LOWER_CNTR_NOZZLE   3       //lowering the centre nozzle
#define VAC_CNTR_NOZZLE     4       //applying the vacuum for the centre nozzle
#define RAISE_CNTR_NOZZLE   5       //raising the centre nozzle
#define MOVE_TO_CAMERA      6
#define LOOK_UP_PHOTO       7
#define MOVE_TO_PCB         8
#define CHECK_ERROR         9
#define CORRECT_ERRORS      10

#define holdingpart         1
#define not_holdingpart     0

/* state_names of up to 19 characters (the 20th character is a null terminator), only required for display purposes */
const char state_name[11][20] = {"HOME               ",
                                "MOVE TO FEEDER     ",
                                "WAIT 1             ",
                                "LOWER CNTR NOZZLE  ",
                                "VAC CNTR NOZZLE    ",
                                "RAISE CNTR NOZZLE  ",
                                "MOVE TO CAMERA     ",
                                "LOOK UP PHOTO      ",
                                "MOVE TO PCB        ",
                                "CHECK ERROR        ",
                                "CORRECT ERRORS     "};

const double TAPE_FEEDER_X[NUMBER_OF_FEEDERS] = {FDR_0_X, FDR_1_X, FDR_2_X, FDR_3_X, FDR_4_X, FDR_5_X, FDR_6_X, FDR_7_X, FDR_8_X, FDR_9_X};
const double TAPE_FEEDER_Y[NUMBER_OF_FEEDERS] = {FDR_0_Y, FDR_1_Y, FDR_2_Y, FDR_3_Y, FDR_4_Y, FDR_5_Y, FDR_6_Y, FDR_7_Y, FDR_8_Y, FDR_9_Y};

const char nozzle_name[3][10] = {"left", "centre", "right"};


int main()
{
    pnpOpen();

    int operation_mode, number_of_components_to_place, res;
    PlacementInfo pi[MAX_NUMBER_OF_COMPONENTS_TO_PLACE];

    /*
     * read the centroid file to obtain the operation mode, number of components to place
     * and the placement information for those components
     */
    res = getCentroidFileContents(&operation_mode, &number_of_components_to_place, pi);

    if (res != CENTROID_FILE_PRESENT_AND_READ)
    {  //throw an error if the centroid file is unreadable or not present
        printf("Problem with centroid file, error code %d, press any key to continue\n", res);
        getchar();
        exit(res);
    }

    /* state machine code for manual control mode */
    if (operation_mode == MANUAL_CONTROL)
    {
        /* initialization of variables and controller window */
        int state = HOME, finished = FALSE;

        char NozzleStatus = not_holdingpart;  //initiating nozzle status
        char c;
        int part_counter = 0;
        double requested_theta;
        double preplace_diff_x, preplace_diff_y;

        printf("Time: %7.2f  Initial state: %.15s  Operating in manual control mode, there are %d parts to place\n\n", getSimulationTime(), state_name[HOME], number_of_components_to_place);
        /* print details of part 0 */
        printf("Part 0 details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
               pi[0].component_designation, pi[0].component_footprint, pi[0].component_value, pi[0].x_target, pi[0].y_target, pi[0].theta_target, pi[0].feeder);

        /* loop until user quits */
        while(!isPnPSimulationQuitFlagOn())
        {

            c = getKey();  //saves the value of the key pressed by the user

            switch (state)
            {
                case HOME:
                    //gantry in home position, waiting for input by user to initiate movement to feeder
                    if (finished == FALSE && (c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9'))
                    {
                        /* the expression (c - '0') obtains the integer value of the number key pressed */
                        setTargetPos(TAPE_FEEDER_X[c - '0'], TAPE_FEEDER_Y[c - '0']);
                        state = MOVE_TO_FEEDER;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to move to tape feeder %c\n", getSimulationTime(), state_name[state], c);
                    }
                    break;

                case MOVE_TO_FEEDER:
                    //waiting for the simulator to complete movement of the gantry
                    if (isSimulatorReadyForNextInstruction())
                    {
                        state = WAIT_1;
                        printf("Time: %7.2f  New state: %.20s  Arrived at feeder, waiting for next instruction\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case WAIT_1:    //waiting for next key press
                    //'p' for pickup
                    if((c == 'p') && (NozzleStatus == not_holdingpart))  //the nozzle needs to be empty before initiating pickup
                    {
                        lowerNozzle(CENTRE_NOZZLE);
                        state = LOWER_CNTR_NOZZLE;
                        printf("Time: %7.2f  New state: %.20s Issued instruction to pick up part\n", getSimulationTime(), state_name[state]);
                    }

                    //'c' for camera
                    if((c == 'c') && (NozzleStatus == holdingpart))
                    {
                        setTargetPos(LOOKUP_CAMERA_X,LOOKUP_CAMERA_Y);  //the gantry will move to the position above the camera
                        state = MOVE_TO_CAMERA;      //after the nozzle picked up a part, send the gantry to the lookup camera
                        printf("Time: %7.2f  New state: %.20s Issued instruction to move to look-up camera\n", getSimulationTime(), state_name[state]);
                    }

                    //'r' for rotate to fix the misalignment errors
                    if(c == 'r')
                    {
                        rotateNozzle(CENTRE_NOZZLE, requested_theta);
                        state = CORRECT_ERRORS;
                        printf("Time: %7.2f  New state: %.20s Correcting part misalignment on nozzle\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case LOWER_CNTR_NOZZLE:
                    if (isSimulatorReadyForNextInstruction())
                    {
                        applyVacuum(CENTRE_NOZZLE);
                        state = VAC_CNTR_NOZZLE;
                        printf("Time: %7.2f  New state: %.20s Applying vacuum\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case VAC_CNTR_NOZZLE:
                     if (isSimulatorReadyForNextInstruction())
                    {
                        raiseNozzle(CENTRE_NOZZLE);
                        state = RAISE_CNTR_NOZZLE;
                        printf("Time: %7.2f  New state: %.20s Raising nozzle\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case RAISE_CNTR_NOZZLE:
                    if (isSimulatorReadyForNextInstruction())
                    {
                        state = WAIT_1;
                        NozzleStatus = holdingpart;
                        printf("Time: %7.2f  New state: %.20s Part acquired. Ready for next instruction\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case MOVE_TO_CAMERA:
                    if (isSimulatorReadyForNextInstruction())
                    {
                        takePhoto(PHOTO_LOOKUP);
                        state = LOOK_UP_PHOTO;
                        printf("Time: %7.2f  New state: %.20s Taking look-up photo of part\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case LOOK_UP_PHOTO:
                    if (isSimulatorReadyForNextInstruction())
                    {
                        setTargetPos(pi[part_counter].x_target, pi[part_counter].y_target);
                        state = MOVE_TO_PCB;
                        printf("Time: %7.2f  New state: %.20s Look-up photo taken of acquired part. Moving to PCB\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case MOVE_TO_PCB:
                    if (isSimulatorReadyForNextInstruction())
                    {
                        takePhoto(PHOTO_LOOKDOWN);
                        state = CHECK_ERROR;
                        printf("Time: %7.2f  New state: %.20s Look-down photo taken of acquired part. Calculating misalignment errors\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case CHECK_ERROR:
                    if (isSimulatorReadyForNextInstruction())
                    {
                        double errortheta = getPickErrorTheta(CENTRE_NOZZLE);
                        requested_theta = pi[part_counter].theta_target - errortheta;  //checking for misalignment after the part has been picked up
                        preplace_diff_x = pi[part_counter].x_target - getPreplaceErrorX();
                        preplace_diff_y = pi[part_counter].y_target - getPreplaceErrorY();
                        state = WAIT_1;
                        printf("Time: %7.2f  New state: %.20s Part misalignment error: %7.2f\n", getSimulationTime(), state_name[state], errortheta);
                        printf("                Preplace misalignment error: x=%7.2f y=%7.2f\n", getPreplaceErrorX(), getPreplaceErrorY());
                    }
                    break;

                case CORRECT_ERRORS:
                    if (isSimulatorReadyForNextInstruction())
                    {
                        amendPos(preplace_diff_x, preplace_diff_y);
                        state = WAIT_1;
                        printf("Time: %7.2f  New state: %.20s Correction made to gantry preplace position\n", getSimulationTime(), state_name[state]);
                    }
                    break;



//
//                case PART_DROP:
//
//                    lowerNozzle(CENTRE_NOZZLE);
//                    releaseVacuum(CENTRE_NOZZLE);
//                    raiseNozzle(CENTRE_NOZZLE);
//
//                    break;

            }
            sleepMilliseconds((long) 1000 / POLL_LOOP_RATE);
        }
    }
    /* state machine code for autonomous control mode */
    else
    {
//        if (operation_mode == MANUAL_CONTROL)
//        {
//            /* initialization of variables and controller window */
//            int state = HOME, finished = FALSE;
//
//            char c;
//            printf("Time: %7.2f  Initial state: %.15s  Operating in automatic mode, there are %d parts to place\n\n", getSimulationTime(), state_name[HOME], number_of_components_to_place);
//            /* print details of part 0 */
//            printf("Part 0 details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
//               pi[0].component_designation, pi[0].component_footprint, pi[0].component_value, pi[0].x_target, pi[0].y_target, pi[0].theta_target, pi[0].feeder);
//
//            /* loop until user quits */
//            while(!isPnPSimulationQuitFlagOn())
//            {
//
//                c = getKey();
//
//                switch (state)
//                {
//
//                }
//            }
//        }

    }

    pnpClose();
    return 0;
}

