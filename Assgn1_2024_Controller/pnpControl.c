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
#define CNOZZLE_LOWER       3       //lowering the centre nozzle
#define CNOZZLE_VAC         4       //applying the vacuum for the centre nozzle
#define CNOZZLE_RAISE       5       //raising the centre nozzle
#define LOOKUPCAM           6

#define holdingpart         1
#define not_holdingpart     0

/* state_names of up to 19 characters (the 20th character is a null terminator), only required for display purposes */
const char state_name[5][20] = {"HOME               ",
                                "MOVE TO FEEDER     ",
                                "WAIT 1             ",
                                "PART PICKUP        ",
                                "PART DROP          "};

const double TAPE_FEEDER_X[NUMBER_OF_FEEDERS] = {FDR_0_X, FDR_1_X, FDR_2_X, FDR_3_X, FDR_4_X, FDR_5_X, FDR_6_X, FDR_7_X, FDR_8_X, FDR_9_X};
const double TAPE_FEEDER_Y[NUMBER_OF_FEEDERS] = {FDR_0_Y, FDR_1_Y, FDR_2_Y, FDR_3_Y, FDR_4_Y, FDR_5_Y, FDR_6_Y, FDR_7_Y, FDR_8_Y, FDR_9_Y};

const char nozzle_name[3][10] = {"left", "centre", "right"};

char NozzleStatus = not_holdingpart;

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
    {
        printf("Problem with centroid file, error code %d, press any key to continue\n", res);
        getchar();
        exit(res);
    }

    /* state machine code for manual control mode */
    if (operation_mode == MANUAL_CONTROL)
    {
        /* initialization of variables and controller window */
        int state = HOME, finished = FALSE;

        char c;
        printf("Time: %7.2f  Initial state: %.15s  Operating in manual control mode, there are %d parts to place\n\n", getSimulationTime(), state_name[HOME], number_of_components_to_place);
        /* print details of part 0 */
        printf("Part 0 details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
               pi[0].component_designation, pi[0].component_footprint, pi[0].component_value, pi[0].x_target, pi[0].y_target, pi[0].theta_target, pi[0].feeder);

        /* loop until user quits */
        while(!isPnPSimulationQuitFlagOn())
        {

            c = getKey();

            switch (state)
            {
                case HOME:

                    if (finished == FALSE && (c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9'))
                    {
                        /* the expression (c - '0') obtains the integer value of the number key pressed */
                        setTargetPos(TAPE_FEEDER_X[c - '0'], TAPE_FEEDER_Y[c - '0']);
                        state = MOVE_TO_FEEDER;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to move to tape feeder %c\n", getSimulationTime(), state_name[state], c);
                    }
                    break;

                case MOVE_TO_FEEDER:

                    if (isSimulatorReadyForNextInstruction())
                    {
                        state = WAIT_1;
                        printf("Time: %7.2f  New state: %.20s  Arrived at feeder, waiting for next instruction\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case WAIT_1:
                    if((c == 'p') && (NozzleStatus == not_holdingpart))
                    {
                        state = CNOZZLE_LOWER;
                        printf("Time: %7.2f  New state: %.20s Ready to pick up part. Lowering centre nozzle...\n", getSimulationTime(), state_name[state]);
                    }

                    if((c == 'c') && (NozzleStatus == holdingpart))
                    {
                        state = LOOKUPCAM;
                        printf("Moving to look up cam...\n");
                    }

                    break;

                case CNOZZLE_LOWER:
                    lowerNozzle(CENTRE_NOZZLE);
                    state = CNOZZLE_VAC;
                    printf("Time: %7.2f  New state: %.20s Nozzle lowered.\n", getSimulationTime(), state_name[state]);
                    break;

                case CNOZZLE_VAC:
                    if (isSimulatorReadyForNextInstruction())
                    {
                        applyVacuum(CENTRE_NOZZLE);
                        state = CNOZZLE_RAISE;
                        printf("Time: %7.2f  New state: %.20s Vacuum applied.\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case CNOZZLE_RAISE:
                    if (isSimulatorReadyForNextInstruction())
                    {
                        raiseNozzle(CENTRE_NOZZLE);
                        state = WAIT_1;
                        NozzleStatus = holdingpart;
                        printf("Time: %7.2f  New state: %.20s Part picked up. Ready for next instruction\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case LOOKUPCAM:

                    if (isSimulatorReadyForNextInstruction())
                    {
                        setTargetPos(LOOKUP_CAMERA_X,LOOKUP_CAMERA_Y);
                        printf("Now waiting to take photo");
                        state = WAIT_1;
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

