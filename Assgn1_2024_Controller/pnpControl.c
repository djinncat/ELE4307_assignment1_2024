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
#define LOOK_DOWN_PHOTO     9
#define CHECK_ERROR         10
#define CORRECT_ERRORS      11
#define MOVE_TO_HOME        12
#define FIX_NOZZLE_ERROR    13
#define FIX_PREPLACE_ERROR  14

#define holdingpart         1
#define not_holdingpart     0

/* state_names of up to 19 characters (the 20th character is a null terminator), only required for display purposes */
const char state_name[15][20] = {"HOME               ",
                                "MOVE TO FEEDER     ",
                                "WAIT 1             ",
                                "LOWER CNTR NOZZLE  ",
                                "VAC CNTR NOZZLE    ",
                                "RAISE CNTR NOZZLE  ",
                                "MOVE TO CAMERA     ",
                                "LOOK UP PHOTO      ",
                                "MOVE TO PCB        ",
                                "LOOK DOWN PHOTO    ",
                                "CHECK ERROR        ",
                                "CORRECT ERRORS     ",
                                "MOVE TO HOME       ",
                                "FIX NOZZLE ERROR   ",
                                "FIX PREPLACE ERROR "};

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
        int state = HOME, finished = FALSE, part_counter = 0;
        char c, part_placed = FALSE, NozzleStatus = not_holdingpart;
        double requested_theta = 0;  //the required angle theta of the nozzle position
        double preplace_diff_x = 0, preplace_diff_y = 0;  //difference in required gantry position and actual gantry position for preplacement

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
                    part_placed = FALSE; //variable to change state actions based on whether a part has just been placed or not. Also resets upon returning to home after placing part
                    if (finished == FALSE && (c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9'))
                    {
                        //check if user inputs a feeder number that is not next in the centroid file to prevent accidentally picking up wrong part
                        if ((c - '0') != pi[part_counter].feeder)
                        {
                            printf("Time: %7.2f  INPUT ERROR  The next part is in feeder %d.\n", getSimulationTime(), pi[part_counter].feeder);
                        }
                        else
                        {   //if the input is accepted, the gantry will move to the feeder position
                            /* the expression (c - '0') obtains the integer value of the number key pressed */
                            setTargetPos(TAPE_FEEDER_X[c - '0'], TAPE_FEEDER_Y[c - '0']);
                            state = MOVE_TO_FEEDER;
                            printf("Time: %7.2f  New state: %.20s  Issued instruction to move to tape feeder %c\n", getSimulationTime(), state_name[state], c);
                        }

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
                    if((c == 'p') && (NozzleStatus == not_holdingpart))  //checking if the nozzle is empty
                    {
                        lowerNozzle(CENTRE_NOZZLE);
                        state = LOWER_CNTR_NOZZLE;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to pick up part. Lowering centre nozzle\n", getSimulationTime(), state_name[state]);
                    }

                    //'p' to place the part that the nozzle is currently holding
//consider applying another condition to stop user if errors are not yet corrected
                    if((c == 'p') && (NozzleStatus == holdingpart))
                    {
                        lowerNozzle(CENTRE_NOZZLE);
                        state = LOWER_CNTR_NOZZLE;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to place part on PCB. Lowering nozzle\n", getSimulationTime(), state_name[state]);
                    }

                    //'c' for camera, should only go to the camera if the nozzle is holding a part
                    if((c == 'c') && (NozzleStatus == holdingpart))
                    {
                        setTargetPos(LOOKUP_CAMERA_X,LOOKUP_CAMERA_Y);  //the gantry will move to the position above the camera
                        state = MOVE_TO_CAMERA;      //after the nozzle picked up a part, send the gantry to the lookup camera
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to move to look-up camera\n", getSimulationTime(), state_name[state]);
                    }

                    //'r' for rotate to fix the nozzle misalignment error
                    if(c == 'r')
                    {
                        rotateNozzle(CENTRE_NOZZLE, requested_theta);  //rotate the nozzle by the required calculated angle theta
                        state = CORRECT_ERRORS;
                        printf("Time: %7.2f  New state: %.20s  Correcting part misalignment on nozzle\n", getSimulationTime(), state_name[state]);
                    }

                    //'a' for adjusting the position of the gantry for preplace misalignment error
                    if(c == 'a')
                    {
                        amendPos(preplace_diff_x, preplace_diff_y); //corrects the position by the calculated difference x and y
                        state = CORRECT_ERRORS;
                        printf("Time: %7.2f  New state: %.20s  Correcting preplace misalignment of gantry\n", getSimulationTime(), state_name[state]);
                    }

                    if(c == 'h')
                    {
                        setTargetPos(HOME_X,HOME_Y);
                        state = MOVE_TO_HOME;
                        printf("Time: %7.2f  New state: %.20s  Moving to home position\n", getSimulationTime(), state_name[state]);
                    }

                    break;

                case LOWER_CNTR_NOZZLE:
                    //Need to wait until simulator is ready before moving on to vacuum
                    if (isSimulatorReadyForNextInstruction())
                    {
                        if(NozzleStatus == not_holdingpart)
                        {   //vacuum will apply when the nozzle is empty
                            applyVacuum(CENTRE_NOZZLE);
                            state = VAC_CNTR_NOZZLE;
                            printf("Time: %7.2f  New state: %.20s  Applying vacuum\n", getSimulationTime(), state_name[state]);
                        }
                        if(NozzleStatus == holdingpart)
                        {   //vacuum will release the part when the nozzle is holding something
                            releaseVacuum(CENTRE_NOZZLE);
                            part_placed = TRUE;  //counter to indicate the part has been placed
                            state = VAC_CNTR_NOZZLE;
                            printf("Time: %7.2f  New state: %.20s  Releasing vacuum to place part\n", getSimulationTime(), state_name[state]);
                        }
                    }
                    break;

                case VAC_CNTR_NOZZLE:
                    //wait until the vacuum action is finished before raising the nozzle
                     if (isSimulatorReadyForNextInstruction())
                    {
                        raiseNozzle(CENTRE_NOZZLE);
                        state = RAISE_CNTR_NOZZLE;
                        printf("Time: %7.2f  New state: %.20s  Raising nozzle\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case RAISE_CNTR_NOZZLE:
                    //once nozzle is raised, if a part hasn't just been placed, then it is determined that a part has just been picked up
                    if (isSimulatorReadyForNextInstruction())
                    {
                        if (part_placed==FALSE)
                        {
                            NozzleStatus = holdingpart;
                            state = WAIT_1;
                            printf("Time: %7.2f  New state: %.20s  Part acquired, ready for next instruction\n", getSimulationTime(), state_name[state]);
                        }
                        //if the vacuum has just released a part, then the part has been placed and the nozzle is free again
                        if (part_placed==TRUE)
                        {
                            NozzleStatus = not_holdingpart;
                            part_counter++;  //increment counter to keep track of the part number in the centroid file that has been placed
                            if (part_counter != number_of_components_to_place)
                            {   //since there are still components to be placed, go back to Home to cycle again. Display the next set of part details
                                state = HOME;
                                printf("Time: %7.2f  New state: %.20s  Part %d placed on PCB successfully\n\n", getSimulationTime(), state_name[state], (part_counter-1));
                                printf("Part %d details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n", part_counter,
                                pi[part_counter].component_designation, pi[part_counter].component_footprint, pi[part_counter].component_value, pi[part_counter].x_target,
                                pi[part_counter].y_target, pi[part_counter].theta_target, pi[part_counter].feeder);
                            }
                            else if(part_counter == number_of_components_to_place)
                            {
                                setTargetPos(HOME_X,HOME_Y);
                                state = MOVE_TO_HOME;
                                printf("Time: %7.2f  New state: %.20s  All parts have been placed! Moving to home\n", getSimulationTime(), state_name[state]);
                            }
                        }
                    }
                    break;

                case MOVE_TO_CAMERA:
                    //waiting for the gantry to move to the camera position before taking look-up photo
                    if (isSimulatorReadyForNextInstruction())
                    {
                        takePhoto(PHOTO_LOOKUP);
                        state = LOOK_UP_PHOTO;
                        printf("Time: %7.2f  New state: %.20s  Arrived at camera. Taking look-up photo of part\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case LOOK_UP_PHOTO:
                    if (isSimulatorReadyForNextInstruction())
                    {   //once look-up photo is taken, move the gantry to the PCB for part placement
                        setTargetPos(pi[part_counter].x_target, pi[part_counter].y_target);
                        state = MOVE_TO_PCB;
                        printf("Time: %7.2f  New state: %.20s  Look-up photo acquired. Moving to PCB\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case MOVE_TO_PCB:
                    //once the gantry has finished moving to the PCB, then it is ready to take a look-down photo
                    if (isSimulatorReadyForNextInstruction())
                    {
                        state = LOOK_DOWN_PHOTO;
                        printf("Time: %7.2f  New state: %.20s  Now at PCB. Taking look-down photo\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case LOOK_DOWN_PHOTO:
                    //take the look-down photo, then move on to check for errors
                    takePhoto(PHOTO_LOOKDOWN);
                    state = CHECK_ERROR;
                    printf("Time: %7.2f  New state: %.20s  Look-down photo acquired. Checking for errors in alignment\n", getSimulationTime(), state_name[state]);
                    break;

                case CHECK_ERROR:
                    //wait until the look-down photo is taken, then calculate errors
                    if (isSimulatorReadyForNextInstruction())
                    {
                        double errortheta = getPickErrorTheta(CENTRE_NOZZLE);  //acquire the part misalignment from the look-up photo
                        requested_theta = pi[part_counter].theta_target - errortheta;  //calculate misalignment of the part on the nozzle
                        preplace_diff_x = pi[part_counter].x_target - (pi[part_counter].x_target+getPreplaceErrorX()); //calculate the difference between the required x position and the actual x position of the gantry
                        preplace_diff_y = pi[part_counter].y_target - (pi[part_counter].y_target+getPreplaceErrorY()); //calculate the difference between the required y position and the actual y position of the gantry
                        state = WAIT_1;  //display the errors to the user so they are aware and then wait for instruction
                        printf("Time: %7.2f             %19s  Part misalignment error: %3.2f, preplace misalignment error: x=%3.2f y=%3.2f\n", getSimulationTime()," ", errortheta, getPreplaceErrorX(), getPreplaceErrorY());
                        printf("Time: %7.2f  New state: %.20s  Waiting for next instruction. Recommend error correction\n", getSimulationTime(),state_name[state]);
                    }
                    break;

                case CORRECT_ERRORS:
                    if (isSimulatorReadyForNextInstruction())
                    {  //once the nozzle or gantry position has been corrected, go back to wait for next instruction
                        state = WAIT_1;
                        printf("Time: %7.2f  New state: %.20s  Misalignment corrected, ready for next instruction\n", getSimulationTime(), state_name[state]);
                    }
                    break;

                case MOVE_TO_HOME:
                    if (isSimulatorReadyForNextInstruction())
                    {
                        state = HOME;
                        printf("Time: %7.2f  New state: %.20s  Gantry in Home position. Provide next instruction or q to quit.\n", getSimulationTime(), state_name[state]);
                    }
                    break;

            }
            sleepMilliseconds((long) 1000 / POLL_LOOP_RATE);
        }
    }

    /* state machine code for autonomous control mode */
    else
    {
        /* initialization of variables and controller window */
        int state = HOME, part_counter = 0;
        char c, part_placed = FALSE, NozzleStatus = not_holdingpart;
        double requested_theta = 0;  //the required angle theta of the nozzle position
        double preplace_diff_x = 0, preplace_diff_y = 0;  //difference in required gantry position and actual gantry position for preplacement


        printf("Time: %7.2f  Initial state: %.15s  Operating in automatic mode, there are %d parts to place\n\n", getSimulationTime(), state_name[HOME], number_of_components_to_place);
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
                setTargetPos(TAPE_FEEDER_X[part_counter], TAPE_FEEDER_Y[part_counter]);
                state = MOVE_TO_FEEDER;
                printf("Time: %7.2f  New state: %.20s  Moving to tape feeder %c\n", getSimulationTime(), state_name[state], pi[part_counter].feeder);

                break;

            case MOVE_TO_FEEDER:
                //waiting for the simulator to complete movement of the gantry
                if (isSimulatorReadyForNextInstruction())
                {
                    lowerNozzle(CENTRE_NOZZLE);
                    state = LOWER_CNTR_NOZZLE;
                    printf("Time: %7.2f  New state: %.20s  Arrived at feeder. Lowering centre nozzle\n", getSimulationTime(), state_name[state]);
                }
                break;

            case LOWER_CNTR_NOZZLE:
                if (isSimulatorReadyForNextInstruction())
                {
                    if(NozzleStatus == not_holdingpart)
                    {   //vacuum will apply when the nozzle is empty
                        applyVacuum(CENTRE_NOZZLE);
                        state = VAC_CNTR_NOZZLE;
                        printf("Time: %7.2f  New state: %.20s  Applying vacuum\n", getSimulationTime(), state_name[state]);
                    }
                    if(NozzleStatus == holdingpart)
                    {   //vacuum will release the part when the nozzle is holding something
                        releaseVacuum(CENTRE_NOZZLE);
                        part_placed = TRUE;  //counter to indicate the part has been placed
                        state = VAC_CNTR_NOZZLE;
                        printf("Time: %7.2f  New state: %.20s  Releasing vacuum to place part\n", getSimulationTime(), state_name[state]);
                    }
                }

                break;

            case VAC_CNTR_NOZZLE:
                //wait until the vacuum action is finished before raising the nozzle
                if (isSimulatorReadyForNextInstruction())
                {
                    raiseNozzle(CENTRE_NOZZLE);
                    state = RAISE_CNTR_NOZZLE;
                    printf("Time: %7.2f  New state: %.20s  Raising nozzle\n", getSimulationTime(), state_name[state]);
                }
                break;

            case RAISE_CNTR_NOZZLE:
                //once nozzle is raised, if a part hasn't just been placed, then it is determined that a part has just been picked up
                if (isSimulatorReadyForNextInstruction())
                {
                    if (part_placed==FALSE)
                    {
                        NozzleStatus = holdingpart;
                        setTargetPos(LOOKUP_CAMERA_X,LOOKUP_CAMERA_Y);  //the gantry will move to the position above the camera
                        state = MOVE_TO_CAMERA;
                        printf("Time: %7.2f  New state: %.20s  Part acquired, moving to look-up camera\n", getSimulationTime(), state_name[state]);
                    }
                        //if the vacuum has just released a part, then the part has been placed and the nozzle is free again
                    if (part_placed==TRUE)
                    {
                        NozzleStatus = not_holdingpart;
                        part_counter++;  //increment counter to keep track of the part number in the centroid file that has been placed
                        if (part_counter != number_of_components_to_place)
                        {   //since there are still components to be placed, go back to Home to cycle again. Display the next set of part details
                            state = HOME;
                            printf("Time: %7.2f  New state: %.20s  Part %d placed on PCB successfully\n\n", getSimulationTime(), state_name[state], (part_counter-1));
                            printf("Part %d details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n", part_counter,
                            pi[part_counter].component_designation, pi[part_counter].component_footprint, pi[part_counter].component_value, pi[part_counter].x_target,
                            pi[part_counter].y_target, pi[part_counter].theta_target, pi[part_counter].feeder);
                        }
                        else if(part_counter == number_of_components_to_place)
                        {
                            setTargetPos(HOME_X,HOME_Y);
                            state = MOVE_TO_HOME;
                            printf("Time: %7.2f  New state: %.20s  All parts have been placed! Moving to home\n", getSimulationTime(), state_name[state]);
                        }
                        }
                    }
                    break;

            case MOVE_TO_CAMERA:
                //waiting for the gantry to move to the camera position before taking look-up photo
                if (isSimulatorReadyForNextInstruction())
                {
                    takePhoto(PHOTO_LOOKUP);
                    state = LOOK_UP_PHOTO;
                    printf("Time: %7.2f  New state: %.20s  Arrived at camera. Taking look-up photo of part\n", getSimulationTime(), state_name[state]);
                }
                break;

            case LOOK_UP_PHOTO:
                if (isSimulatorReadyForNextInstruction())
                {   //once look-up photo is taken, move the gantry to the PCB for part placement
                    setTargetPos(pi[part_counter].x_target, pi[part_counter].y_target);
                    state = MOVE_TO_PCB;
                    printf("Time: %7.2f  New state: %.20s  Look-up photo acquired. Moving to PCB\n", getSimulationTime(), state_name[state]);
                }
                break;

            case MOVE_TO_PCB:
                //once the gantry has finished moving to the PCB, then it is ready to take a look-down photo
                if (isSimulatorReadyForNextInstruction())
                {
                    state = LOOK_DOWN_PHOTO;
                    printf("Time: %7.2f  New state: %.20s  Now at PCB. Taking look-down photo\n", getSimulationTime(), state_name[state]);
                }
                break;

            case LOOK_DOWN_PHOTO:
                //take the look-down photo, then move on to calculate errors
                takePhoto(PHOTO_LOOKDOWN);
                state = CHECK_ERROR;
                printf("Time: %7.2f  New state: %.20s  Look-down photo acquired. Checking for errors in alignment\n", getSimulationTime(), state_name[state]);
                break;

            case CHECK_ERROR:
                //wait until the look-down photo is taken, then calculate errors
                if (isSimulatorReadyForNextInstruction())
                {
                    double errortheta = getPickErrorTheta(CENTRE_NOZZLE);  //acquire the part misalignment from the look-up photo
                    requested_theta = pi[part_counter].theta_target - errortheta;  //calculate misalignment of the part on the nozzle

                    preplace_diff_x = pi[part_counter].x_target - (pi[part_counter].x_target+getPreplaceErrorX()); //calculate the difference between the required x position and the actual x position of the gantry
                    preplace_diff_y = pi[part_counter].y_target - (pi[part_counter].y_target+getPreplaceErrorY()); //calculate the difference between the required y position and the actual y position of the gantry

                    state = FIX_NOZZLE_ERROR;  //display the errors to the user so they are aware and then move on to correction
                    printf("Time: %7.2f             %19s  Part misalignment error: %3.2f, preplace misalignment error: x=%3.2f y=%3.2f\n", getSimulationTime()," ", errortheta, getPreplaceErrorX(), getPreplaceErrorY());
                    printf("Time: %7.2f  New state: %.20s  Initiating error correction\n", getSimulationTime(),state_name[state]);
                }
                break;

            case FIX_NOZZLE_ERROR:
                if (isSimulatorReadyForNextInstruction())
                {  //apply correction to nozzle rotation for part alignment
                    rotateNozzle(CENTRE_NOZZLE, requested_theta);  //rotate the nozzle by the required calculated angle theta
                    state = FIX_PREPLACE_ERROR;
                    printf("Time: %7.2f  New state: %.20s  Correction made to nozzle for part misalignment.\n", getSimulationTime(),state_name[state]);
                }
                break;

            case FIX_PREPLACE_ERROR:
                if (isSimulatorReadyForNextInstruction())
                {
                    amendPos(preplace_diff_x, preplace_diff_y);  //fix the gantry preplace position over the PCB
                    state = LOWER_CNTR_NOZZLE;
                    printf("Time: %7.2f  New state: %.20s  Gantry preplace misalignment fixed. Ready to place part\n", getSimulationTime(),state_name[state]);
                }
                break;

            case MOVE_TO_HOME:
                if (isSimulatorReadyForNextInstruction())
                {
                    printf("Time: %7.2f  New state: %.20s  Gantry in Home position. Placement complete. Press q to quit.\n", getSimulationTime(), state_name[state]);
                }
                break;




                }
            }
        }


    pnpClose();
    return 0;
}

