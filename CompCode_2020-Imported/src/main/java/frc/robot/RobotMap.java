/*----------------------------------------------------------------------------*/
/* Class contains all constants used in order to maintain cleanliness of code */
/* In order to call a value use "RobotMap.NAME_OF_VALUE" without quotes       */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotMap {

    //Right
    public static final int R1 = 20;
    public static final int R2 = 23;
    public static final int R3 = 25;

    //Left
    public static final int L1 = 22;
    public static final int L2 = 21;
    public static final int L3 = 24;

    //Feeder Motors
    public static final int AGIT_MOTOR_ID = 7;
    //^switched from 5
    public static final int INDEX_MOTOR_ID = 2;
    public static final int FLYWHEEL_MOTOR_ID = 7;

    //shooter motor
    public static final int SHOOTER_MOTOR_ID = 46;

    //Floor Loading Motors
    public static final int ARM_MOTOR_ID = 10;
    public static final int INTAKE_MOTOR_ID = 9;
    //^from 9 and from 3 switched back to 9 9/16/21
    public static final int UPTAKE_MOTOR_ID = 6;

    //Climber Motors
    public static final int CLIMBER_MOTOR_L = 3; //not climber, agitator
    //^from 3 then 9 switched back to 3 9/16
    public static final int CLIMBER_MOTOR_R = 4; //winch thing 2 that was four
    public static final int CLIMBER_MOTOR_M = 8; //hook  thing that's now spinny thing
    //Warren #1, go to Phoenix Tuner and click on each motor, then blink each until you see the motor controller's lights blinking
    //Look at the motor control ID
    //input into value below
    public static final int SPINNY_MOTOR_ID = 8; //same as climbermotorm, but repeated for convenience; disregard

    //Driver Controls
    public static final int DRIVER_PORT = 0;
    public static final int DRIVER_RIGHT_AXIS = 2;
    public static final int DRIVER_LEFT_AXIS = 1;
    public static final int DRIVER_OUTAKE_DWTAKE_BUTTON = 6;
    public static final int DRIVER_INTAKE_UPTAKE_BUTTON = 5;
    public static final int DRIVER_SLOW_BUTTON = 2;
    public static final int DRIVER_FAST_BUTTON = 8;
    public static final int DRIVER_INDEXER_AGIT_BUTTON = 4;
    //^switched from 12
    public static final int DRIVER_SHOOT_BUTTON = 2;

    
    //Operator Controls
    public static final int OPERATOR_PORT = 1;
    public static final int OPERATOR_ARM_AXIS=1;
    public static final int OPERATOR_HOOK_AXIS=3;
    public static final int OPERATOR_INDEXER_AGIT_BUTTON = 5;
    //^switched from 5, given to driver
    public static final int OPERATOR_AGIT_BUTTON = 4;
    //^switched from 7
    //5 and 7 switched
    
    public static final int OPERATOR_FLYWHEEL_BUTTON = 6;
    
    //
    public static final int OPERATOR_FLYWHEEL_SLow_BUTTON = 5;
    public static final int OPERATOR_CLIMBER_UP_BUTTON = 4;
    public static final int OPERATOR_CLIMBER_DOWN_BUTTON = 2;
    public static final int OPERATOR_ARM_DOWN_BUTTON = 7;
    public static final int OPERATOR_ARM_UP_BUTTON = 8;
    public static final int OPERATOR_INDEXER_JOYSTICK = 3;
    //

    //Speed/Misc Variables
    public static final double DRIVE_SPEED = 0.8;
    public static final double DRIVE_FAST_SPEED = 1.0;
    public static final double DRIVE_SLOW_SPEED = 0.1;
    public static final double INDEX_SPEED = 0.8;
    public static final double FLYWHEEL_SPEED = -0.9;
    public static final double ARM_SPEED = 0.3;
    public static final double INTAKE_UPTAKE_SPEED = 0.7;
    public static final double INDEX_AGIT_SPEED = -0.7;
    public static final double AGIT_SPEED = 0.7;
    public static final double CLIMB_SPEED = 0.6;
    //Warren--to change shooting speed, change the value below
    public static final double SHOOT_SPEED = 0.5;

    public static final double RAMP_VAL = 0.1;
    public static final int LIMIT_SWITCH_D_PORT = 0;
    public static final int LIMIT_SWITCH_U_PORT = 1;
}