/*----------------------------------------------------------------------------*/
/* Class contains all constants used in order to maintain cleanliness of code */
/* In order to call a value use "RobotMap.NAME_OF_VALUE" without quotes       */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotMap {

    //Right
    public static final int R1 = 22;
    public static final int R2 = 23;
    public static final int R3 = 25;

    //Left
    public static final int L1 = 20;
    public static final int L2 = 21;
    public static final int L3 = 25;

    //Feeder Motors
    public static final int INDEX_AGIT_MOTOR_ID = -1;
    public static final int FLYWHEEL_MOTOR_ID = -1;

    //Floor Loading Motors
    public static final int ARM_MOTOR_ID = -1;
    public static final int INTAKE_MOTOR_ID = -1;
    public static final int UPTAKE_MOTOR_ID = -1;

    //Climber Motors
    public static final int CLIMBER_MOTOR_L = -1;
    public static final int CLIMBER_MOTOR_R = -1;

    //Driver Controls
    public static final int DRIVER_PORT = 0;
    public static final int DRIVER_RIGHT_AXIS = 3;
    public static final int DRIVER_LEFT_AXIS = 1;
    public static final int DRIVER_ARM_BUTTON = 3;
    public static final int DRIVER_INTAKE_UPTAKE_BUTTON = 6;
    public static final int DRIVER_SLOW_BUTTON_1 = 7;
    public static final int DRIVER_SLOW_BUTTON_2 = 8;
    
    //Operator Controls
    public static final int OPERATOR_PORT = 1;
    public static final int OPERATOR_INDEXER_AGIT_BUTTON = 6;
    public static final int OPERATOR_FLYWHEEL_BUTTON = 5;
    public static final int OPERATOR_CLIMBER_UP_BUTTON = 8;
    public static final int OPERATOR_CLIMBER_DOWN_BUTTON = 7;

    //Speed Variables
    public static final double DRIVE_SPEED = 1.0;
    public static final double DRIVE_SLOW_SPEED = 0.8;
    public static final double DRIVE_SLOW_SPEED = 0.4;
    public static final double INDEX_SPEED = 0.8;
    public static final double FLYWHEEL_SPEED = 0.9;
    public static final double ARM_SPEED = 0.7;
    public static final double INTAKE_UPTAKE_SPEED = 0.7;
    public static final double INDEX_AGIT_SPEED = 0.7;
    public static final double CLIMB_SPEED = 0.6;
}