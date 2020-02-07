/*----------------------------------------------------------------------------*/
/* Class contains all constants used in order to maintain cleanliness of code */
/* In order to call a value use "RobotMap.NAME_OF_VALUE" without quotes       */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotMap {

    //Motor ID Constants
    //Right
    public static final int R1 = 22;
    public static final int R2 = 23;
    public static final int R3 = 25;

    //Left
    public static final int L1 = 20;
    public static final int L2 = 21;
    public static final int L3 = 25;

    //Feeder Motors
    public static final int INDEX_ID = 7;
    public static final int FLYWHEEL_ID = 5;

    //Joystick Variables
    public static final int JOYSTICK_DRIVER_PORT = 0;
    public static final int JOYSTICK_OPERATOR_PORT = 1;

    //Speed Variables
    public static final double DRIVE_SPEED = 0.8;
    public static final double INDEX_SPEED = 0.8;
    public static final double FLYWHEEL_SPEED = 0.9;
}