/*----------------------------------------------------------------------------*/
/* Class contains all constants used in order to maintain cleanliness of code */
/* In order to call a value use "RobotMap.NAME_OF_VALUE" without quotes       */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotMap {

    //Motor ID Constants
    public static final int LEFT_ID = 0;
    public static final int RIGHT_ID = 3;

    public static final int FLYWHEEL_MOTOR_ID = 5;
    public static final int INDEX_MOTOR_ID = 7;

    //Speed Constant
    public static final double DRIVE_SPEED_ID = 1.0;
    public static final double INDEX_SPEED_ID = 0.8;
    public static final double FLYWHEEL_SPEED_ID = 0.9;
    public static final double FLYWHEEL_SLOW_SPEED_ID = 0.5;

    //Joystick Constants
    public static final int JOYSTICK_DRIVER_PORT_ID = 0;
    public static final int JOYSTICK_OPERATOR_PORT_ID =1;
    public static final int LEFT_AXIS_ID = 1;
    public static final int RIGHT_AXIS_ID = 0;
    public static final int INDEX_BUTTON_ID = 5;
    public static final int FLYWHEEL_BUTTON_ID = 6;
    public static final int FLYWHEEL_SLOW_BUTTON_ID = 7;

    //Sensor Values
    public static final double GYRO_SETPOINT = 0.0;
    public static final double GYRO_TURNING_CONSTANT = 0.005;
}