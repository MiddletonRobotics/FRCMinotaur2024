package frc.robot.utilities;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.utilities.constants.Constants;

public class Controller {
    private static final Joystick DriverController = new Joystick(Constants.DriverConstants.driverControllerPort);
    private static final Joystick OperatorController = new Joystick(Constants.DriverConstants.operatorControllerPort);

    public static Joystick getDriverController() {
        return DriverController;
    }

    public static Joystick getOperatorController() {
        return OperatorController;
    }

    public static boolean a(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.BTN_A);
    }

    public static int a() {
        return Constants.ControllerRawButtons.BTN_A;
    }

    public static boolean b(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.BTN_B);
    }

    public static int b() {
        return Constants.ControllerRawButtons.BTN_B;
    }

    public static boolean x(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.BTN_X);
    }

    public static int x() {
        return Constants.ControllerRawButtons.BTN_X;
    }

    public static boolean y(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.BTN_Y);
    }

    public static int y() {
        return Constants.ControllerRawButtons.BTN_Y;
    }

    public static boolean leftBumper(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.BTN_LB);
    }

    public static int leftBumper() {
        return Constants.ControllerRawButtons.BTN_LB;
    }

    public static boolean rightBumper(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.BTN_RB);
    }

    public static int rightBumper() {
        return Constants.ControllerRawButtons.BTN_RB;
    }

    public static double leftY(Joystick Controller) {
        return Controller.getRawAxis(Constants.ControllerRawButtons.LEFT_Y_AXIS);
    }

    public static double leftX(Joystick Controller) {
        return Controller.getRawAxis(Constants.ControllerRawButtons.LEFT_X_AXIS);
    }

    public static double rightY(Joystick Controller) {
        return Controller.getRawAxis(Constants.ControllerRawButtons.RIGHT_Y_AXIS);
    }

    public static double rightX(Joystick Controller) {
        return Controller.getRawAxis(Constants.ControllerRawButtons.RIGHT_X_AXIS);
    }
}
