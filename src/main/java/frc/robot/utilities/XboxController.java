package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utilities.constants.Constants;

public class XboxController {
    private static final CommandXboxController DriverController = new CommandXboxController(Constants.DriverConstants.driverControllerPort);
    private static final CommandXboxController OperatorController = new CommandXboxController(Constants.DriverConstants.operatorControllerPort);

    public static CommandXboxController getDriverController() {
        return DriverController;        
    }

    public static CommandXboxController getOperatorController() {
        return OperatorController;
    }
}
