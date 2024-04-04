// Import information on controls (joystick, buttons) to set up each operation.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utilities.constants.Constants;

public class Controller {
    public static final CommandXboxController DriverController = new CommandXboxController(Constants.DriverConstants.driverControllerPort);
    public static final CommandXboxController OperatorController = new CommandXboxController(Constants.DriverConstants.operatorControllerPort);

    public static CommandXboxController getDriverController() {
        return DriverController;
    }

    public static CommandXboxController getOperatorController() {
        return OperatorController;
    }

    public Command resetRumble() {
        return new InstantCommand(() -> DriverController.getHID().setRumble(RumbleType.kBothRumble, 0))
            .andThen(new InstantCommand(() -> OperatorController.getHID().setRumble(RumbleType.kBothRumble, 0)));
    }
}
