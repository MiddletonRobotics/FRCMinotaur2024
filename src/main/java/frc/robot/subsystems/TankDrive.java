package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.XboxController;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class TankDrive extends SubsystemBase {

    public TalonFX[] rightMotors;
    public TalonFX[] leftMotors;

    public DifferentialDrive TankDrive;

    private CommandXboxController DriverController;
    private CommandXboxController OperatorController;

    public TankDrive() {
        rightMotors = new TalonFX[Constants.rightMotorCount];
        rightMotors[0] = new TalonFX(Constants.leftMasterID);
        rightMotors[1] = new TalonFX(Constants.rightSlaveID);

        leftMotors = new TalonFX[Constants.leftMotorCount];
        leftMotors[0] = new TalonFX(Constants.leftMasterID);
        leftMotors[1] = new TalonFX(Constants.leftSlaveID);

        for (int i = 0; i < 2; i++) {
            leftMotors[i].setInverted(false);
            leftMotors[i].setNeutralMode(NeutralModeValue.Brake);
            
            rightMotors[i].setInverted(false);
            leftMotors[i].setNeutralMode(NeutralModeValue.Brake);

            if(i != 0) {
                leftMotors[1].setControl(new Follower(leftMotors[0].getDeviceID(), false));
                rightMotors[1].setControl(new Follower(rightMotors[0].getDeviceID(), false));
            }
        }

        TankDrive = new DifferentialDrive(leftMotors[0], rightMotors[0]);
        TankDrive.setSafetyEnabled(false);
    }

    public void setRightInverted() {
        for(int i = 0; i < Constants.rightMotorCount; i++) {
            rightMotors[i].setInverted(true);
        }
    }

    public void setLeftInverted() {
        for(int i = 0; i < Constants.leftMotorCount; i++) {
            leftMotors[i].setInverted(true);
        }
    }

    public void setIdleMode(String mode) {
        if(mode != "Brake" || mode != "Coast") {
            System.out.println("Invalid Neutral Mode");
            return;
        } else {
            for (int i = 0; i < 2; i++) {
                if(mode == "Brake") {
                    leftMotors[i].setNeutralMode(NeutralModeValue.Brake);
                    rightMotors[i].setNeutralMode(NeutralModeValue.Brake);
                } else if(mode == "Coast") {
                    leftMotors[i].setNeutralMode(NeutralModeValue.Coast);
                    rightMotors[i].setNeutralMode(NeutralModeValue.Coast);
                }
            }
        }
    }

    public Command driveCommand(DoubleSupplier x, DoubleSupplier y) {
        DriverController = XboxController.getDriverController();

        // x = Math.abs(x.getAsDouble()) > Constants.kDeadband ? x : 0.0;
        // y = Math.abs(y.getAsDouble()) > Constants.kDeadband ? y : 0.0;

        return run(() -> TankDrive.arcadeDrive(x.getAsDouble(), y.getAsDouble())).withName("drive");
    }
    
}
