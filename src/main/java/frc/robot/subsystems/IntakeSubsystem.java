package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {

    public CANSparkMax rollerMotor;
    public CANSparkMax pivotMotor;

    public IntakeSubsystem() {
        rollerMotor = new CANSparkMax(Constants.IntakeConstants.rollerMotorID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(Constants.IntakeConstants.pivotMotorID, MotorType.kBrushless);
    }


    public void configureRollerMotor() {

    }

    public void configurePivotMotor() {

    }

    @Override
    public void periodic() {

    }
}
