package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.constants.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {

    public CANSparkMax rollerMotor;
    public CANSparkMax pivotMotor;

    public IntakeSubsystem() {
        rollerMotor = new CANSparkMax(Constants.IntakeConstants.rollerMotorID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(Constants.IntakeConstants.pivotMotorID, MotorType.kBrushless); //ints are just there as placeholders till we get actual ones, same with ids
    }

//we felt a little silly with the names
    public void intakeConsume() {
        rollerMotor.set(0.5); //whatever makes motor take thingy
    }

    public void intakeRegurgitate() {
        rollerMotor.set(-0.5); //whatever makes motor release thingy
    }

    public void intakeForward() {
        pivotMotor.set(0.5); //placeholder values for rn
    }

    public void intakeBackward() {
        pivotMotor.set(-0.5); //same comment as intakeForward
    }
  
    public void configureRollerMotor() {

    }

    public void configurePivotMotor() {

    }

    public void reset() {
        rollerMotor.set(0);
    }

    @Override
    public void periodic() {

    }
}
