
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Utilities.Drivers.MinoGamepad;
import frc.robot.Utilities.Section;
import com.revrobotics.CANSparkMax;


public class Intake extends Subsystem implements Section, Constants {

    private CANSparkMax topIntake;
    private CANSparkMax bottomIntake;


    private static Intake instance = null;

    private Intake() {
        topIntake = new CANSparkMax​(intakeTopID, intakeTopMotorType);
        bottomIntake = new CANSparkMax(intakeBottomID, intakeBottomMotorType);

        configTalons();

        closeIntake();
    }

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private void configTalons() {
        topIntake.set(0.0);
        bottomIntake.set(0.0);
        topIntake.configOpenloopRamp(0, 0);
        bottomIntake.configOpenloopRamp(0, 0);
        bottomIntake.setNeutralMode(NeutralMode.Brake);
        topIntake.setNeutralMode(NeutralMode.Brake);

    }

    @Override
    protected void initDefaultCommand() {
        System.out.println("yeah this thing sucks");
    }

    private boolean intaking = false;
    @Override
    public void teleop(MinoGamepad gamepad) {

/*
        if(gamepad.bottomTriggerPressed()) {
            intaking = false;
            topIntake.configOpenloopRamp(0, kTimeoutMs);
            bottomIntake.configOpenloopRamp(0, kTimeoutMs);
        } else if (gamepad.topTriggerPressed()){
            intaking = true;
            topIntake.configOpenloopRamp(0.25, kTimeoutMs);
            bottomIntake.configOpenloopRamp(0.25, kTimeoutMs);
            setPercentSpeed(intakeSolenoid.getValue() == DoubleSolenoid.Value.kForward ? 1 : -1); //IN
        } else {
            if (intaking) {
                setPercentSpeed(intakeSolenoid.getValue() == DoubleSolenoid.Value.kForward ? 0.05 : -0.05); //IN
            } else {
                topIntake.set(0);
                bottomIntake.set(0);
            }

        }
*/
        if (gamepad.getRawButton(BTN_X)) {
            toggleIntake();
        }

    }

//we felt a little silly with the names
    public void intakeConsume() {
        topIntake.set(0.5); //whatever makes motor take thingy
        bottomIntake.set(0.5); //make it match top intake(reverse probably if motors are backwards)
    }

    public void intakeRegurgitate() {
        topIntake.set(-0.5); //whatever makes motor release thingy
        bottomIntake.set(-0.5); //make it match bottom
    }

    public void toggleIntake() {
        
    }

    @Override
    public void reset() {
        topIntake.set(0);
        bottomIntake.set(0);
        closeIntake();
    }

    public void setPercentSpeed(double speed) {
        topIntake.set(-speed);
        bottomIntake.set(speed);
    }
}
