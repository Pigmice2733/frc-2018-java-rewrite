package frc.robot.components;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;

public class Intake {
    private SpeedController leftMotor;
    private SpeedController rightMotor;
    private DoubleSolenoid solenoid;

    private enum Mode {
        INTAKE, IDLE, OUTTAKE
    };

    private Mode mode;

    public Intake(SpeedController leftMotor, SpeedController rightMotor, DoubleSolenoid solenoid) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.solenoid = solenoid;
    }

    public void update() {
        double speed = 0;
        if (mode == Mode.INTAKE) {
            speed = 0.7;
            solenoid.set(DoubleSolenoid.Value.kForward);
        } else if (mode == Mode.OUTTAKE) {
            speed = -0.6;
            solenoid.set(DoubleSolenoid.Value.kReverse);
        } else {
            speed = 0.2;
            solenoid.set(DoubleSolenoid.Value.kReverse);
        }

        this.leftMotor.set(speed);
        this.rightMotor.set(-speed);

        mode = Mode.IDLE;
    }

    public void intake() {
        this.mode = Mode.INTAKE;
    }

    public void outtake() {
        this.mode = Mode.OUTTAKE;
    }
}
