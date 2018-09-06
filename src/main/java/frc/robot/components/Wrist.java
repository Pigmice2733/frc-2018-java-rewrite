package frc.robot.components;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.utils.Bounds;

import edu.wpi.first.wpilibj.Timer;

public class Wrist {
    private WPI_TalonSRX motor;
    private PIDF pidf;
    private Target target;

    public enum Target {
        DOWN(2), UP(1400), START(2860);

        private final int value;

        Target(int value) {
            this.value = value;
        }

        private int value() {
            return value;
        }
    }

    public Wrist(WPI_TalonSRX motor) {
        this.motor = motor;

        this.motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
        this.motor.setSelectedSensorPosition(Target.DOWN.value(), 0, 0);

        target = Target.START;

        pidf = new PIDF(new Gains(0.1, 0, 0), new Bounds(-1, 1));
    }

    public void resetTop() {
        motor.setSelectedSensorPosition(Target.START.value(), 0, 0);
    }

    private double getPosition() {
        return motor.getSelectedSensorPosition(0);
    }

    public void setTarget(Target target) {
        this.target = target;
    }

    public void update() {
        double speed = pidf.calculateOutput(getPosition(), target.value(), Timer.getFPGATimestamp());
        motor.set(speed);
    }
}
