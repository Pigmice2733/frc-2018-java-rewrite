package frc.robot.components;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.motion.StaticProfile;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.utils.Bounds;

public class ProfiledElevator {
    private WPI_TalonSRX winch;
    private DigitalInput bottomLimit;

    private Target target;
    private double speed, profileStartTime;
    private StaticProfile profile;
    private PIDF pidController;
    private final double gravityCompensation;

    public enum Target {
        BOTTOM(0.0), SWITCH(4.0), SCALE(12.0);

        private final double value;

        Target(double value) {
            this.value = value;
        }

        private double value() {
            return value;
        }
    }

    public ProfiledElevator(WPI_TalonSRX winch, DigitalInput bottomLimit) {
        this.winch = winch;
        this.bottomLimit = bottomLimit;
        target = Target.BOTTOM;
        speed = 0.0;
        profileStartTime = Timer.getFPGATimestamp();

        winch.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);

        gravityCompensation = 0.1;
        profile = new StaticProfile(0.0, 0.0, 0.0, 3.0, 1.0, 1.0);

        Gains pidGains = new Gains(0.75, 0.0, 0.0, 0.0, 0.0, 0.0);
        Bounds outputBounds = new Bounds(-1.0, 1.0);
        pidController = new PIDF(pidGains, outputBounds);
    }

    public double getPosition() {
        return winch.getSelectedSensorPosition(0) / -4096;
    }

    private double getVelocity() {
        return winch.getSelectedSensorVelocity(0) / -4096;
    }

    public void setTarget(Target newTarget) {
        if (target == newTarget) {
            return;
        }
        target = newTarget;
        profile = new StaticProfile(getVelocity(), getPosition(), target.value(), 3.0, 1.0, 1.0);
        profileStartTime = Timer.getFPGATimestamp();
        pidController.initialize(getVelocity(), profileStartTime, speed);
    }

    public void update() {
        final double currentTarget = profile.getPosition(Timer.getFPGATimestamp() - profileStartTime);
        final double targetVelocity = profile.getVelocity(Timer.getFPGATimestamp() - profileStartTime);
        speed = pidController.calculateOutput(getPosition(), currentTarget, targetVelocity, 0.0,
                Timer.getFPGATimestamp());

        winch.set(speed + gravityCompensation);

        if (bottomLimit.get()) {
            winch.setSelectedSensorPosition(0, 0, 0);
        }
    }
}