package frc.robot.components;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.Bounds;

public class ManualElevator {
    private WPI_TalonSRX winch;
    private DigitalInput bottomLimit;

    private Bounds speedBounds;

    private double speed;
    private final double gravityCompensation;

    public ManualElevator(WPI_TalonSRX winch, DigitalInput bottomLimit) {
        this.winch = winch;
        this.bottomLimit = bottomLimit;
        this.speed = 0;
        winch.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);

        speedBounds = new Bounds(-0.3, 0.6);
        gravityCompensation = 0.4;
    }

    public void setSpeed(double speed) {
        this.speed = speedBounds.clamp(speed) + gravityCompensation;
    }

    public double getPosition() {
        return winch.getSelectedSensorPosition(0) / -4096;
    }

    public void update() {
        if (getPosition() > 12.0 && speed > 0.0) {
            speed = gravityCompensation;
        } else if (getPosition() < 1.0 && speed < 0.0) {
            speed = gravityCompensation;
        }

        winch.set(-speed);

        if (bottomLimit.get()) {
            winch.setSelectedSensorPosition(0, 0, 0);
        }

        speed = 0;
    }
}