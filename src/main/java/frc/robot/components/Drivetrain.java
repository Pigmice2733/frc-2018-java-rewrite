package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.motion.StaticProfile;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.utils.Bounds;

public class Drivetrain {
    private enum ProfileTaskType {
        TURN, DRIVE
    }

    public class ProfileTask {
        private StaticProfile profile;
        private PIDF pidController;
        private double profileStartTime;
        private ProfileTaskType type;
        private double startValue, targetEndValue;

        private ProfileTask(StaticProfile profile, PIDF pidController, double profileStartTime, ProfileTaskType type) {
            this.profile = profile;
            this.pidController = pidController;
            this.type = type;
            this.profileStartTime = profileStartTime;

            targetEndValue = this.profile.getPosition(this.profile.getDuration());

            if (type == ProfileTaskType.DRIVE) {
                startValue = getTotalDistance();
            } else {
                startValue = getOrientation();
            }
        }

        public boolean update() {
            double time = Timer.getFPGATimestamp();
            final double elapsedTime = time - profileStartTime;
            final double currentTarget = profile.getPosition(elapsedTime);
            if (type == ProfileTaskType.DRIVE) {
                double targetVelocity = profile.getVelocity(elapsedTime);
                double speed = pidController.calculateOutput(getTotalDistance() - startValue, currentTarget,
                        targetVelocity, 0.0, time);
                arcadeDrive(speed, 0.0);
            } else {
                double rotation = pidController.calculateOutput(getOrientation() - startValue, currentTarget, 0.0, 0.0,
                        time);
                arcadeDrive(0.0, rotation);
            }
            return (elapsedTime > profile.getDuration() && (targetEndValue - currentTarget) < (targetEndValue * 0.01));
        }
    }

    private WPI_TalonSRX leftDrive, rightDrive;
    private DifferentialDrive drive;
    private AHRS navx;
    private double leftWheelDistance, rightWheelDistance, totalDistance;

    public Drivetrain(WPI_TalonSRX leftDrive, WPI_TalonSRX rightDrive, AHRS navx) {
        drive = new DifferentialDrive(leftDrive, rightDrive);
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.navx = navx;
        leftWheelDistance = 0.0;
        rightWheelDistance = 0.0;
        totalDistance = 0.0;

        setOrientation(0.0);
    }

    public void arcadeDrive(double forward, double rotation) {
        drive.arcadeDrive(forward, rotation);
    }

    public ProfileTask forwardTask(double meters) {
        double currentDistance = getTotalDistance();
        StaticProfile profile = new StaticProfile(getVelocity(), currentDistance, currentDistance + meters, 2.2, 2.0,
                1.0);
        Gains gains = new Gains(1.5, 0.2, 0.0);
        Bounds outputBounds = new Bounds(-1.0, 1.0);
        PIDF pidController = new PIDF(gains, outputBounds);
        double time = Timer.getFPGATimestamp();
        return new ProfileTask(profile, pidController, time, ProfileTaskType.DRIVE);
    }

    public ProfileTask rotateTask(double degrees) {
        double radians = Math.toRadians(degrees);
        double currentAngle = getOrientation();
        StaticProfile profile = new StaticProfile(getVelocity(), currentAngle, currentAngle + radians, 1.5, 0.75, 1.0);
        Gains gains = new Gains(0.85, 0.1, 0.1);
        Bounds outputBounds = new Bounds(-1.0, 1.0);
        PIDF pidController = new PIDF(gains, outputBounds);
        double time = Timer.getFPGATimestamp();
        return new ProfileTask(profile, pidController, time, ProfileTaskType.TURN);
    }

    public double getOrientation() {
        return Math.toRadians(-navx.getAngle());
    }

    public double getVelocity() {
        double encoderScaling = 4096 / (6 * Math.PI * 0.02540);
        double encoderVelocity = (rightDrive.getSelectedSensorVelocity(0) - leftDrive.getSelectedSensorVelocity(0))
                / 2.0;

        return 10 * encoderVelocity / encoderScaling;
    }

    public double getTotalDistance() {
        double encoderScaling = 4096 / (6 * Math.PI * 0.02540);

        double newLeftDistance = -leftDrive.getSelectedSensorPosition(0) / encoderScaling;
        double newRightDistance = rightDrive.getSelectedSensorPosition(0) / encoderScaling;

        double deltaLeftDistance = newLeftDistance - leftWheelDistance;
        double deltaRightDistance = newRightDistance - rightWheelDistance;

        totalDistance += (deltaLeftDistance + deltaRightDistance) / 2.0;

        leftWheelDistance = newLeftDistance;
        rightWheelDistance = newRightDistance;
        return totalDistance;
    }

    private void setOrientation(double orientation) {
        navx.setAngleAdjustment(0.0);
        navx.setAngleAdjustment(-navx.getAngle() - Math.toDegrees(orientation));
    }
}
