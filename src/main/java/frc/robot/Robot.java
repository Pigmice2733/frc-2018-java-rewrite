package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.intake.Intake;
import frc.robot.auto.Forward;

public class Robot extends TimedRobot {
    private Drivetrain drivetrain;
    private Joystick driverJoystick;
    private Joystick operatorJoystick;
    private JoystickButton xboxB;
    private JoystickButton xboxX;
    private Intake intake;
    private SendableChooser<AutoMode> autoChooser;
    private Forward autoForward;

    private enum AutoMode {
        NONE, FORWARD
    }

    public void robotInit() {
        WPI_TalonSRX leftDrive = new WPI_TalonSRX(0);
        WPI_TalonSRX rightDrive = new WPI_TalonSRX(2);

        WPI_TalonSRX leftFollower = new WPI_TalonSRX(1);
        leftFollower.set(ControlMode.Follower, leftDrive.getDeviceID());

        WPI_TalonSRX rightFollower = new WPI_TalonSRX(3);
        rightFollower.set(ControlMode.Follower, rightDrive.getDeviceID());

        WPI_VictorSPX rightIntakeMotor = new WPI_VictorSPX(4);
        WPI_VictorSPX leftIntakeMotor = new WPI_VictorSPX(5);

        DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, 3);

        drivetrain = new Drivetrain(leftDrive, rightDrive);
        intake = new Intake(leftIntakeMotor, rightIntakeMotor, intakeSolenoid);
        driverJoystick = new Joystick(0);
        operatorJoystick = new Joystick(1);
        xboxB = new JoystickButton(operatorJoystick, 2);
        xboxX = new JoystickButton(operatorJoystick, 3);

        autoChooser = new SendableChooser<AutoMode>();
        autoChooser.addDefault("Drive Forward", AutoMode.FORWARD);
        autoChooser.addObject("None", AutoMode.NONE);
        SmartDashboard.putData("Autonomous mode chooser", autoChooser);
        autoForward = new Forward(drivetrain);

        setPeriod(0.02);
    }

    public void teleopPeriodic() {
        if (xboxB.get()) {
            intake.intake();
        } else if (xboxX.get()) {
            intake.outtake();
        }

        this.intake.update();
        drivetrain.arcadeDrive(-driverJoystick.getY(), driverJoystick.getX());
    }

    public void autonomousPeriodic() {
        AutoMode autoMode = autoChooser.getSelected();
        if (autoMode == AutoMode.NONE) {
            return;
        }

        autoForward.update();
    }
}
