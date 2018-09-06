package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.components.Drivetrain;
import frc.robot.components.ProfiledElevator;
import frc.robot.components.Intake;

public class CenterSwitch {
    private Drivetrain drivetrain;
    private Drivetrain.ProfileTask task;

    private enum State {
        FORWARD, TURN, TO_SWITCH, EJECT, REVERSE, DONE
    };

    private State state;
    private ProfiledElevator elevator;
    private Intake intake;
    private double stateStartTime;

    public CenterSwitch(Drivetrain drivetrain, ProfiledElevator elevator, Intake intake) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        this.state = State.FORWARD;

        stateStartTime = 0.0;

        task = drivetrain.forwardTask(1.5);
    }

    public void update() {
        if (state == State.FORWARD) {
            if (task.update()) {
                elevator.setTarget(ProfiledElevator.Target.SWITCH);
                state = State.TURN;
                task = drivetrain.rotateTask(40);
            }
        } else if (state == State.TURN) {
            if (task.update()) {
                state = State.TO_SWITCH;
                task = drivetrain.forwardTask(1.2);
            }
        } else if (state == State.TO_SWITCH) {
            if (task.update()) {
                state = State.EJECT;
                stateStartTime = Timer.getFPGATimestamp();
            }
        } else if (state == State.EJECT) {
            intake.outtake();
            double stateLength = 1.0;
            if (Timer.getFPGATimestamp() - stateStartTime > stateLength) {
                state = State.REVERSE;
                task = drivetrain.forwardTask(-0.8);
            }
        } else if (state == State.REVERSE) {
            if (task.update()) {
                state = State.DONE;
            }
        } else {
            drivetrain.arcadeDrive(0.0, 0.0);
        }
    }
}
