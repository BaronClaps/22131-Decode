package org.firstinspires.ftc.teamcode.config.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Instant;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.command.GamepadEx;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

public class Drivetrain {
    private Follower f;
    private final Alliance a;
    private boolean hold = false, field = true;
    private Gamepad g;
    public Drivetrain(HardwareMap hardwareMap, Alliance a, Pose start) {
        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(start);
        this.a = a;
    }

    public void startDrive() {
        f.startTeleopDrive();
    }

    public Instant start() { return new Instant(this::startDrive); }

    public void resetDrive() {
        if (a.equals(Alliance.BLUE)) {
            f.setPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
        } else {
            f.setPose(new Pose(8, 6.25, Math.toRadians(0)));
        }
    }

    public Instant reset() { return new Instant(this::resetDrive); }

    public void periodic() {
        if (g != null)
            if (!hold)
                if (field)
                    f.setTeleOpDrive(-g.left_stick_y, -g.left_stick_x, -g.right_stick_x, false, a == Alliance.BLUE ? Math.toRadians(180) : 0);
                else
                    f.setTeleOpDrive(-g.left_stick_y, -g.left_stick_x, -g.right_stick_x, true);

        f.update();
    }

    public void setGamepad(Gamepad g) { this.g = g; }

    public void holdCurrent() {
        f.holdPoint(new BezierPoint(f.getPose()), f.getHeading(), true);
        hold = true;
    }

    public void releaseHold() {
        hold = false;
    }

    public void teleToggleCentric() {
        field = !field;
    }

    public void cornerReset() {
        if (a.equals(Alliance.BLUE))
            f.setPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
        else
            f.setPose(new Pose(8, 6.25, Math.toRadians(0)));
    }

    public Instant toggleCentric() {
        return new Instant(this::teleToggleCentric);
    }

    public Instant hold() {
        return new Instant(this::holdCurrent);
    }

    public Instant release() {
        return new Instant(this::releaseHold);
    }

    public Instant corner() {
        return new Instant(this::cornerReset);
    }

    public void setStart(Pose start) {
        f.setStartingPose(start);
    }

    public Pose getPose() {
        return f.getPose();
    }

    public Pose isBusy() {
        return f.getPose();
    }

    public double getT() {
        return f.getCurrentTValue();
    }

    public Follower getFollower() { return f;}
}
