package org.firstinspires.ftc.teamcode.config.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

public class Drivetrain {
    private Follower f;
    private final Alliance a;
    private boolean hold = false, field = true;
    public Drivetrain(HardwareMap hardwareMap, Alliance a, Pose start) {
        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(start);
        this.a = a;
    }

    public void startDrive() {
        f.startTeleopDrive();
    }

    public CommandBuilder start() { return Commands.instant(this::startDrive); }

    public void resetDrive() {
        if (a.equals(Alliance.BLUE)) {
            f.setPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
        } else {
            f.setPose(new Pose(8, 6.25, Math.toRadians(0)));
        }
    }

    public CommandBuilder reset() { return Commands.instant(this::resetDrive); }

    public void periodic() {
        f.update();
    }

    public void drive(Gamepad g) {
        if (!hold)
            if (field)
                f.setTeleOpDrive(-g.left_stick_y, -g.left_stick_x, -g.right_stick_x, false, a == Alliance.BLUE ? Math.toRadians(180) : 0);
            else
                f.setTeleOpDrive(-g.left_stick_y, -g.left_stick_x, -g.right_stick_x, true);
    }

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

    public CommandBuilder toggleCentric() {
        return Commands.instant(this::teleToggleCentric);
    }

    public CommandBuilder hold() {
        return Commands.instant(this::holdCurrent);
    }

    public CommandBuilder release() {
        return Commands.instant(this::releaseHold);
    }

    public CommandBuilder corner() {
        return Commands.instant(this::cornerReset);
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
