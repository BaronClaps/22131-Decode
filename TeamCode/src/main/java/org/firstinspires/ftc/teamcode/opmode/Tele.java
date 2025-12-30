package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.command.ButtonMapper;
import org.firstinspires.ftc.teamcode.config.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.config.command.GamepadEx;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@Config
public class Tele extends CommandOpMode {



    MultipleTelemetry multipleTelemetry;

    Robot r;
    final Alliance a;

    public boolean shoot = false, manual = false, field = true, hold = false, autoFlipping = false, manualFlip = false;
    public double intakeOn = 0, dist, speed = 1;
    public static double shootTarget = 1200;
    private final Timer upTimer = new Timer(), autoFlipTimer = new Timer();

    public Tele(Alliance alliance) {
        a = alliance;
    }

    @Override
    public void init() {
        r = new Robot(hardwareMap, a);

        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        r.g.closeGate();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed())
            r.t.resetTurret();
    }

    @Override
    public void start() {
        r.setShootTarget();
        r.periodic();
        r.t.reset();
        r.f.startTeleopDrive();

        upTimer.resetTimer();
    }

    @Override
    public void loop() {
        r.periodic();

        if (!hold)
            if (field)
                r.f.setTeleOpDrive(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x, speed * -gamepad1.right_stick_x, false, r.a == Alliance.BLUE ? Math.toRadians(180) : 0);
            else
                r.f.setTeleOpDrive(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x, speed * -gamepad1.right_stick_x, true);

        if (upTimer.getElapsedTimeSeconds() > 1 && !r.g.closed() && manualFlip)
            gamepad1.rumbleBlips(1);

        if (gamepad1.rightBumperWasPressed())
            if (intakeOn == 1)
                intakeOn = 0;
            else
                intakeOn = 1;

        if (gamepad1.dpadDownWasPressed())
            if (intakeOn == 2)
                intakeOn = 0;
            else
                intakeOn = 2;

        if (intakeOn == 1)
            r.i.spinIn();
        else if (intakeOn == 2)
            r.i.spinOut();
        else
            r.i.spinOff();

        if (shoot) {
            r.s.on();
            r.t.on();

            if (manual) {
                r.t.manual(-gamepad1.right_trigger + gamepad1.left_trigger);
                r.s.setTarget(shootTarget);
            } else {
                dist = r.getShootTarget().distanceFrom(r.f.getPose());
                boolean close = r.f.getPose().getY() > 48;
                r.s.forDistance(dist, close);
                r.t.face(r.getShootTarget(), r.f.getPose());
                r.t.automatic();
            }
        } else {
            r.s.off();
            r.t.off();
        }

        if (gamepad1.aWasPressed())
            if (manualFlip) {
                upTimer.resetTimer();
                r.g.toggle();
                autoFlipping = false;
            } else {
                autoFlipping = true;
                autoFlipTimer.resetTimer();
            }

        if (!manualFlip && autoFlipping) {
            if (autoFlipTimer.getElapsedTimeSeconds() > 1.25) {
                r.g.closeGate();
                autoFlipping = false;
            } else if (autoFlipTimer.getElapsedTimeSeconds() > .1) {
                r.i.spinIn();
                intakeOn = 1;
            }
            else if (autoFlipTimer.getElapsedTimeSeconds() > 0) {
                r.g.openGate();
                intakeOn = 0;
                r.i.spinOff();
            }
        }

        if (gamepad1.leftStickButtonWasPressed())
            manualFlip = !manualFlip;

        if (gamepad1.bWasPressed())
            shoot = !shoot;

        if (gamepad1.dpadUpWasPressed()) {
            if (r.a.equals(Alliance.BLUE)) {
                r.f.setPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
            } else {
                r.f.setPose(new Pose(8, 6.25, Math.toRadians(0)));
            }
        }

        if (gamepad1.dpadLeftWasPressed())
            manual = !manual;

        if (gamepad1.dpadRightWasPressed())
            field = !field;

        if (gamepad1.yWasPressed()) {
            hold = !hold;

            if (hold) {
                r.f.holdPoint(new BezierPoint(r.f.getPose()), r.f.getHeading(), false);
            } else {
                r.f.startTeleopDrive();
            }
        }

        if (gamepad1.xWasPressed())
            r.t.resetTurret();

        if (gamepad1.left_bumper)
            speed = 0.5;
        else
            speed = 1.0;

        multipleTelemetry.addData("Follower Pose", r.f.getPose().toString());
        multipleTelemetry.addData("Shooter Velocity", r.s.getVelocity());
        multipleTelemetry.addData("Shooter Target", r.s.getTarget());
        multipleTelemetry.addData("Shooter Distance", dist);
        multipleTelemetry.addData("Turret Yaw", r.t.getYaw());
        multipleTelemetry.addData("Turret Target", r.t.getTurretTarget());
        multipleTelemetry.addData("Turret Ticks", r.t.getTurret());
        multipleTelemetry.addData("Shooter On", shoot);
        multipleTelemetry.addData("Gate Closed", r.g.closed());
        multipleTelemetry.addData("Distance from Target", dist);
        multipleTelemetry.addData("Manual Shooter + Turret", manual);
        multipleTelemetry.addData("Field Centric", field);
        multipleTelemetry.addData("Hold Position", hold);
        multipleTelemetry.update();
    }


    @Override
    public void stop() {
        r.saveEnd();
    }
}
