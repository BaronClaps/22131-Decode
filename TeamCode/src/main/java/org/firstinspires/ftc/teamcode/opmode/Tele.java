package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@TeleOp
@Config
public class Tele extends OpMode {
    TelemetryManager telemetryM;
    MultipleTelemetry multipleTelemetry;

    Robot r;

    public boolean shoot = false, manual = false, field = true, hold = false;
    public static double intakeOn = 0, dist;
    private final Timer upTimer = new Timer();

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        r.s.down();
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) {
            r.a = Alliance.BLUE;
        }

        if (gamepad1.bWasPressed()) {
            r.a = Alliance.RED;
        }

        if (gamepad1.xWasPressed())
            r.t.resetTurret();

        telemetryM.addData("Alliance", r.a);
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        r.setShootTarget();

        if (Robot.endPose == null) {
            r.f.setStartingPose(r.a.equals(Alliance.BLUE) ? Robot.defaultPose : Robot.defaultPose.mirror());
        } else {
            r.f.setStartingPose(Robot.endPose);
        }

        r.periodic();
        r.t.reset();
        r.f.startTeleopDrive();

        upTimer.resetTimer();
    }

    @Override
    public void loop() {
        r.periodic();

        if (hold)
            r.f.holdPoint(new BezierPoint(r.f.getPose()), r.f.getHeading(), false);
        else
            r.f.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, shoot ? -gamepad1.right_stick_x * 0.5 : -gamepad1.right_stick_x * 0.75, !field, r.a == Alliance.BLUE ? Math.toRadians(180) : 0);

        if (upTimer.getElapsedTimeSeconds() > 2 && r.s.atUp())
            gamepad1.rumbleBlips(1);

        if (gamepad1.rightBumperWasPressed())
            if (intakeOn == 1)
                intakeOn = 0;
            else
                intakeOn = 1;

        if (gamepad1.leftBumperWasPressed())
            if (intakeOn == 2)
                intakeOn = 0;
            else
                intakeOn = 2;

        if (intakeOn == 1)
            r.i.spinIn();
        else if (intakeOn == 2)
            r.i.spinOut();
        else
            r.i.spinIdle();

        if (shoot) {
            r.s.on();
            r.t.on();

            if (manual) {
                r.t.manual(gamepad1.right_trigger - gamepad1.left_trigger);
                r.s.close();
            } else {
                dist = r.getShootTarget().distanceFrom(r.f.getPose());
                r.s.forDistance(dist);
                r.t.face(r.getShootTarget(), r.f.getPose());
                r.t.automatic();
            }
        } else {
            r.s.off();
            r.t.off();
        }

        if (gamepad1.bWasPressed())
            shoot = !shoot;

        if (gamepad1.aWasPressed()) {
            upTimer.resetTimer();
            r.s.flip();
        }

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

        if (gamepad1.dpadDownWasPressed())
            hold = !hold;

        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("Follower Pose: " + r.f.getPose().toString());
        packet.addLine("Shooter Velocity: " + r.s.getVelocity());
        packet.addLine("Shooter Target: " + r.s.getTarget());
        packet.addLine("Shooter Distance: " + dist);
        packet.addLine("Turret Yaw: " + r.t.getYaw());
        packet.addLine("Turret Target: " + r.t.getTurretTarget());
        packet.addLine("Turret Ticks: " + r.t.getTurret());
        packet.addLine("Shooter On: " + shoot);
        packet.addLine("Flipped Up: " + r.s.atUp());
        packet.addLine("Distance from Target: " + dist);
        packet.addLine("Manual Shooter + Turret: " + manual);
        packet.addLine("Field Centric: " + field);
        packet.addLine("Hold Position: " + hold);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetryM.addData("Follower Pose", r.f.getPose().toString());
        telemetryM.addData("Shooter Velocity", r.s.getVelocity());
        telemetryM.addData("Shooter Target", r.s.getTarget());
        telemetryM.addData("Shooter Distance", dist);
        telemetryM.addData("Turret Yaw", r.t.getYaw());
        telemetryM.addData("Turret Target", r.t.getTurretTarget());
        telemetryM.addData("Turret Ticks", r.t.getTurret());
        telemetryM.addData("Shooter On", shoot);
        telemetryM.addData("Flipped Up", r.s.atUp());
        telemetryM.addData("Distance from Target", dist);
        telemetryM.addData("Manual Shooter + Turret", manual);
        telemetryM.addData("Field Centric", field);
        telemetryM.addData("Hold Position", hold);
        telemetryM.update(telemetry);
    }


    @Override
    public void stop() {
        r.stop();
    }

}
