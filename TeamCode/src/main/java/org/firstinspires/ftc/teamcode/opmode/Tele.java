package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class Tele extends OpMode {
    TelemetryManager telemetryM;
    MultipleTelemetry multipleTelemetry;

    Robot r;

    public boolean shoot = false, manual = false;
    public static double intakeOn = 0;
    private final Timer upTimer = new Timer();

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        r.f.setStartingPose(Robot.endPose);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        r.s.down();
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed())
            r.a = Alliance.BLUE;

        if (gamepad1.bWasPressed())
            r.a = Alliance.RED;

        if (gamepad1.xWasPressed())
            r.t.resetTurret();

        telemetryM.addData("alliance", r.a);
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        r.periodic();
        r.t.reset();
        r.f.startTeleopDrive();

        upTimer.resetTimer();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        r.periodic();
        r.f.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, shoot ? -gamepad1.right_stick_x * 0.5 : -gamepad1.right_stick_x * 0.75, false, r.a == Alliance.BLUE ? Math.toRadians(180) : 0);

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
                double dist = r.getShootTarget().distanceFrom(r.f.getPose());
                r.s.forDistance(dist);
                r.t.face(r.getShootTarget(), r.f.getPose());
                r.t.automatic();
            }
        } else {
            r.s.off();
            r.t.off();
        }

        if (gamepad1.bWasPressed()) {
            shoot = !shoot;
        }

        if (gamepad1.aWasPressed()) {
            upTimer.resetTimer();
            r.s.flip();
        }

        if (gamepad1.dpadLeftWasPressed())
            manual = !manual;

        if (gamepad1.dpadDownWasPressed()) {
            if (r.a.equals(Alliance.BLUE)) {
                r.f.setPose(new Pose(8,6.25, Math.toRadians(0)).mirror());
            } else {
                r.f.setPose(new Pose(8,6.25, Math.toRadians(0)));
            }
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Shooter Velocity", r.s.getVelocity());
        packet.put("Shooter Target", r.s.getTarget());
        packet.put("Turret Yaw", r.t.getYaw());
        packet.put("Turret Target", r.t.getTurretTarget());
        packet.put("Turret Ticks", r.t.getTurret());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetryM.debug("Up Timer", upTimer.getElapsedTimeSeconds());
        telemetryM.debug("x:" + r.f.getPose().getX());
        telemetryM.debug("y:" + r.f.getPose().getY());
        telemetryM.debug("heading:" + r.f.getPose().getHeading());
        telemetryM.debug("velocity", r.s.getVelocity());
        telemetryM.debug("target", r.s.getTarget());
        telemetryM.debug("Turret Yaw", r.t.getYaw());
        telemetryM.debug("Turret Target", r.t.getTurretTarget());
        telemetryM.debug("Turret Ticks", r.t.getTurret());
        telemetryM.update(telemetry);
    }
}
