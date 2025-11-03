package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
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
    public static Pose shootTarget = new Pose(8, 144-8, 0);

    public static double targetV = 1200;
    public boolean shoot = false;
    public static double intakeOn = 0;

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        r.f.setStartingPose(Robot.endPose);
        r.t.resetTurret();

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

        telemetryM.addData("alliance", r.a);
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        if (r.a == Alliance.BLUE)
            shootTarget = new Pose(6, 144 - 6, 0);
        else
            shootTarget = new Pose(144-6, 144-6, 0);

        r.periodic();
        r.t.reset();
        r.f.startTeleopDrive();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        r.periodic();
        r.f.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, shoot ? -gamepad1.right_stick_x * 0.5 : -gamepad1.right_stick_x * 0.75, false, r.a == Alliance.BLUE ? Math.toRadians(180) : 0);

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

//        if (r.l.distanceFromShoot() != 0)
        r.s.forDistance(shootTarget.distanceFrom(r.f.getPose()));
//        else
//            r.s.setTarget(targetV);

        if (shoot) {
            r.s.on();
        } else
            r.s.off();

        r.s.periodic();

        if (gamepad1.bWasPressed()) {
            shoot = !shoot;
        }

        if (gamepad1.aWasPressed()) {
            r.s.flip();
        }

        if (gamepad1.dpadDownWasPressed())
            r.f.setPose(r.f.getPose().withHeading(0));

        // r.t.periodicError((r.l.angleFromShoot()));
        //if (gamepad1.dpad_up)
        r.t.face(shootTarget, r.f.getPose());

        r.t.periodic();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Shooter Velocity", r.s.getVelocity());
        packet.put("Shooter Target", r.s.getTarget());
        packet.put("Turret Yaw", r.t.getYaw());
        packet.put("Turret Target", r.t.getTurretTarget());
        packet.put("Turret Ticks", r.t.getTurret());

        //packet.put("limelight angle", r.l.angleFromShoot());
        //packet.put("Distance from shoot", r.l.distanceFromShoot());
        packet.put("Loop Time (ms)", r.getLoopTime());
        packet.put("Loop Time (hz)", (1000 / r.getLoopTime()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetryM.debug("x:" + r.f.getPose().getX());
        telemetryM.debug("y:" + r.f.getPose().getY());
        telemetryM.debug("heading:" + r.f.getPose().getHeading());
        telemetryM.debug("velocity", r.s.getVelocity());
        telemetryM.debug("target", r.s.getTarget());
        telemetryM.debug("Turret Yaw", r.t.getYaw());
        telemetryM.debug("Turret Target", r.t.getTurretTarget());
        telemetryM.debug("Turret Ticks", r.t.getTurret());
        //telemetryM.debug("limelight angle", r.l.angleFromShoot());
        //telemetryM.debug("Distance from shoot", r.l.distanceFromShoot());
        telemetryM.debug("Loop Time (ms)", r.getLoopTime());
        telemetryM.debug("Loop Time (hz)", (1000 / r.getLoopTime()));
        telemetryM.update(telemetry);
    }
}
