package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystem.Turret;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.vision.Limelight;
import org.psilynx.psikit.Logger;
import org.psilynx.psikit.io.RLOGServer;

@TeleOp
@Config
public class Test extends OpMode {
    TelemetryManager telemetryM;
    MultipleTelemetry multipleTelemetry;

    Follower follower;
    Turret turret;
    public static double targetV = 1200;
    public boolean shoot = false;
    public static double intakeOn = 0;
    Limelight limelight;
    Shooter shooter;
    Intake intake;

    Pose target = new Pose(144-5, 5);
    boolean am = false, tm = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        PanelsConfigurables.INSTANCE.refreshClass(this);

        follower.setStartingPose(new Pose(8, 8.25));
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap, Alliance.BLUE);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        shooter.down();
        Logger.addDataReceiver(new RLOGServer());
        Logger.start();

    }

    /**
     * This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryM.update(telemetry);
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
        turret.reset();
        limelight.switchToShoot();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {

        if ((!follower.isBusy() && am) || (!follower.isBusy() && am && (Math.abs(gamepad1.left_stick_y) > 0.05 || Math.abs(gamepad1.left_stick_x) > 0.05 || Math.abs(gamepad1.right_stick_x) > 0.05))) {
            am = false;
            follower.startTeleopDrive();
        }

        if (!am)
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, shoot ? -gamepad1.right_stick_x * 0.75: -gamepad1.right_stick_x, false);
        follower.update();

    //    Pose2D p = PoseConverter.poseToPose2D(follower.getPose(), DecodeCoordinates.INSTANCE);
     //   Logger.recordOutput("Pose2D", new Pose2d(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH), new Rotation2d(p.getHeading(AngleUnit.RADIANS))));

        if (gamepad1.rightBumperWasPressed())
            if (intakeOn == 2 || intakeOn == 1)
                intakeOn = 0;
            else
                intakeOn = 1;

        if (gamepad1.leftBumperWasPressed())
            if (intakeOn == 1 || intakeOn == 2)
                intakeOn = 0;
            else
                intakeOn = 2;

        if (intakeOn == 1)
            intake.spinIn();
        else if (intakeOn == 2)
            intake.spinOut();
        else
            intake.spinIdle();

        if (gamepad1.xWasPressed()) {
            tm = !tm;
        }

        if (gamepad1.dpadUpWasPressed())
            tm = !tm;

        turret.periodicError((limelight.angleFromShoot()));
       // turret.m.setPower((gamepad1.right_trigger - gamepad1.left_trigger)/1.5);

        if (shoot) {
            shooter.setTarget(targetV);
            shooter.on();
        } else
            shooter.off();

        shooter.periodic();

        if (gamepad1.bWasPressed()) {
            shoot = !shoot;
        }

        if (gamepad1.aWasPressed()) {
            shooter.flip();
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Shooter Velocity", shooter.getVelocity());
        packet.put("Shooter Target", shooter.getTarget());
        packet.put("limelight angle", limelight.angleFromShoot());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.debug("velocity", shooter.getVelocity());
        telemetryM.debug("target", shooter.getTarget());
        telemetryM.debug("pid on: " + shoot);
        telemetryM.debug("xdeg", limelight.angleFromShoot());
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        Logger.end();
    }
}
