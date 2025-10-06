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
    public static double targetV = 0;
    public boolean shoot = false;
    public static double shooterPower = 0;
    public static double shooterPowerHigh = 1;
    public static double flipUp = 0.5;
    public static double flipDown = 0.3;
    Limelight limelight;
    Shooter shooter;
    Intake intake;
    DcMotor sl;
    Servo flip;

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
       // limelight = new Limelight(hardwareMap, Alliance.BLUE);
        sl = hardwareMap.get(DcMotor.class, "sl");
        flip = hardwareMap.get(Servo.class, "f");

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

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
      //  limelight.start();
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
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

    //    Pose2D p = PoseConverter.poseToPose2D(follower.getPose(), DecodeCoordinates.INSTANCE);
     //   Logger.recordOutput("Pose2D", new Pose2d(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH), new Rotation2d(p.getHeading(AngleUnit.RADIANS))));

        if (gamepad1.right_bumper)
            intake.set(1);
        else if (gamepad1.left_bumper)
            intake.set(-1);
        else
            intake.set(.1);

        if (gamepad1.xWasPressed()) {
            tm = !tm;
        }

        double ttg = 0;
        double ttl = 0;
        double dy = 0;
        double dx = 0;
        if (gamepad1.dpadUpWasPressed() || tm) {
        //    turret.setYaw(turret.getYaw() + Math.toRadians(limelight.angleFromBlue()));
//            dy = target.getY() - follower.getPose().getY();
//            dx = target.getX() - follower.getPose().getX();
//            ttg = Math.atan2(dy , dx );
//            telemetryM.addData("ttg", ttg);
//            telemetry.addData("ttg", ttg);
//            ttl = MathFunctions.normalizeAngle(ttg + follower.getHeading());
//            telemetryM.addData("ttl", ttl);
//            telemetry.addData("ttl", ttl);
//            turret.setYaw(ttl);
        } else {
            turret.setYaw(turret.getYaw() + ((gamepad1.right_trigger - gamepad1.left_trigger) / 5));
        }

        if (shoot) {
            shooter.setTarget(targetV);
            shooter.periodic();
        } else
            sl.setPower(shooterPower);

        if (gamepad1.bWasPressed()) {
            if (shooterPower == 0) {
                shooterPower = shooterPowerHigh;
            } else {
                shooterPower = 0;
            }
            shoot = !shoot;
        }

        if (gamepad1.aWasPressed()) {
            if (flip.getPosition() == flipDown) {
                flip.setPosition(flipUp);
            } else {
                flip.setPosition(flipDown);
            }
        }

        turret.periodic();

        TelemetryPacket packet = new TelemetryPacket();
        //packet.put("Pose x", p.getX(DistanceUnit.INCH)); // Inches
   //     packet.put("Pose y", p.getY(DistanceUnit.INCH)); // Inches
  //      packet.put("Pose heading", p.getHeading(AngleUnit.RADIANS)); // Radians
//        packet.put("Turret Angle", turret.getYaw());
//        packet.put("Turret Target", Math.toRadians(turret.getTurretTarget()));
//        packet.put("Turret Auto Aim", tm);
        packet.put("Shooter Velocity", shooter.getVelocity());
        packet.put("Shooter Target", shooter.getTarget());
      //  packet.put("Limelight Distance", limelight.distanceFromShoot());
      //  packet.put("Limelight Angle", Math.toRadians(limelight.angleFromShoot()));
//        packet.put("Turret kp", Turret.kp);
//        packet.put("Turret kd", Turret.kd);
//        packet.put("Turret Error", Turret.error);
//        packet.put("Turret Power", Turret.power);
//        packet.put("Turret Target Goal", ttg);
//        packet.put("Turret Target Local", ttl);
//        packet.put("Dy", dy);
//        packet.put("Dx", dx);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.debug("velocity", shooter.getVelocity());
        telemetryM.debug("target", shooter.getTarget());
        telemetryM.debug("pid on: " + shoot);
        telemetryM.debug(dx);
        telemetryM.debug(dy);
        telemetryM.debug("ttg: " + ttg);
        telemetryM.debug("ttl: " + ttl);
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        Logger.end();
    }
}
