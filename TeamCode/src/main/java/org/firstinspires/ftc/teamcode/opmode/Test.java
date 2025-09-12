package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Rotation;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystem.Turret;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.DecodeCoordinates;
import org.firstinspires.ftc.teamcode.config.vision.Limelight;
import org.psilynx.psikit.Logger;
import org.psilynx.psikit.io.RLOGServer;
import org.psilynx.psikit.wpi.Pose2d;
import org.psilynx.psikit.wpi.Rotation2d;

@TeleOp
@Config
public class Test extends OpMode {
    TelemetryManager telemetryM;
    MultipleTelemetry multipleTelemetry;

    Follower follower;
    Turret turret;
    Limelight limelight;

    Pose target = new Pose(144-5, 5);
    boolean am = false, tm = false;
    private DcMotor i;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        PanelsConfigurables.INSTANCE.refreshClass(this);

        follower.setStartingPose(new Pose(8, 8.25));
        i = hardwareMap.dcMotor.get("i");
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap, Alliance.BLUE);

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
        limelight.start();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {

        if ((!follower.isBusy() && am) || (!follower.isBusy() && am && (Math.abs(gamepad1.left_stick_y) > 0.05 || Math.abs(gamepad1.left_stick_x) > 0.05 || Math.abs(gamepad1.right_stick_x) > 0.05))) {
            follower.startTeleopDrive();
        }

        if (!am)
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        Pose2D p = PoseConverter.poseToPose2D(follower.getPose(), DecodeCoordinates.INSTANCE);
        Logger.recordOutput("Pose2D", new Pose2d(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH), new Rotation2d(p.getHeading(AngleUnit.RADIANS))));

        i.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if (gamepad1.xWasPressed()) {
            tm = !tm;
        }

        if (tm) {

            double ttg = Math.atan2(target.getY() - follower.getPose().getY(), target.getX() - follower.getPose().getX());
            double ttl = MathFunctions.normalizeAngle(ttg - follower.getHeading());
            double tt = turret.getYaw();
            double error = MathFunctions.normalizeAngle(ttl - tt);
            turret.setYaw(error);
        } //else {
          //  turret.addYaw((gamepad1.right_trigger - gamepad1.left_trigger) * 5);
      //  }

        if (gamepad1.yWasPressed()) {
            am = !am;
            if (am) {
                Path pa = new Path(new BezierLine(follower.getPose(), new Pose(85, 15, Math.toRadians(0))));
                pa.setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(0));
                follower.followPath(pa);
            }
        }

        turret.periodic();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Pose x", p.getX(DistanceUnit.INCH)); // Inches
        packet.put("Pose y", p.getY(DistanceUnit.INCH)); // Inches
        packet.put("Pose heading", p.getHeading(AngleUnit.RADIANS)); // Radians
        packet.put("Turret Angle", turret.getYaw());
        packet.put("Turret Target", Math.toRadians(turret.getTurretTarget()));
        packet.put("Turret Auto Aim", tm);
        packet.put("Limelight Distance", limelight.distanceFromShoot());
        packet.put("Limelight Angle", Math.toRadians(limelight.angleFromShoot()));
        packet.put("Turret kp", Turret.kp);
        packet.put("Turret kd", Turret.kd);
        packet.put("Turret Error", Turret.error);
        packet.put("Turret Power", Turret.power);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        Logger.end();
    }
}
