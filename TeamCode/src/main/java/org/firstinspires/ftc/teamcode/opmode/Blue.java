package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@TeleOp
@Config
public class Blue extends OpMode {
    TelemetryManager telemetryM;
    MultipleTelemetry multipleTelemetry;

    Robot r;

    public static double targetV = 1200;
    public boolean shoot = false;
    public static double intakeOn = 0;

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        r.f.setStartingPose(Robot.endPose);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        r.s.down();
    }

    @Override
    public void start() {
        r.periodic();
        r.t.reset();
        r.l.switchToShoot();
        r.f.startTeleopDrive();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        r.periodic();
        r.f.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, shoot ? -gamepad1.right_stick_x * 0.5 : -gamepad1.right_stick_x * 0.75, false);

        if (gamepad1.rightBumperWasPressed())
            if (intakeOn == 2 || intakeOn == 1)
                intakeOn = 0;
            else
                intakeOn = 1;

        if (gamepad1.leftBumperWasPressed())
            if (intakeOn == 1)
                intakeOn = 0;
            else
                intakeOn = 2;

        if (intakeOn == 1)
            r.i.spinIn();
        else if (intakeOn == 2)
            r.i.spinOut();
        else
            r.i.spinIdle();

        if (r.l.distanceFromShoot() != 0)
            r.s.forDistance(r.l.distanceFromShoot());
        else
            r.s.setTarget(targetV);

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

        r.t.periodicError((r.l.angleFromShoot()));

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Shooter Velocity", r.s.getVelocity());
        packet.put("Shooter Target", r.s.getTarget());
        packet.put("limelight angle", r.l.angleFromShoot());
        packet.put("Distance from shoot", r.l.distanceFromShoot());
        packet.put("Loop Time (ms)", r.getLoopTime());
        packet.put("Loop Time (hz)", (1000 / r.getLoopTime()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetryM.debug("x:" + r.f.getPose().getX());
        telemetryM.debug("y:" + r.f.getPose().getY());
        telemetryM.debug("heading:" + r.f.getPose().getHeading());
        telemetryM.debug("velocity", r.s.getVelocity());
        telemetryM.debug("target", r.s.getTarget());
        telemetryM.debug("limelight angle", r.l.angleFromShoot());
        telemetryM.debug("Distance from shoot", r.l.distanceFromShoot());
        telemetryM.debug("Loop Time (ms)", r.getLoopTime());
        telemetryM.debug("Loop Time (hz)", (1000 / r.getLoopTime()));
        telemetryM.update(telemetry);
    }
}
