package org.firstinspires.ftc.teamcode.config.vision;

import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

import java.util.List;
public class Limelight extends SubsystemBase {
    private Limelight3A l;
    private Alliance a;
    private static final int shoot = 0, zone = 1;
    private int pipeline = shoot;
    public Limelight(HardwareMap hardwareMap, Alliance alliance) {
        a = alliance;
        l = hardwareMap.get(Limelight3A.class, "limelight");
        switchToShoot();
        l.stop();
    }

    public void start() {
        l.start();
    }

    public void stop() {
        l.stop();
    }

    public void pause() {
        l.pause();
    }

    public double distanceFromTag(double tagID) {
        switchToShoot();
        List<LLResultTypes.FiducialResult> r = l.getLatestResult().getFiducialResults();
        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() ==  tagID) {
                target = i;
                break;
            }
        }

        if (target != null) {
            double x = target.getCameraPoseTargetSpace().getPosition().x; // right/left from tag
            double z = target.getCameraPoseTargetSpace().getPosition().z; // forward/back from tag

            Vector e = new Vector();
            e.setOrthogonalComponents(x, z);
            return e.getMagnitude();
        }

        return 0;
    }

    public double distanceFromBlue() {
        return distanceFromTag(20);
    }

    public double distanceFromRed() {
        return distanceFromTag(24);
    }

    public double angleFromTag(double tagID) {
        switchToShoot();
        List<LLResultTypes.FiducialResult> r = l.getLatestResult().getFiducialResults();
        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() ==  tagID) {
                target = i;
                break;
            }
        }

        if (target != null) {
            return target.getTargetXDegrees();
        }

        return 0;
    }

    public double angleFromBlue() {
        return angleFromTag(20);
    }

    public double angleFromRed() {
        return angleFromTag(24);
    }

    public double angleFromShoot() {
        return a == Alliance.BLUE ? angleFromBlue() : angleFromRed();
    }

    public double distanceFromShoot() {
        return a == Alliance.BLUE ? distanceFromBlue() : distanceFromRed();
    }

    public void switchToShoot() {
        if (pipeline != shoot)
            l.pipelineSwitch(shoot);
        l.start();
    }
}
