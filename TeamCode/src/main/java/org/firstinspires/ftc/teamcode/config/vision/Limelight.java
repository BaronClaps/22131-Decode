package org.firstinspires.ftc.teamcode.config.vision;

import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.List;

public class Limelight extends SubsystemBase {
    private Limelight3A l;
    private static final int shoot = 0, zone = 1;
    private int pipeline = shoot;
    public Limelight(HardwareMap hardwareMap) {
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

    public Vector distanceFromTag(double tagID) {
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
            return e;
        }

        return new Vector();
    }

    public Vector distanceFromBlue() {
        return distanceFromTag(20);
    }

    public Vector distanceFromRed() {
        return distanceFromTag(24);
    }

    public void switchToShoot() {
        if (pipeline != shoot)
            l.pipelineSwitch(shoot);
    }
}
