package org.firstinspires.ftc.teamcode.config.vision;

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

            return (Math.sqrt((x * x) + (z * z)));
        }

        return 0;
    }

    public double distanceFromBlue() {
        return distanceFromTag(20);
    }

    public double distanceFromRed() {
        return distanceFromTag(24);
    }

    public void switchToShoot() {
        if (pipeline != shoot)
            l.pipelineSwitch(shoot);
    }
}
