package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

public class NoCorner12 {
    private final Follower f;

    public Pose start = new Pose(24+6.25, 120+8+4.75, Math.toRadians(90));
    public Pose scorePControl = new Pose(55.593, 94.779);
    public Pose score = new Pose(48, 96.0, Math.toRadians(135)); // score
    public Pose intake1 = new Pose(21.5, 84, Math.toRadians(180)); // intake\
    public Pose intake1Control = new Pose(50.000, 80.000);
    public Pose intake2 = new Pose(18, 60.050-2, Math.toRadians(-170)); // intake
    public Pose intake2Control = new Pose(65.400, 59.00);
    public Pose gate = new Pose(16.25, 72.500, Math.toRadians(180)); //new Pose(144-132.781509, 61, Math.toRadians(28+90)); // gate
    public Pose gateControl = new Pose(48, 73); //62);
    public Pose intake3 = new Pose(16, 39.750-3.5, Math.toRadians(180));
    public Pose intake3Control = new Pose(75, intake3.getY()-5);
    public Pose intakeCorner = new Pose(12, 14, Math.toRadians(180+65));
    public Pose intakeCornerControl = intakeCorner.withY(50);
    public Pose scoreToCorner = score.withHeading(Math.toRadians(-135));
    public Pose scoreCorner = score; //new Pose(56, 20, Math.toRadians(180));
    public Pose park = new Pose(48, 72, Math.toRadians(180));//new Pose(36, 12, Math.toRadians(180));

    private int index;

    public NoCorner12(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.RED)) {
            start = start.mirror();
            scorePControl = scorePControl.mirror();
            score = score.mirror();
            intake1 = intake1.mirror();
            intake1Control = intake1Control.mirror();
            gate = gate.mirror();
            gateControl = gateControl.mirror();
            intake2 = intake2.mirror();
            intake2Control = intake2Control.mirror();
            intake3 = intake3.mirror();
            intake3Control = intake3Control.mirror();
            intakeCorner = intakeCorner.mirror();
            intakeCornerControl = intakeCornerControl.mirror();
            scoreToCorner = scoreToCorner.mirror();
            scoreCorner = scoreCorner.mirror();
            park = park.mirror();
        }

        index = 0;
    }

    public PathChain scoreP() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                start,
                                scorePControl,
                                score
                        )
                )
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake1() { // intake 1
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                intake1Control,
                                intake1
                        )
                )
                .setBrakingStrength(.5)
                .setLinearHeadingInterpolation(score.getHeading(), intake1.getHeading(), 0.3)
                .build();
    }

    public PathChain score1() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, score))
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake2() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                intake2Control,
                                intake2
                        )
                )
                .setBrakingStrength(.75)
                .setLinearHeadingInterpolation(score.getHeading(), intake2.getHeading(), 0.5)
                .build();
    }

    public PathChain score2() {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, score))
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();
    }

    public PathChain gate() { // go to gate from intake1
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, gateControl, gate))
                .setLinearHeadingInterpolation(intake2.getHeading(), gate.getHeading())
                .build();
    }

//    public PathChain scoreGate() {
//        return f.pathBuilder()
//                .addPath(new BezierCurve(gate, gateControl, score))
//                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
//                .build();
//    }

    public PathChain intake3() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, intake3Control, intake3))
                .setBrakingStrength(.75)
                .setLinearHeadingInterpolation(score.getHeading(), intake3.getHeading(), 0.7)
                .build();
    }

    public PathChain score3() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake3, scoreCorner))
                .setLinearHeadingInterpolation(intake3.getHeading(), scoreCorner.getHeading())
                .build();
    }

    public PathChain intakeCorner() {
        return f.pathBuilder()
                .addPath(new BezierCurve(scoreCorner, intakeCornerControl, intakeCorner))
                .setLinearHeadingInterpolation(scoreCorner.getHeading(), intakeCorner.getHeading(), 0.3)
                .build();
    }

    public PathChain scoreCorner() {
        return f.pathBuilder()
                .addPath(new BezierLine(intakeCorner, scoreCorner))
                .setLinearHeadingInterpolation(intakeCorner.getHeading(), scoreCorner.getHeading())
                .build();
    }

    public PathChain park() {
        return f.pathBuilder()
                .addPath(new BezierLine(scoreCorner, park))
                .setLinearHeadingInterpolation(scoreCorner.getHeading(), park.getHeading())
                .build();
    }

    public PathChain next() {
        switch (index++) {
            case 0: return scoreP();
            case 1: return intake1();
            case 2: return score1();
            case 3: return intake2();
            case 4: return gate();
            case 5: return score2();
            //case 6: return scoreGate();
            case 6: return intake3();
            case 7: return score3();
            case 8: return intakeCorner();
            case 9: return scoreCorner();
            case 10: return park();
            default: return null;
        }
    }

    public boolean hasNext() {
        int PATH_COUNT = 9;
        return index < PATH_COUNT;
    }

    public void reset() {
        index = 0;
    }
}
