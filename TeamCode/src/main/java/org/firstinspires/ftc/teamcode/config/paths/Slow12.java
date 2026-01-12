package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

public class Slow12 {
    private final Follower f;

    public Pose start = new Pose(24+6.25, 120+8+6, Math.toRadians(90));
    public Pose scorePControl = new Pose(55.593, 94.779);
    public Pose score = new Pose(48, 96.0, Math.toRadians(135)); // score
    public Pose intake1 = new Pose(17,88-1.5, Math.toRadians(180)); // intake\
    public Pose intake1Mid = intake1.withX(48);
    public Pose intake2 = new Pose(10, 62-1, Math.toRadians(180)); // intake
    public Pose intake2Mid = intake2.withX(48);
    public Pose intake2Return = intake2.withX(40);
    public Pose gate = new Pose(16, 75, Math.toRadians(180)); //new Pose(144-132.781509, 61, Math.toRadians(28+90)); // gate
    public Pose gateMid = gate.withX(27);
    public Pose intake3 = new Pose(10, 39.750+1.5-3-1, Math.toRadians(180));
    public Pose intake3Mid = intake3.withX(48);
    public Pose park = new Pose(48, 72, Math.toRadians(180));//new Pose(36, 12, Math.toRadians(180));

    private int index;

    public Slow12(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.RED)) {
            start = start.mirror();
            scorePControl = scorePControl.mirror();
            score = score.mirror();
            intake1 = intake1.mirror();
            intake1Mid = intake1Mid.mirror();
            gate = gate.mirror();
            gateMid = gateMid.mirror();
            intake2 = intake2.mirror();
            intake2Mid = intake2Mid.mirror();
            intake2Return = intake2Return.mirror();
            intake3 = intake3.mirror();
            intake3Mid = intake3Mid.mirror();
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

    public PathChain intake1() {
        return f.pathBuilder()
                .addPath(
                        new BezierLine(
                                intake1Mid,
                                intake1
                        )
                )
                .setConstantHeadingInterpolation(intake1.getHeading())
                .setBrakingStrength(0.75)
                .build();
    }

    public PathChain alignIntake1() {
        return f.pathBuilder()
                .addPath(
                        new BezierLine(
                                score,
                                intake1Mid
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), intake1Mid.getHeading(), 0.5)
                .build();
    }

    public PathChain gateAlign() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, gateMid))
                .setLinearHeadingInterpolation(intake1.getHeading(), gateMid.getHeading())
                .build();
    }

    public PathChain gate() {
        return f.pathBuilder()
                .addPath(new BezierLine(gateMid, gate))
                .setConstantHeadingInterpolation(gate.getHeading())
                .build();
    }

    public PathChain score1() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, score))
                .setLinearHeadingInterpolation(intake1.getHeading(), score.getHeading())
                .build();
    }


    public PathChain intake2() {
        return f.pathBuilder()
                .addPath(
                        new BezierLine(
                                intake2Mid,
                                intake2
                        )
                )
                .setConstantHeadingInterpolation(intake2.getHeading())
                .setBrakingStrength(0.75)
                .build();
    }

    public PathChain alignIntake2() {
        return f.pathBuilder()
                .addPath(
                        new BezierLine(
                                score,
                                intake2Mid
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), intake2Mid.getHeading(), 0.5)
                .build();
    }

    public PathChain score2() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, intake2Return, score))
                .setLinearHeadingInterpolation(intake2.getHeading(), score.getHeading())
                .build();
    }

    public PathChain alignIntake3() {
        return f.pathBuilder()
                .addPath(
                        new BezierLine(
                                score,
                                intake3Mid
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), intake3Mid.getHeading(), .5)
                .build();
    }

    public PathChain intake3() {
        return f.pathBuilder()
                .addPath(
                        new BezierLine(
                                intake3Mid,
                                intake3
                        )
                )
                .setConstantHeadingInterpolation(intake3.getHeading())
                .setBrakingStrength(0.75)
                .build();
    }

    public PathChain score3() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake3, score))
                .setLinearHeadingInterpolation(intake3.getHeading(), score.getHeading())
                .build();
    }

    public PathChain park() {
        return f.pathBuilder()
                .addPath(new BezierLine(score, park))
                .setLinearHeadingInterpolation(score.getHeading(), park.getHeading())
                .build();
    }

    public PathChain next() {
        switch (index++) {
            case 0: return scoreP();
            case 1: return alignIntake1();
            case 2: return intake1();
//            case 3: return gateAlign();
//            case 4: return gate();
            case 3: return score1();
            case 4: return alignIntake2();
            case 5: return intake2();
            case 6: return score2();
            case 7: return alignIntake3();
            case 8: return intake3();
            case 9: return score3();
            case 10: return park();
            default: return null;
        }
    }

    public boolean hasNext() {
        int PATH_COUNT = 12;
        return index < PATH_COUNT;
    }

    public void reset() {
        index = 0;
    }
}
