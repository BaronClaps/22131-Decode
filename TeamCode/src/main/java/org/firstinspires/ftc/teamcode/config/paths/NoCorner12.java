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

    public Pose start = new Pose(24-3.5, 120+2.5, Math.toRadians(90));
    public Pose score = new Pose(48, 90.0, Math.toRadians(135)); // score
    public Pose intake1 = new Pose(20, 84, Math.toRadians(180)); // intake
    public Pose gate = new Pose(18.5, 71.000, Math.toRadians(180)); // gate
    public Pose intake2 = new Pose(16, 60.050, Math.toRadians(-170)); // intake
    public Pose intake3 = new Pose(16, 39.750, Math.toRadians(180));
    public Pose park = new Pose(24, 72, 0);

    private int index = 0;
    private static final int PATH_COUNT = 14;

    public NoCorner12(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.RED)) {
            start = start.mirror();
            score = score.mirror();
            intake1 = intake1.mirror();
            gate = gate.mirror();
            intake2 = intake2.mirror();
            intake3 = intake3.mirror();
            park = park.mirror();
        }

        index = 0;
    }

    public PathChain scoreP() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                start,
                                new Pose(55.593, 94.779),
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
                                new Pose(50.000, 80.000),
                                intake1
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), intake1.getHeading(), 0.3)
                .build();
    }

    public PathChain gate() { // go to gate from intake1
        return f.pathBuilder()
                .addPath(new BezierCurve(intake1, new Pose(48, intake1.getY()), gate))
                .setLinearHeadingInterpolation(intake1.getHeading(), gate.getHeading())
                .build();
    }

    public PathChain score1() {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, score))
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake2() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                new Pose(55.400, 66.300),
                                intake2
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), intake2.getHeading())
                .build();
    }

    public PathChain score2() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake2, score))
                .setLinearHeadingInterpolation(intake2.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake3() { // intake 3
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                new Pose(72, 44.000),
                                intake3
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), intake3.getHeading())
                .build();
    }

    public PathChain score3() { // score intake 3
        return f.pathBuilder()
                .addPath(new BezierLine(intake3, score))
                .setLinearHeadingInterpolation(intake3.getHeading(), score.getHeading())
                .build();
    }

    public PathChain park() { // to corner
        return f.pathBuilder()
                .addPath(new BezierLine(score, park))
                .setLinearHeadingInterpolation(score.getHeading(), park.getHeading())
                .setNoDeceleration()
                .build();
    }

    public PathChain next() {
        switch (index++) {
            case 0: return scoreP();
            case 1: return intake1();
            case 2: return gate();
            case 3: return score1();
            case 4: return intake2();
            case 5: return score2();
            case 6: return intake3();
            case 7: return score3();
            case 8: return park();
            default: return null;
        }
    }

    public boolean hasNext() {
        return index < PATH_COUNT;
    }

    public void reset() {
        index = 0;
    }
}
