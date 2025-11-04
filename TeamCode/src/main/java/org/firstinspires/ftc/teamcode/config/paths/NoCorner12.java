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
    public Pose score = new Pose(48, 85.250, Math.toRadians(135)); // score
    public Pose intake1 = new Pose(20, 84, Math.toRadians(180)); // intake
    public Pose gate = new Pose(16.250, 70.000, Math.toRadians(180)); // gate
    public Pose intake2 = new Pose(16, 60.050, Math.toRadians(-170)); // intake
    public Pose intake3 = new Pose(16, 39.750, Math.toRadians(180));

    private int index = 0;
    private static final int PATH_COUNT = 14;

    public NoCorner12(Robot r) {
        this.f = r.f;

        if (r.a.equals(Alliance.RED)) {
            start = start.mirror();
            scoreEnd = scoreEnd.mirror();
            intake1 = intake1.mirror();
            gate = gate.mirror();
            line3End = line3End.mirror();
            line3End = line3End.mirror();
            intake2 = intake2.mirror();
            line5End = line5End.mirror();
            line6End = line6End.mirror();
            intake3 = intake3.mirror();
            line8End = line8End.mirror();
            line9End = line9End.mirror();
            line10End = line10End.mirror();
            line11End = line11End.mirror();
            line12End = line12End.mirror();
            line13End = line13End.mirror();
            line14End = line14End.mirror();
        }

        index = 0;
    }

    public PathChain score() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                start,
                                new Pose(55.593, 94.779),
                                scoreEnd
                        )
                )
                .setLinearHeadingInterpolation(start.getHeading(), scoreEnd.getHeading())
                .build();
    }

    public PathChain line1() { // intake 1
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scoreEnd,
                                new Pose(50.000, 80.000),
                                intake1
                        )
                )
                .setLinearHeadingInterpolation(scoreEnd.getHeading(), intake1.getHeading(), 0.3)
                .build();
    }

    public PathChain line2() { // go back from intake 1 to score
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, gate))
                .setLinearHeadingInterpolation(intake1.getHeading(), gate.getHeading())
                .build();
    }

    public PathChain line3() { // intake 2
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                gate,
                                new Pose(55.400, 66.300),
                                line3End
                        )
                )
                .setLinearHeadingInterpolation(gate.getHeading(), line3End.getHeading())
                .build();
    }

    public PathChain line4() { // go score intake 2
        return f.pathBuilder()
                .addPath(new BezierLine(line3End, intake2))
                .setLinearHeadingInterpolation(line3End.getHeading(), intake2.getHeading())
                .setReversed()
                .build();
    }

    public PathChain line5() { // do the gate
        return f.pathBuilder()
                .addPath(new BezierLine(intake2, line5End))
                .setLinearHeadingInterpolation(intake2.getHeading(), line5End.getHeading(), 0.5)
                .build();
    }

    public PathChain line6() { // intake 3
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                line5End,
                                new Pose(72, 44.000),
                                line6End
                        )
                )
                .setLinearHeadingInterpolation(line5End.getHeading(), line6End.getHeading())
                .build();
    }

    public PathChain line7() { // score intake 3
        return f.pathBuilder()
                .addPath(new BezierLine(line6End, intake3))
                .setLinearHeadingInterpolation(line6End.getHeading(), intake3.getHeading())
                .build();
    }

    public PathChain line8() { // to corner
        return f.pathBuilder()
                .addPath(new BezierLine(intake3, line8End))
                .setConstantHeadingInterpolation(intake3.getHeading())
                .build();
    }

    public PathChain line9() { // score corner
        return f.pathBuilder()
                .addPath(new BezierLine(line8End, line9End))
                .setConstantHeadingInterpolation(line8End.getHeading())
                .build();
    }

    public PathChain next() {
        switch (index++) {
            case 0: return score();
            case 1: return line1();
            case 2: return line2();
            case 3: return line3();
            case 4: return line4();
            case 5: return line5();
            case 6: return line6();
            case 7: return line7();
            case 8: return line8();
            case 9: return line9();
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
