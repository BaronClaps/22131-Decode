package org.firstinspires.ftc.teamcode.config.command;


import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;

public class FollowPath extends Command {
    private final Follower follower;
    private final PathChain path;
    private boolean holdEnd = true;
    private double maxPower = 1;

    public FollowPath(Robot r, PathChain pathChain) {
        this.follower = r.d.getFollower();
        this.path = pathChain;
    }

    public FollowPath(Robot r, PathChain pathChain, double maxPower) {
        this.follower = r.d.getFollower();
        this.path = pathChain;
        this.maxPower = maxPower;
    }

    public FollowPath(Robot r, PathChain pathChain, boolean holdEnd) {
        this.follower = r.d.getFollower();
        this.path = pathChain;
        this.holdEnd = holdEnd;
    }

    public FollowPath(Robot r, PathChain pathChain, boolean holdEnd, double maxPower) {
        this.follower = r.d.getFollower();
        this.path = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    /**
     * Decides whether or not to make the robot maintain its position once the path ends.
     *
     * @param holdEnd If the robot should maintain its ending position
     * @return This command for compatibility in command groups
     */
    public FollowPath setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    /**
     * Sets the follower's maximum power
     * @param power Between 0 and 1
     * @return This command for compatibility in command groups
     */
    public FollowPath setMaxPower(double power) {
        this.maxPower = power;
        return this;
    }

    @Override
    public void start() {
        follower.followPath(path, maxPower, holdEnd);
    }

    @Override
    public boolean done() {
        return !follower.isBusy();
    }
}
