package frc.robot.util;

public class VisionTargetCache<T> {
    private final int expireAfterAmount;

    private int cycleAmtSinceLastUpdate;
    private T lastTarget;

    public VisionTargetCache(int expireAfterAmount) {
        this.expireAfterAmount = expireAfterAmount;
    }

    public void updateTarget(T target) {
        lastTarget = target;
        cycleAmtSinceLastUpdate = 0;
    }

    public boolean targetExpired() {
        return cycleAmtSinceLastUpdate > expireAfterAmount;
    }

    public boolean hasTarget() {
        return lastTarget != null;
    }

    public void reset() {
        cycleAmtSinceLastUpdate = 0;
        lastTarget = null;
    }

    public T getAndIncrement() {
        if(hasTarget())
            cycleAmtSinceLastUpdate++;

        return lastTarget;
    }
}
