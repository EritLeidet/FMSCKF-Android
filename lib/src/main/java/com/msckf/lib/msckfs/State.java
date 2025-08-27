package com.msckf.lib.msckfs;

import static com.android.msckfs.utils.MathUtils.NANOSECOND_TO_SECOND;

public abstract class State {
    public int id;
    public double time; // unix time // TODO: convert to seconds time?

    public State(int id, double time) {
        this.id = id;
        this.time = time;
    }



}
