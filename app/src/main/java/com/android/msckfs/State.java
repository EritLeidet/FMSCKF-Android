package com.android.msckfs;

public abstract class State {
    public int id;
    public long timestamp; // unix time

    public State(int id, long timestamp) {
        this.id = id;
        this.timestamp = timestamp;
    }


}
