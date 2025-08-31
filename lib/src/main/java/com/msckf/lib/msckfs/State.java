package com.msckf.lib.msckfs;

public abstract class State {
    public int id;
    public double time; // seconds

    public State(int id, double time) {
        this.id = id;
        this.time = time;
    }



}
