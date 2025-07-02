package com.android.msckfs;

import java.util.ArrayList;
import java.util.List;

public class Feature {

    public final int id;

    public List<Integer> cameraIds = new ArrayList<>();

    public Feature(int id, Integer cameraId) {
        this.id = id;
        this.cameraIds.add(cameraId);
    }
}
