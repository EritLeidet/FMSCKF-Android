package com.msckf.lib.imageProcessing;

import boofcv.struct.image.GrayU8;

public class ImageMessage {
    public final GrayU8 image;
    public final double time;

    public ImageMessage(GrayU8 image, double time) {
        this.image = image;
        this.time = time;
    }
}
