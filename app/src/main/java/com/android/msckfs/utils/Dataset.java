package com.android.msckfs.utils;


import java.util.List;
import java.util.stream.Collectors;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.image.GrayU8;

/**
 * Parser for 100-Phones benchmark dataset.
 * References:
 * - <a href="https://github.com/zju3dv/100-Phones?tab=readme-ov-file">...</a>
 * - <a href="https://github.com/rohiitb/msckf_vio_python/blob/main/dataset.py">...</a>
 * - <a href="https://boofcv.org/index.php?title=Tutorial_Processing">...</a>
 **/
public class Dataset {


    public static class ImageReader {

        public static List<GrayU8> loadImages(String datasetPath) {
            return UtilImageIO.loadImages(datasetPath + "/camera/images", ".+")
                    .stream().map(ConvertBufferedImage::extractGrayU8).collect(Collectors.toList());
        }

        public static class ImageEntry {
            public final GrayU8 image;
            public final double time;

            public ImageEntry(GrayU8 image, double time) {
                this.image = image;
                this.time = time;
            }
        }

    }

    public static void main(String[] args) {
        // Try out parser.
        final String datasetPath = "C:/Users/Jessica/Downloads/huawei-mate50";
        List<GrayU8> images = Dataset.ImageReader.loadImages(datasetPath);

        // Visualize image parsing result.
        ShowImages.showWindow(images.get(0), "Parse Result");

    }
}
