package com.msckf.lib.benchmark;


import static com.msckf.lib.utils.MathUtils.quaternionToRotation;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;

import com.msckf.lib.imageProcessing.CameraConfig;
import com.msckf.lib.imageProcessing.FeatureMessage;
import com.msckf.lib.imageProcessing.FeatureTracker;
import com.msckf.lib.imageProcessing.ImageMessage;
import com.msckf.lib.imuProcessing.ImuMessage;
import com.msckf.lib.msckfs.Fmsckf;
import com.msckf.lib.msckfs.Msckf;
import com.msckf.lib.msckfs.Odometry;
import com.msckf.lib.utils.Isometry3D;
import com.opencsv.CSVReader;
import com.opencsv.exceptions.CsvValidationException;

import org.ejml.simple.SimpleMatrix;
import org.yaml.snakeyaml.LoaderOptions;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;

import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.distort.brown.LensDistortionBrown;
import boofcv.alg.distort.pinhole.LensDistortionPinhole;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.calib.CameraPinhole;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.image.GrayU8;

/**
 * Parser for 100-Phones benchmark dataset.
 * References:
 * - <a href="https://github.com/zju3dv/100-Phones?tab=readme-ov-file">...</a>
 * - <a href="https://github.com/rohiitb/msckf_vio_python/blob/main/dataset.py">...</a>
 * - <a href="https://boofcv.org/index.php?title=Tutorial_Processing">...</a>
 * - <a href="https://www.geeksforgeeks.org/java/reading-csv-file-java-using-opencsv/">...</a>
 **/
public class Dataset {

    public static List<ImageMessage> loadImages(String datasetPath) throws CsvValidationException, IOException {
        List<GrayU8> images = UtilImageIO.loadImages(datasetPath + "/camera/images", ".+")
                .stream().map(bufferedImage -> ConvertBufferedImage.convertFrom(bufferedImage, (GrayU8) null)).collect(Collectors.toList());
        List<Double> timestamps = loadImageTimestamps(datasetPath);
        List<ImageMessage> imageMessages = new ArrayList<>(images.size());
        for (int i = 0; i < images.size(); i++) {
            imageMessages.add(new ImageMessage(images.get(i), timestamps.get(i)));
        }
        return imageMessages;
    }

    private static List<Double> loadImageTimestamps(String datasetPath) throws IOException, CsvValidationException {
        List<Double> timestamps = new ArrayList<>();
        CSVReader reader = new CSVReader(new FileReader(datasetPath + "/camera/data.csv"));
        String[] nextRecord;
        while ( (nextRecord = reader.readNext()) != null) {
            timestamps.add(Double.valueOf(nextRecord[0]));
        }

        reader.close();
        return timestamps;

    }

    public static List<Odometry> loadGroundTruth(String datasetPath) throws IOException, CsvValidationException {
        CSVReader reader = new CSVReader(new FileReader(datasetPath + "/groundTruth/data.csv"));
        String[] nextRecord;
        List<Odometry> groundTruth = new ArrayList<>();
        reader.readNext();
        while ( (nextRecord = reader.readNext()) != null) {
            // Convert quaternion Hamilton Convention (used in benchmark) to JPL convention (used in MSCKF).
            // TODO: muss ich dafür nicht quaternionConjugate verwenden???? Es reicht doch nicht, nur die Reihenfolge von [w x y z] zu ändern
            SimpleMatrix quaternion = new SimpleMatrix(new double[]{ // TODO: was ist das Quaternion-Format in ground truth?
                    Double.parseDouble(nextRecord[5]),
                    Double.parseDouble(nextRecord[6]),
                    Double.parseDouble(nextRecord[7]),
                    Double.parseDouble(nextRecord[4])
            });

            SimpleMatrix rotationMatrix = quaternionToRotation(quaternion);
            SimpleMatrix translation = new SimpleMatrix(new double[]{
                    Double.parseDouble(nextRecord[1]),
                    Double.parseDouble(nextRecord[2]),
                    Double.parseDouble(nextRecord[3])
            });
            Isometry3D pose = new Isometry3D(rotationMatrix, translation);
            groundTruth.add(new Odometry(Double.parseDouble(nextRecord[0]), pose, null, null));
        }

        reader.close();
        return groundTruth;
    }

    // TODO: Sensor-Daten von Kamera und imu mit integrieren.

    public static List<ImuMessage> loadImuData(String datasetPath) throws IOException, CsvValidationException {
        CSVReader reader = new CSVReader(new FileReader(datasetPath + "/imu/data.csv"));

        List<ImuMessage> imuMessages = new ArrayList<>();
        String[] nextRecord;
        while ( (nextRecord = reader.readNext()) != null) {
            float[] angularVelocity = new float[]{
                    Float.parseFloat(nextRecord[1]),
                    Float.parseFloat(nextRecord[2]),
                    Float.parseFloat(nextRecord[3])
            };
            float[] linearAcceleration = new float[]{
                    Float.parseFloat(nextRecord[4]),
                    Float.parseFloat(nextRecord[5]),
                    Float.parseFloat(nextRecord[6])
            };


            imuMessages.add(new ImuMessage(Double.parseDouble(nextRecord[0]), angularVelocity, linearAcceleration));
        }

        reader.close();

        return imuMessages;

    }

    /**
     * References:
     * - <a href="https://www.baeldung.com/java-snake-yaml">...</a>
     * - <a href="https://stackabuse.com/reading-and-writing-yaml-files-in-java-with-snakeyaml/">...</a>
     */
    public static BenchmarkCameraConfig loadCameraConfig(String datasetPath) throws FileNotFoundException {

        InputStream inputStream = new FileInputStream(datasetPath + "/camera/sensor.yaml");
        Yaml yaml = new Yaml(new Constructor(BenchmarkCameraConfig.class, new LoaderOptions()));
        // TODO: automatically remove two lines of imported file. (YAML-Version declaration)
        return yaml.load(inputStream);

    }

    public static BenchmarkCameraConfig loadImuConfig(String datasetPath) throws FileNotFoundException {

        InputStream inputStream = new FileInputStream(datasetPath + "/imu/sensor.yaml");
        Yaml yaml = new Yaml(new Constructor(BenchmarkCameraConfig.class, new LoaderOptions()));
        // TODO: automatically remove two lines of imported file. (YAML-Version declaration)
        return yaml.load(inputStream);

    }


    public static LensDistortionNarrowFOV buildLensDistortionModel(BenchmarkCameraConfig benchmarkConfig, int width, int height) {
        CameraInstrinsicParameters intrinsics = benchmarkConfig.getIntrinsic();

        // camera parameters
        float fx = intrinsics.getCamera()[0];
        float fy = intrinsics.getCamera()[1];
        float cx = intrinsics.getCamera()[2];
        float cy = intrinsics.getCamera()[3];

        // distortion parameters
        float k1 = intrinsics.getDistortion()[0];
        float k2 = intrinsics.getDistortion()[1];
        float r1 = intrinsics.getDistortion()[2];
        float r2 = intrinsics.getDistortion()[3];

        CameraPinholeBrown cameraModel = new CameraPinholeBrown(fx, fy, 0, cx, cy, width, height);
        cameraModel.setRadial(k1, k2);
        cameraModel.setT1(r1);
        cameraModel.setT2(r2);

        return new LensDistortionBrown(cameraModel);
    }


    public static void main(String[] args) throws CsvValidationException, IOException {

        // Parse benchmark data.
        final String datasetPath = "C:/Users/Jessica/Downloads/huawei-mate50";
        List<ImageMessage> images = loadImages(datasetPath);
        List<ImuMessage> imuData = loadImuData(datasetPath);
        List<Odometry> groundTruth = loadGroundTruth(datasetPath);
        BenchmarkCameraConfig benchmarkCameraConfig = loadCameraConfig(datasetPath);
        LensDistortionNarrowFOV lensDistortion = buildLensDistortionModel(benchmarkCameraConfig, images.get(0).image.width, images.get(0).image.height);

        FeatureTracker featureTracker = new FeatureTracker(lensDistortion);
        Msckf msckf = new Fmsckf();


        // Visualize parsing results.
        // ShowImages.showWindow(images.get(0), "Parse Result");


        // Execute benchmark test.
        int imuCounter = 0;
        int imageCounter = 0;
        List<Odometry> testResults = new LinkedList<>();
        while (imuCounter < imuData.size() || imageCounter < images.size()) {
            // We know that the image and imu data is sorted by timestamp in ascending order.
            if (!imuData.isEmpty() && imuData.get(imuCounter).time < images.get(imageCounter).time) {
                msckf.imuCallback(imuData.get(imuCounter));
                imuCounter++;
            } else {
                FeatureMessage featureMessage = featureTracker.trackFeatures(images.get(imageCounter));
                Odometry odom = msckf.featureCallback(featureMessage);
                testResults.add(odom);
                imageCounter++;
            }
        }
        // TODO: Orientierung visualisieren
        // TODO: Orientierung mit Ground Truth vergleichen.

    }
}
