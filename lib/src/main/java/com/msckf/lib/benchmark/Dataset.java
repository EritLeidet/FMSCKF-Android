package com.msckf.lib.benchmark;


import static com.msckf.lib.utils.MathUtils.quaternionToRotation;
import static com.msckf.lib.utils.MathUtils.toDoubleArray;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;

import com.msckf.lib.benchmark.datatypes.cam.BenchmarkCameraConfig;
import com.msckf.lib.benchmark.datatypes.cam.CameraInstrinsicParameters;
import com.msckf.lib.benchmark.datatypes.imu.BenchmarkImuConfig;
import com.msckf.lib.benchmark.datatypes.imu.DiscreteNoiseParameters;
import com.msckf.lib.imageProcessing.FeatureMessage;
import com.msckf.lib.imageProcessing.FeatureTracker;
import com.msckf.lib.imageProcessing.ImageMessage;
import com.msckf.lib.imuProcessing.ImuMessage;
import com.msckf.lib.msckfs.Fmsckf;
import com.msckf.lib.msckfs.Msckf;
import com.msckf.lib.msckfs.MsckfExternalConfig;
import com.msckf.lib.msckfs.Odometry;
import com.msckf.lib.utils.Isometry3D;
import com.opencsv.CSVReader;
import com.opencsv.CSVWriter;
import com.opencsv.exceptions.CsvValidationException;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;
import org.yaml.snakeyaml.LoaderOptions;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;

import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.distort.brown.LensDistortionBrown;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
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
            SimpleMatrix jplQuaternion = new SimpleMatrix(new double[]{
                    Double.parseDouble(nextRecord[1]),
                    Double.parseDouble(nextRecord[2]),
                    Double.parseDouble(nextRecord[3]),
                    Double.parseDouble(nextRecord[4])
            });

            SimpleMatrix rotationMatrix = quaternionToRotation(jplQuaternion);
            SimpleMatrix translation = new SimpleMatrix(new double[]{
                    Double.parseDouble(nextRecord[5]),
                    Double.parseDouble(nextRecord[6]),
                    Double.parseDouble(nextRecord[7])
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

        InputStream inputStream = new FileInputStream(datasetPath + "/camera/sensor.yaml"); // TODO: do you need to .close()?
        Yaml yaml = new Yaml(new Constructor(BenchmarkCameraConfig.class, new LoaderOptions()));
        // TODO: automatically remove two lines of imported file. (YAML-Version declaration)
        return yaml.load(inputStream);


    }

    public static BenchmarkImuConfig loadImuConfig(String datasetPath) throws FileNotFoundException {

        InputStream inputStream = new FileInputStream(datasetPath + "/imu/sensor.yaml");
        Yaml yaml = new Yaml(new Constructor(BenchmarkImuConfig.class, new LoaderOptions()));
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

    public static List<String[]> odometryAsTable(List<Odometry> odometries) {
        List<String[]> table = new ArrayList<>(odometries.size());
        for (Odometry o : odometries) {

            String[] tableRow = new String[]{
                    String.valueOf(o.time),             // t
                    String.valueOf(o.pose.t.get(0)),    // pos.x
                    String.valueOf(o.pose.t.get(1)),    // pos.y
                    String.valueOf(o.pose.t.get(2)),    // pos.z

            };
            table.add(tableRow);
        }
        return table;
    }

    public static MsckfExternalConfig createMsckfConfig(BenchmarkCameraConfig cameraConfig, BenchmarkImuConfig imuConfig) {
        // camera to body transform. (In benchmark body pose and imu pose are the synonymous.)
        float[] rotationArray = cameraConfig.getExtrinsic().getQ();
        float[] translationArray = cameraConfig.getExtrinsic().getP();
        DMatrixRMaj rotationQuaternion = new DMatrixRMaj(toDoubleArray(rotationArray));
        DMatrixRMaj rotationMatrix = quaternionToRotation(rotationQuaternion);
        DMatrixRMaj translationMatrix = new DMatrixRMaj(toDoubleArray(translationArray));
        Isometry3D camBody = new Isometry3D(SimpleMatrix.wrap(rotationMatrix), SimpleMatrix.wrap(translationMatrix));

        // Noise related parameters.
        // TODO: try out the possibility that the unit is [rad/s] is meant instead of [rad/s/hz]
        DiscreteNoiseParameters noiseParameters = imuConfig.getIntrinsic();
        double gyroNoise = noiseParameterToVariance(noiseParameters.getSigma_w(), imuConfig.getFrequency());
        double gyroBiasNoise = noiseParameterToVariance(noiseParameters.getSigma_bw(), imuConfig.getFrequency());
        double accNoise = noiseParameterToVariance(noiseParameters.getSigma_a(), imuConfig.getFrequency());
        double accBiasNoise = noiseParameterToVariance(noiseParameters.getSigma_ba(), imuConfig.getFrequency());

        return new MsckfExternalConfig(camBody, gyroNoise, gyroBiasNoise, accNoise, accBiasNoise);

    }

    /**
     * Convert noise density or random walk to variance.
     * @param sampleRate in units of hz
     * References:
     * - <a href="https://www.vectornav.com/resources/inertial-navigation-primer/examples/noise">...</a>
     */
    public static double noiseParameterToVariance(double noiseParameter, double sampleRate) {
        return noiseParameter * noiseParameter * sampleRate;
    }





    public static void main(String[] args) throws CsvValidationException, IOException {


        // Parse benchmark data.
        final String datasetPath = "C:/Users/Jessica/Downloads/huawei-mate50";
        List<ImageMessage> images = loadImages(datasetPath);
        List<ImuMessage> imuData = loadImuData(datasetPath);
        List<Odometry> groundTruth = loadGroundTruth(datasetPath);
        BenchmarkCameraConfig cameraConfig = loadCameraConfig(datasetPath);
        BenchmarkImuConfig imuConfig = loadImuConfig(datasetPath);

        // Create configurations from benchmark data.
        LensDistortionNarrowFOV lensDistortion = buildLensDistortionModel(cameraConfig, images.get(0).image.width, images.get(0).image.height);
        MsckfExternalConfig msckfConfig = createMsckfConfig(cameraConfig, imuConfig);
        FeatureTracker featureTracker = new FeatureTracker(lensDistortion);
        Msckf msckf = new Fmsckf(msckfConfig); // TODO: add Config to Msckf


        // Visualize parsing results.
        // ShowImages.showWindow(images.get(0), "Parse Result");


        // Execute benchmark test.
        int imuCounter = 0;
        int imageCounter = 0;
        List<Odometry> testResults = new LinkedList<>();
        int cutoff = 2;
        while ((imuCounter < imuData.size() || imageCounter < images.size()) && cutoff > 0) {
            // We know that the image and imu data is sorted by timestamp in ascending order.
            if (!imuData.isEmpty() && imuData.get(imuCounter).time < images.get(imageCounter).time) {
                msckf.imuCallback(imuData.get(imuCounter));
                imuCounter++;
            } else {
                FeatureMessage featureMessage = featureTracker.trackFeatures(images.get(imageCounter));
                Odometry odom = msckf.featureCallback(featureMessage);
                if (odom != null) {
                    testResults.add(odom);
                    cutoff--;
                }
                imageCounter++;
            }
        }

        if (testResults.isEmpty()) throw new AssertionError();

        // Save results to file.
        String exportPath = "C:/Users/Jessica/Downloads/";
        CSVWriter writer = new CSVWriter(
                new FileWriter(exportPath + "benchmarkResults.csv"),
                ',',
                CSVWriter.NO_QUOTE_CHARACTER,
                CSVWriter.DEFAULT_ESCAPE_CHARACTER,
                CSVWriter.DEFAULT_LINE_END);
        List<String[]> resultTable = odometryAsTable(testResults);

        System.out.println("TESTING?");
        //System.out.println(Arrays.toString(resultTable.get(0)));
        for (String[] row : resultTable) {
            System.out.println(Arrays.toString(row));
        }
        if (resultTable.isEmpty()) throw new AssertionError();
        try {
            writer.writeAll(resultTable);
        } finally {
            writer.close();
        }
        ;


        // TODO: Orientierung visualisieren
        // TODO: Orientierung mit Ground Truth vergleichen.


    }
}
