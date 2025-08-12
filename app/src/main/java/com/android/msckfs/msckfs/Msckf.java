package com.android.msckfs.msckfs;
import static com.android.msckfs.utils.MathUtils.deleteColumns;
import static com.android.msckfs.utils.MathUtils.deleteRows;
import static com.android.msckfs.utils.MathUtils.fromTwoVectors;
import static com.android.msckfs.utils.MathUtils.quaternionMultiplication;
import static com.android.msckfs.utils.MathUtils.quaternionNormalize;
import static com.android.msckfs.utils.MathUtils.quaternionToRotation;
import static com.android.msckfs.utils.MathUtils.rotationToQuaternion;
import static com.android.msckfs.utils.MathUtils.skewSymmetric;
import static com.android.msckfs.utils.MathUtils.smallAngleQuaternion;
import static org.ejml.dense.row.CommonOps_DDRM.add;
import static org.ejml.dense.row.CommonOps_DDRM.divide;
import static org.ejml.dense.row.CommonOps_DDRM.identity;
import static org.ejml.dense.row.CommonOps_DDRM.insert;
import static org.ejml.dense.row.CommonOps_DDRM.mult;
import static org.ejml.dense.row.CommonOps_DDRM.scale;
import static org.ejml.dense.row.CommonOps_DDRM.setIdentity;
import static org.ejml.dense.row.CommonOps_DDRM.transpose;

import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Thread.sleep;

import android.util.Log;

import com.android.msckfs.imageProcessing.FeatureMessage;
import com.android.msckfs.imuProcessing.ImuMessage;
import com.android.msckfs.imageProcessing.FeatureMeasurement;
import com.android.msckfs.utils.Isometry3D;

import org.apache.commons.statistics.distribution.ChiSquaredDistribution;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.dense.row.linsol.chol.LinearSolverCholLDL_DDRM;
import org.ejml.interfaces.decomposition.QRSparseDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.simple.SimpleMatrix;
import org.ejml.sparse.FillReducing;
import org.ejml.sparse.csc.factory.DecompositionFactory_DSCC;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * <a href="https://github.com/Edwinem/msckf_tutorial/blob/main/src/msckf.py">...</a>
 * <a href="https://ejml.org/wiki/index.php?title=Example_Kalman_Filter">...</a>
 * <a href="https://github.com/daniilidis-group/msckf_mono/blob/master/include/msckf_mono/msckf.h">...</a>
 */

public class Msckf {

    private final static String TAG = "MSCKF";
    /**
     *  IMU data buffer.
     *  This is buffer is used to handle the unsynchronization or
     *  transfer delay between IMU and Image messages.
     */
    private final List<ImuMessage> imuBuffer = Collections.synchronizedList(new LinkedList<>());

    private final StateServer stateServer;


    private boolean isGravitySet = false;

    // Indicate if the received image is the first one. The system will
    // start after receiving the first image.
    private boolean isFirstImg = true;
    private final DMatrixRMaj F, G;


    private DMatrixRMaj g = new DMatrixRMaj();
    private final DMatrixRMaj Fdt = new DMatrixRMaj(), FdtSquare = new DMatrixRMaj(), FdtCube = new DMatrixRMaj();

    private final Map<Integer,Double> chiSquaredTestTable = new HashMap<>();
    
    public Msckf() {
        assert(!denseSolver.modifiesB());

        F = new DMatrixRMaj(StateInfo.IMU_STATE_SIZE,StateInfo.IMU_STATE_SIZE);
        G = new DMatrixRMaj(StateInfo.IMU_STATE_SIZE,12); // 12, not 21.

        SimpleMatrix continuousNoiseCov = new SimpleMatrix(12,12);
        continuousNoiseCov.insertIntoThis(0,0, SimpleMatrix.identity(3).scale(Config.GYRO_NOISE));
        continuousNoiseCov.insertIntoThis(3,3, SimpleMatrix.identity(3).scale(Config.GYRO_BIAS_NOISE));
        continuousNoiseCov.insertIntoThis(6,6, SimpleMatrix.identity(3).scale(Config.ACC_NOISE));
        continuousNoiseCov.insertIntoThis(9,9, SimpleMatrix.identity(3).scale(Config.ACC_BIAS_NOISE));
        SimpleMatrix stateCov = new SimpleMatrix(StateInfo.IMU_STATE_SIZE, StateInfo.IMU_STATE_SIZE);
        this.stateServer = new StateServer(stateCov, continuousNoiseCov);

        // Initialize the chi squared test table with confidence
        // level 0.95.
        for (int i = 1; i<100; i++) {
            ChiSquaredDistribution chiSquaredDist = ChiSquaredDistribution.of(i);
            chiSquaredTestTable.put(i, chiSquaredDist.inverseCumulativeProbability(0.05));

        }

        stateServer.imuState.velocity = Config.velocity;
        resetStateCov();

        // Gravity vector in the world frame
        ImuState.GRAVITY = Config.GRAVITY;

        // Transformation between the IMU and the camera
        SimpleMatrix tCamImu = Config.tImuCam.invert();
        stateServer.imuState.rImuCam = tCamImu.extractMatrix(0,3,0,3).transpose();
        stateServer.imuState.tCamImu = tCamImu.extractMatrix(0,3,3,SimpleMatrix.END);

        // Extrinsic parameters of camera and IMU
        ImuState.tImuBody = new Isometry3D(
                Config.tImuBody.extractMatrix(0,3,0,3).transpose(),
                Config.tImuBody.extractMatrix(0,3,3,SimpleMatrix.END));


    }

    public void resetStateCov() {
        SimpleMatrix stateCov = new SimpleMatrix(StateInfo.IMU_STATE_SIZE, StateInfo.IMU_STATE_SIZE);
        SimpleMatrix identity = SimpleMatrix.identity(3);
        stateCov.insertIntoThis(3,3, identity.scale(Config.GYRO_BIAS_COV));
        stateCov.insertIntoThis(6,6, identity.scale(Config.VELOCITY_COV));
        stateCov.insertIntoThis(9,9, identity.scale(Config.ACC_BIAS_COV));
        stateCov.insertIntoThis(15,15, identity.scale(Config.EXTRINSIC_ROTATION_COV));
        stateCov.insertIntoThis(18,18, identity.scale(Config.EXTRINSIC_TRANSLATION_COV));
        stateServer.stateCov = stateCov;

    }

    /**
     * Computes the transition matrix F for the EKF.
     * The transition matrix contains the jacobians of our process model with respect to our current state.
     */
    public void calcF(SimpleMatrix gyro, SimpleMatrix acc) {
        F.zero();

        // f = -skew(gyro)
        // Matrix calculation buffers
        DMatrixRMaj f = skewSymmetric(gyro).getDDRM();
        scale(-1, f);
        insert(f,F,0,0);

        // f = eye(3)
        setIdentity(f);
        insert(f,F,12,6);

        // f = -eye(3)
        scale(-1, f);
        insert(f,F,0,3);

        // f = -to_rotation(imu_state.orientation).T
        f = quaternionToRotation(stateServer.imuState.orientation.getDDRM());
        transpose(f);
        scale(-1, f);
        insert(f,F,6,9);

        // f = -to_rotation(imu_state.orientation).T * skew(acc)
        f = mult(f,skewSymmetric(acc.getDDRM()), new DMatrixRMaj());
        insert(f,F,6,0);
    }

    /**
     * This matrix contains the jacobians of our process model with respect to the system input.
     */
    public void calcG() {
        G.zero();

        // g = -eye(3)
        setIdentity(g);
        scale(-1, g);
        insert(g,G,0,0);

        // g = eye(3)
        setIdentity(g);
        insert(g,G,3,3);
        insert(g,G,9,9);

        // g = -to_rotation(imu_state.orientation).T
        g = quaternionToRotation(stateServer.imuState.orientation.getDDRM());
        transpose(g);
        scale(-1,g);
        insert(g,G,6,6);

    }


    private static class Config {
        static final double GRAVITY_ACC = 9.81f;
        static final SimpleMatrix GRAVITY = new SimpleMatrix(new double[] {0.0,0.0,-GRAVITY_ACC});
        static final int MAX_CAM_STATES = 20;

        // Noise related parameters (Use variance instead of standard deviation)

        // TODO: undo my changes to the noise

        static final int more = 1;
        static final double GYRO_NOISE = pow(0.005, 2) * more;
        static final double ACC_NOISE = pow(0.05,2) * more;
        static final double GYRO_BIAS_NOISE = pow(0.001,2) * more;
        static final double ACC_BIAS_NOISE = pow(0.01,2) * more;
        static final double OBSERVATION_NOISE = pow(0.035,2);

        // Initial state
        static final SimpleMatrix velocity = new SimpleMatrix(3,1);

        // The initial covariance of orientation and position can be
        // set to 0. But for velocity, bias and extrinsic parameters,
        // there should be nontrivial uncertainty
        static final double VELOCITY_COV = 0.25 * more;
        static final double GYRO_BIAS_COV = 0.01 * more;
        static final double ACC_BIAS_COV = 0.01 * more;
        static final double EXTRINSIC_ROTATION_COV = 3.0462e-4 * more;
        static final double EXTRINSIC_TRANSLATION_COV = 2.5e-5 * more;


        // calibration parameters
        // T_imu_cam: takes a vector from the IMU frame to the cam frame.
        static final SimpleMatrix tImuCam = SimpleMatrix.identity(4); // TODO:  what are these for my phone? How to find out? See paper.

        static final SimpleMatrix tImuBody = SimpleMatrix.identity(4);

    }

    private static class StateInfo {
        static final int IMU_STATE_SIZE = 21;
        static final int CAM_STATE_SIZE = 6;

    }

    private int getStateSize() {
        return StateInfo.IMU_STATE_SIZE + stateServer.camStates.size() * StateInfo.CAM_STATE_SIZE;
    }

    /**
     *
     */
    public void processModel(double time, SimpleMatrix mGyro, SimpleMatrix mAcc) {

        final String localTag = "processModel";
        ImuState imuState = stateServer.imuState;

        SimpleMatrix gyro = mGyro.minus(imuState.gyroBias);
        SimpleMatrix acc = mAcc.minus(imuState.accBias);
        double dt = time - imuState.time;

        // Compute discrete transition F, G matrices in Appendix A in "MSCKF" paper
        calcF(gyro, acc);
        calcG();

        // Approximate matrix exponential to the 3rd order, which can be
        // considered to be accurate enough assuming dt is within 0.01s
        scale(dt, F, Fdt);
        mult(Fdt,Fdt,FdtSquare);
        mult(FdtSquare,Fdt,FdtCube);

        // Phi = np.eye(21) + Fdt + 0.5*Fdt_square + (1./6.)*Fdt_cube
        SimpleMatrix Phi = SimpleMatrix
                .identity(StateInfo.IMU_STATE_SIZE)
                .plus(SimpleMatrix.wrap(Fdt))
                .plus(SimpleMatrix.wrap(FdtSquare).scale(0.5))
                .plus(SimpleMatrix.wrap(FdtCube).scale(1./6.));

        // Propogate the state using 4th order Runge-Kutta
        predictNewState(dt, gyro, acc);

        // Modify the transition matrix
        SimpleMatrix Rkk1 = quaternionToRotation(imuState.orientationNull);
        Phi.insertIntoThis(0,0, quaternionToRotation(imuState.orientation).mult(Rkk1.transpose()));
        SimpleMatrix u = Rkk1.mult(ImuState.GRAVITY); // vec3
        SimpleMatrix s = (u.transpose().mult(u)).invert().mult(u.transpose()); // is a row vector in C++ MSCKF-S implementations. Column vector in Python.
        SimpleMatrix A1 = Phi.extractMatrix(6,9,0,3);
        SimpleMatrix w1 = skewSymmetric(imuState.velocityNull.minus(imuState.velocity))
                .mult(ImuState.GRAVITY); // vec3

        Phi.insertIntoThis(6,0,
                A1.minus(
                        A1
                                .mult(u) // col vec
                                .minus(w1) // col vec
                        .mult(s)));
        SimpleMatrix A2 = Phi.extractMatrix(12,15,0,3);
        SimpleMatrix w2 = skewSymmetric(imuState.velocityNull.scale(dt).plus(imuState.positionNull).minus(imuState.position))
                .mult(ImuState.GRAVITY);
        Phi.insertIntoThis(12,0,
                A2
                        .minus(
                                A2
                                        .mult(u)
                                        .minus(w2)
                                        .mult(s)));
        // Propogate the state covariance matrix.
        SimpleMatrix Gwrapped = SimpleMatrix.wrap(G);
        SimpleMatrix Q = Phi
                .mult(Gwrapped)
                .mult(stateServer.continuousNoiseCov)
                .mult(Gwrapped.transpose())
                .mult(Phi.transpose())
                .scale(dt);
        stateServer.stateCov.insertIntoThis(0,0,
                Phi
                        .mult(stateServer.stateCov.extractMatrix(0,StateInfo.IMU_STATE_SIZE,0,StateInfo.IMU_STATE_SIZE))
                        .mult(Phi.transpose())
                        .plus(Q));

        if (!stateServer.camStates.isEmpty()) {
            stateServer.stateCov.insertIntoThis(0, StateInfo.IMU_STATE_SIZE, Phi.mult(
                    stateServer.stateCov.extractMatrix(0,StateInfo.IMU_STATE_SIZE,StateInfo.IMU_STATE_SIZE,SimpleMatrix.END)));
            stateServer.stateCov.insertIntoThis(StateInfo.IMU_STATE_SIZE,0,
                    stateServer.stateCov.extractMatrix(StateInfo.IMU_STATE_SIZE,SimpleMatrix.END,0,StateInfo.IMU_STATE_SIZE).mult(Phi.transpose()));
        }

        // Fix the covariance to be symmetric
        stateServer.stateCov = stateServer.stateCov.plus(stateServer.stateCov.transpose()).scale(0.5);


        // Update the state correspondes to null space.
        stateServer.imuState.orientationNull = imuState.orientation;
        stateServer.imuState.positionNull = imuState.position;
        stateServer.imuState.velocityNull = imuState.velocity;

        // Update the state info
        stateServer.imuState.time = time;
    }

    public void imuCallback(ImuMessage imuMsg) {
        imuBuffer.add(imuMsg);

        if (!isGravitySet && imuBuffer.size() >= 200) {
            initializeGravityAndBias();
            this.isGravitySet = true;
        }
    }


    /**
     * Initialize the IMU bias and initial orientation based on the
     * first few IMU readings.
     */
    public void initializeGravityAndBias() {
        // Initialize the gyro_bias given the current angular and linear velocity
        DMatrixRMaj sumAngVel = new DMatrixRMaj(3,1);
        DMatrixRMaj sumLinearAcc = new DMatrixRMaj(3,1);

        SimpleMatrix gravityImu;
        synchronized (imuBuffer) {
            for (ImuMessage imuMsg : imuBuffer) {
                add(sumAngVel, imuMsg.angularVelocity.getDDRM(), sumAngVel);
                add(sumLinearAcc, imuMsg.linearAcceleration.getDDRM(), sumLinearAcc);
            }

            stateServer.imuState.gyroBias = SimpleMatrix.wrap(sumAngVel).divide(imuBuffer.size());


            // Find the gravity in the IMU frame.
            gravityImu = SimpleMatrix.wrap(sumLinearAcc).divide(imuBuffer.size());
        }

        // Normalize the gravity and save to IMUState
        double gravNorm = gravityImu.normF();
        ImuState.GRAVITY = new SimpleMatrix(new double[] {0.0, 0.0, -gravNorm});
        // Initialize the initial orientation, so that the estimation
        // is consistent with the inertial frame.
        stateServer.imuState.orientation = fromTwoVectors(ImuState.GRAVITY.negative(), gravityImu);
    }


    private int featureCallbackDebugCt = 0;

    public Odometry featureCallback(FeatureMessage featureMsg) {
        if (!this.isGravitySet) return null;

        if (isFirstImg) {
            isFirstImg = false;
            stateServer.imuState.time = featureMsg.time;
        }


        // Propogate the IMU state.
        // that are received before the image msg.
        Log.e("predictNewState", "pos before:" + stateServer.imuState.position);
        batchImuProcessing(featureMsg.time);
        Log.e("predictNewState", "pos after:" + stateServer.imuState.position);

        //assert(featureCallbackDebugCt != 2);
        featureCallbackDebugCt++;


        // Augment the state vector.
        stateAugmentation(featureMsg.time);

        // Add new observations for existing features or new features
        // in the map server.
        addFeatureObservations(featureMsg);

        // Perform measurement update if necessary.
        // And prune features
        removeLostFeatures();

        // Prune camera states.
        removeOldCamStates();

        // Publish the odometry.
        return publish(featureMsg.time);

    }



    private Odometry publish(double time) {
        ImuState imuState = stateServer.imuState;

        Isometry3D Tiw = new Isometry3D(
                quaternionToRotation(imuState.orientation).transpose(),
                imuState.position);
        Isometry3D Tbw = ImuState.tImuBody.mult(Tiw).mult(ImuState.tImuBody.inverse());
        SimpleMatrix bodyVelocity = ImuState.tImuBody.R.mult(imuState.velocity);

        SimpleMatrix Rwc = imuState.rImuCam.mult(Tiw.R.transpose());
        SimpleMatrix tcw = imuState.position.plus(Tiw.R.mult(imuState.tCamImu));
        Isometry3D Tcw = new Isometry3D(Rwc.transpose(), tcw);

        return new Odometry(time, Tbw, bodyVelocity, Tcw);
    }

    private void addFeatureObservations(FeatureMessage featureMsg) {
        // get the current imu state id and number of current features
        int curStateId = stateServer.imuState.id;

        // add all features in the feature_msg to self.map_server
        for (FeatureMeasurement measurement : featureMsg.features) {
            DMatrixRMaj observation = new DMatrixRMaj(new double[]{measurement.u0, measurement.v0});
            if (!stateServer.mapServer.containsKey(measurement.id)) {
                Feature newFeature = new Feature(measurement.id);
                newFeature.observations.put(curStateId,observation);
                stateServer.mapServer.put(newFeature.id, newFeature);

            } else {
                stateServer.mapServer.get(measurement.id).observations.put(curStateId, observation);
            }
        }
    }

    /**
     * Basic sliding window which removes the oldest camera states from our state vector.
     */
    private void removeOldCamStates() {
        if (stateServer.camStates.size() < Config.MAX_CAM_STATES) return;
        // Find two camera states to be removed. Here we choose the oldest two.
        List<Integer> rmCamStateIds = new ArrayList<>(2);
        rmCamStateIds.add(stateServer.camStates.get(0));
        rmCamStateIds.add(stateServer.camStates.get(1));

        // Find the size of the Jacobian matrix.
        int jacobianRowSize = 0;
        for (Feature feature : stateServer.mapServer.values()) {
            List<Integer> involvedCamStateIds = new ArrayList<>(2);
            for (Integer camId : rmCamStateIds) {
                if (feature.observations.containsKey(camId)) involvedCamStateIds.add(camId);
            }

            if (involvedCamStateIds.isEmpty()) continue;

            if (involvedCamStateIds.size() == 1) {
                feature.observations.remove(involvedCamStateIds.get(0));
                continue;
            }

            if (!feature.isInitialized) {
                // Ensure there is enough translation to triangulate the feature
                if (!feature.checkMotion(stateServer.camStates)) {

                    // If the feature cannot be initialized, just remove
                    // the observations associated with the camera states
                    // to be removed.
                    for (Integer camId : involvedCamStateIds) {
                        feature.observations.remove(camId);
                    }
                    continue;
                }

                // Intialize the feature position based on all current available measurements.
                boolean ret = feature.initializePosition(stateServer.camStates);
                if (!ret) {
                    for (Integer camId : involvedCamStateIds) {
                        feature.observations.remove(camId);
                    }
                    continue;
                }
            }
            jacobianRowSize += 2 * involvedCamStateIds.size() - 3;
        }

        SimpleMatrix Hx = new SimpleMatrix(jacobianRowSize, getStateSize());
        SimpleMatrix r = new SimpleMatrix(jacobianRowSize,1);
        int stackCount = 0;
        SimpleMatrix Hxj = new SimpleMatrix(3,getStateSize());
        SimpleMatrix rj = new SimpleMatrix(3,1);


        for (Feature feature : stateServer.mapServer.values()) {
            // Check how many camera states to be removed are associated
            // with this feature
            List<Integer> involvedCamStateIds = new ArrayList<>(2);
            for (Integer camId : rmCamStateIds) {
                if (feature.observations.containsKey(camId)) involvedCamStateIds.add(camId);
            }

            if (involvedCamStateIds.isEmpty()) {
                continue;
            }


            featureJacobian(feature, involvedCamStateIds, Hxj, rj);

            if (gatingTest(Hxj, rj, involvedCamStateIds.size())) {
                Hx.insertIntoThis(stackCount, 0, Hxj);
                r.insertIntoThis(stackCount, 0, rj);
                stackCount += Hxj.getNumRows();
            }

            for (Integer camId : involvedCamStateIds) {
                feature.observations.remove(camId);
            }
        }

        Hx = Hx.rows(0, stackCount);
        r = r.rows(0,stackCount);

        // Perform measurment update.
        measurementUpdate(Hx, r);

        for (Integer camId : rmCamStateIds) {
            int idx = stateServer.camStates.indexOf(camId);
            int camStateStart = StateInfo.IMU_STATE_SIZE + StateInfo.CAM_STATE_SIZE*idx;
            int camStateEnd = camStateStart + 6;

            // Remove the corresponding rows and columns in the state
            // covariance matrix.
            Log.i("removeOldCamStates", String.format("stateCov size: (%d, %d)", stateServer.stateCov.getNumRows(), stateServer.stateCov.getNumCols()));
            SimpleMatrix newCov = deleteColumns(    deleteRows(stateServer.stateCov,camStateStart,camStateEnd)   ,camStateStart,camStateEnd);
            assert(newCov.getNumRows() == newCov.getNumCols());
            assert(newCov.getNumRows() <= 141);
            stateServer.stateCov = newCov;

            // Remove this camera state in the state vector.
            stateServer.camStates.remove(camId);
        }
    }


    private void removeLostFeatures() {
        int jacobianRowSize = 0;
        List<Long> invalidFeatureIds = new LinkedList<>();
        List<Feature> processedFeatures = new LinkedList<>();
        for (Feature feature : stateServer.mapServer.values()) {
            // Pass the features that are still being tracked
            if (feature.observations.containsKey(stateServer.imuState.id)) {
                continue;
            }

            if (feature.observations.size() < 3) {
                invalidFeatureIds.add(feature.id);
                continue;
            }

            if (!feature.isInitialized) {
                // Ensure there is enough translation to triangulate the feature
                if (!feature.checkMotion(stateServer.camStates)) {
                    // If the feature cannot be initialized, just remove
                    // the observations associated with the camera states
                    // to be removed.
                    invalidFeatureIds.add(feature.id);
                    continue;
                }

                // Intialize the feature position based on all current available measurements.
                boolean ret = feature.initializePosition(stateServer.camStates);
                if (!ret) {
                    invalidFeatureIds.add(feature.id);
                    continue;
                }
            }

            // Each feature provides 2 residuals per feature.
            // The -3 comes from the nullspace projection.
            jacobianRowSize += 2 * feature.observations.size() - 3;
            processedFeatures.add(feature);
        }
        // Remove the features that do not have enough measurements.
        for (Long featureId : invalidFeatureIds) {
            stateServer.mapServer.remove(featureId);
        }

        // Return if there is no lost feature to be processed.
        if (processedFeatures.isEmpty()) return;

        SimpleMatrix Hx = new SimpleMatrix(jacobianRowSize, getStateSize());
        SimpleMatrix r = new SimpleMatrix(jacobianRowSize,1);
        int stackCount = 0;
        SimpleMatrix Hxj = new SimpleMatrix(3,getStateSize());
        SimpleMatrix rj = new SimpleMatrix(3,1);

        // Process the features which lose track.
        for (Feature feature : processedFeatures) {
            assert(!feature.observations.isEmpty());
            featureJacobian(feature, feature.observations.asList(), Hxj, rj);

            if (gatingTest(Hxj, rj, feature.observations.size()-1)) {
                Hx.insertIntoThis(stackCount, 0, Hxj);
                r.insertIntoThis(stackCount, 0, rj);
                stackCount += Hxj.getNumRows();
            }
            // Put an upper bound on the row size of measurement Jacobian,
            // which helps guarantee the execution time.
            if (stackCount > 1500) break;
        }
        Hx.reshape(stackCount, Hx.getNumCols());
        r.reshape(stackCount, 1);

        // Perform the measurement update step.
        measurementUpdate(Hx, r);

        // Remove all processed features from the map.
        for (Feature feature : processedFeatures) {
            stateServer.mapServer.remove(feature.id);
        }

    }


    private final LinearSolver<DMatrixRMaj,DMatrixRMaj> denseSolver = new LinearSolverCholLDL_DDRM();
    //private final LinearSolver<DMatrixSparseCSC,DMatrixRMaj> sparseSolver = LinearSolverFactory_DSCC.qr(FillReducing.NONE);
    private final DMatrixRMaj A = new DMatrixRMaj(), X = new DMatrixRMaj();

    @SuppressWarnings("ConstantConditions") // suppress unwanted null pointer warnings
    private boolean gatingTest(SimpleMatrix H, SimpleMatrix r, int dof) {
        DMatrixRMaj P1 = H.mult(stateServer.stateCov).mult(H.transpose()).getDDRM();
        DMatrixRMaj P2 = SimpleMatrix.identity(H.getNumRows()).scale(Config.OBSERVATION_NOISE).getDDRM();
        add(P1,P2,A);

        // Solve linear system
        denseSolver.setA(A);
        denseSolver.solve(r.getDDRM(), X); // X is a column vector

        double gamma = r.transpose().dot(SimpleMatrix.wrap(X));

        return gamma < chiSquaredTestTable.get(dof);
    }

    private void batchImuProcessing(double timeBound) {
        int usedImuMsgCt = 0;
        final String localTag = "PredictNewState";


        synchronized (imuBuffer) {
            for (ImuMessage imuMsg : imuBuffer) {
                if (imuMsg.time < stateServer.imuState.time) {
                    usedImuMsgCt++;
                    continue;
                }
                if (imuMsg.time > timeBound) {
                    break;
                }
                processModel(imuMsg.time, imuMsg.angularVelocity, imuMsg.linearAcceleration);
                usedImuMsgCt++;
            }
        }

        stateServer.imuState.id = ImuState.nextId;
        ImuState.nextId += 1;



        // Remove all used IMU msgs.
        synchronized (imuBuffer) {imuBuffer.subList(0,usedImuMsgCt).clear();}



    }

    public void onlineReset() {
        // TODO.
    }

    private final DMatrixRMaj J = new DMatrixRMaj(StateInfo.CAM_STATE_SIZE, StateInfo.IMU_STATE_SIZE);

    /**
     * Generates a new camera state and adds it to the full state and covariance.
     */
    public void stateAugmentation(double time) {
        // Get the imu_state, rotation from imu to cam, and translation from cam to imu
        SimpleMatrix Ric = stateServer.imuState.rImuCam;
        SimpleMatrix tci = stateServer.imuState.tCamImu;

        // Add a new camera state to the state server.
        SimpleMatrix Rwi = quaternionToRotation(stateServer.imuState.orientation);
        SimpleMatrix Rwc = Ric.mult(Rwi);
        SimpleMatrix tcw = stateServer.imuState.position.plus(Rwi.transpose().mult(tci));
        CamState camState = new CamState(stateServer.imuState.id, time);
        camState.orientation = rotationToQuaternion(Rwc);

        camState.position.setTo(tcw);
        camState.orientationNull.setTo(camState.orientation);

        camState.positionNull.setTo(camState.position);
        stateServer.camStates.put(stateServer.imuState.id, camState);

        // Update the covariance matrix of the state.
        SimpleMatrix J = new SimpleMatrix(StateInfo.CAM_STATE_SIZE, StateInfo.IMU_STATE_SIZE);
        J.insertIntoThis(0,0,Ric);
        J.insertIntoThis(0,15, SimpleMatrix.identity(3));
        J.insertIntoThis(3,0, skewSymmetric(Rwi.transpose().mult(tci)));
        J.insertIntoThis(3,12, SimpleMatrix.identity(3));
        J.insertIntoThis(3,18, Rwi.transpose());

        // Resize the state covariance matrix.
        int oldSize = stateServer.stateCov.getNumRows();
        int newSize = oldSize + 6;
        SimpleMatrix augmentedMatrix = SimpleMatrix.identity(newSize); // TODO: difference Python, C++

        SimpleMatrix P11 = stateServer.stateCov.extractMatrix(0,StateInfo.IMU_STATE_SIZE,0,StateInfo.IMU_STATE_SIZE);

        // Fill in the augmented state covariance
        SimpleMatrix r1 = J.mult(P11);
        augmentedMatrix.insertIntoThis(oldSize, 0, r1);
        augmentedMatrix.insertIntoThis(0,oldSize, r1.transpose());
        augmentedMatrix.insertIntoThis(oldSize,oldSize, J.mult(P11).mult(J.transpose()));

        // Fix the covariance to be symmetric
        stateServer.stateCov = augmentedMatrix.plus(augmentedMatrix.transpose()).scale(0.5);


    }


    /**
     * Given an IMU measurement, propagates the latest camera pose to the timestamp in measurement
     */


    private final QRSparseDecomposition<DMatrixSparseCSC> QR = DecompositionFactory_DSCC.qr(FillReducing.NONE);

    /**
     * Update the state vector given a deltaX computed from a measurement update.
     */
    public void measurementUpdate(SimpleMatrix H, SimpleMatrix r) {


        // Check if H and r are empty
        if (H.getNumElements() == 0 || r.getNumElements() == 0) return;

        // Decompose the final Jacobian matrix to reduce computational complexity.
        SimpleMatrix Hthin, rThin;
        if (H.getNumRows() > H.getNumCols()) {
            // H_sparse
            DMatrixSparseCSC Hsparse = new DMatrixSparseCSC(H.getNumRows(), H.getNumCols());
            Hsparse.setTo(H.getDSCC());

            QR.decompose(Hsparse);
            // TODO: compare with MatLab: https://github.com/utiasSTARS/msckf-swf-comparison/blob/ad9566ef35c3e4792a89b04623e1fa2f99238435/msckf/calcTH.m#L4
            DMatrixSparseCSC Q = QR.getQ(null,true); // Python equivalent: linalg.qr(a, mode='reduced')
            DMatrixSparseCSC R = QR.getR(null,true); // TODO: true or false?

            // TODO: C++ and Python difference.

            // H_thin
            // TODO: why not R.T*
            Hthin = SimpleMatrix.wrap(new DMatrixRMaj(R)); // convert to DMatrixRMaj before wrapping

            // r_thin
            rThin = SimpleMatrix.wrap(new DMatrixRMaj(Q)).transpose().mult(r);
            
        } else {
            Hthin = H;
            rThin = r;
        }

        // Compute the Kalman gain:

        // P
        SimpleMatrix P = stateServer.stateCov;

        // S
        SimpleMatrix S = SimpleMatrix
                .identity(Hthin.getNumRows())
                .scale(Config.OBSERVATION_NOISE)
                .plus(
                        Hthin.mult(P).mult(Hthin.transpose())
                );


        denseSolver.setA(S.getDDRM());
        SimpleMatrix K;
        {
            DMatrixRMaj buffer = new DMatrixRMaj();
            denseSolver.solve(Hthin.mult(P).getDDRM(), buffer);
            transpose(buffer);
            K = SimpleMatrix.wrap(buffer);
        }

        // Compute the error of the state
        SimpleMatrix deltaX = K.mult(rThin); // vector

        // Update the IMU state.
        SimpleMatrix deltaXimu = deltaX.rows(0,StateInfo.IMU_STATE_SIZE);
        SimpleMatrix dqImu = smallAngleQuaternion(deltaXimu.rows(0,3));

        ImuState imuState = stateServer.imuState;
        imuState.orientation = quaternionMultiplication(dqImu, imuState.orientation);
        imuState.gyroBias = imuState.gyroBias.plus(deltaXimu.rows(3,6));
        imuState.velocity = imuState.velocity.plus(deltaXimu.rows(6,9));
        imuState.accBias = imuState.accBias.plus(deltaXimu.rows(9,12));
        imuState.position = imuState.position.plus(deltaXimu.rows(12,15));
        SimpleMatrix dqExtrinsic = smallAngleQuaternion(deltaXimu.rows(15,18));
        stateServer.imuState.rImuCam = quaternionToRotation(dqExtrinsic).mult(stateServer.imuState.rImuCam);
        stateServer.imuState.tCamImu = stateServer.imuState.tCamImu.plus(deltaXimu.rows(18,StateInfo.IMU_STATE_SIZE));


        // Update the camera states.
        for (Map.Entry<Integer,CamState> entry : stateServer.camStates.entrySet()) {
            CamState camState = entry.getValue();
            SimpleMatrix deltaXcam = deltaX.rows(
                    StateInfo.IMU_STATE_SIZE + entry.getKey() * StateInfo.CAM_STATE_SIZE,
                    (StateInfo.IMU_STATE_SIZE + StateInfo.CAM_STATE_SIZE) + (entry.getKey() * StateInfo.CAM_STATE_SIZE));
            SimpleMatrix dqCam = smallAngleQuaternion(deltaXcam.rows(0,3));
            camState.orientation = quaternionMultiplication(dqCam, camState.orientation);
            camState.position = camState.position.plus(deltaXcam.rows(3,SimpleMatrix.END));
        }

        // Update state covariance
        SimpleMatrix IKH = SimpleMatrix
                .identity(K.getNumRows())
                .minus(
                    K.mult(Hthin));
        SimpleMatrix stateCov = IKH.mult(stateServer.stateCov);

        // Fix the covariance to be symmetric
        stateServer.stateCov = stateCov.plus(stateCov.transpose()).scale(0.5);
    }


    /**
     * Compute the jacobian and the residual of a 3D point.
     * Modifies Hx, r.
     */
    private void featureJacobian(Feature feature, List<Integer> camStateIds, SimpleMatrix Hx, SimpleMatrix r) {
        int jacobianRowSize = 2 * camStateIds.size();
        SimpleMatrix Hxj = new SimpleMatrix(jacobianRowSize, getStateSize());
        SimpleMatrix Hfj = new SimpleMatrix(jacobianRowSize, 3);
        SimpleMatrix rj = new SimpleMatrix(jacobianRowSize,1);

        SimpleMatrix Hxi = new SimpleMatrix(2,6);
        SimpleMatrix Hfi = new SimpleMatrix(2,3);
        SimpleMatrix ri = new SimpleMatrix(2,1);

        int stackCount = 0;
        for (Integer camId : camStateIds) {
            measurementJacobian(camId, feature, Hxi, Hfi, ri);

            // Stack the Jacobians.
            int idx = stateServer.camStates.indexOf(camId);
            Hxj.insertIntoThis(stackCount, StateInfo.IMU_STATE_SIZE + StateInfo.CAM_STATE_SIZE * idx, Hxi);
            Hfj.insertIntoThis(stackCount,0,Hfi);
            rj.insertIntoThis(stackCount,0,ri);
            stackCount += 2;
        }

        // Project the residual and Jacobians onto the nullspace of H_fj.
        // svd of H_fj
        SimpleMatrix U = Hfj.svd().getU(); // Shape (m x m), m = Hfj.numRows = jacobianRowSize
        SimpleMatrix A = U.cols(jacobianRowSize - 3, SimpleMatrix.END); // Shape (m, 3)

        Hx.setTo(A.transpose().mult(Hxj));
        r.setTo(A.transpose().mult(rj));
    }

    /**
     * This function is used to compute the measurement Jacobian
     * for a single feature observed at a single camera frame.
     * Modifies Hx, Hf, r.
     */
    @SuppressWarnings("UnnecessaryLocalVariable")
    private void measurementJacobian(Integer camStateId, Feature feature, SimpleMatrix Hx, SimpleMatrix Hf, SimpleMatrix r) {
        Log.i("measurementJacobian", "METHOD CALL!");
        CamState camState = stateServer.camStates.get(camStateId);
        assert(camState != null);

        // Cam pose.
        SimpleMatrix Rwc = quaternionToRotation(camState.orientation);
        SimpleMatrix tcw = camState.position;

        // 3d feature position in the world frame.
        // And its observation with the stereo camera
        assert(feature.isInitialized);
        SimpleMatrix pw = feature.position;
        SimpleMatrix z = SimpleMatrix.wrap(feature.observations.get(camStateId));

        // Convert the feature position from the world frame to the cam frame.
        SimpleMatrix pc = Rwc.mult(pw.minus(tcw));

        // Compute the Jacobians.
        SimpleMatrix dzDpc = new SimpleMatrix(2,3);
        dzDpc.set(0,0, 1/pc.get(2));
        dzDpc.set(1,1, 1/pc.get(2));
        dzDpc.set(0,2, -pc.get(0) / (pc.get(2) * pc.get(2)));
        dzDpc.set(1,2, -pc.get(1) / (pc.get(2) * pc.get(2)));

        SimpleMatrix dpcDxc = new SimpleMatrix(3,6);
        dpcDxc.insertIntoThis(0,0, skewSymmetric(pc));
        dpcDxc.insertIntoThis(0,3, Rwc.negative());

        SimpleMatrix dpcDpg = Rwc;

        Hx.setTo(dzDpc.mult(dpcDxc));
        Hf.setTo(dzDpc.mult(dpcDpg));

        // Modify the measurement Jacobian to ensure observability constraint.
        SimpleMatrix A = Hx;
        SimpleMatrix u = new SimpleMatrix(6,1);
        u.insertIntoThis(0,0, quaternionToRotation(camState.orientationNull).mult(ImuState.GRAVITY));
        u.insertIntoThis(3,0, skewSymmetric(pw.minus(camState.positionNull)).mult(ImuState.GRAVITY));

        Log.d("measurementJacobian", "A=" + A + ", u=" + u + "Hx=" + Hx);
        Hx.setTo(A
                .minus(
                        A.mult(u)
                                .mult(u
                                        .transpose()
                                        .mult(u)
                                        .invert())
                                .mult(u
                                        .transpose())));
        Hf.setTo(Hx
                .extractMatrix(0,SimpleMatrix.END,3,SimpleMatrix.END)
                .negative());

        // Compute the residual
        r.setTo(z.minus(new SimpleMatrix(new double[]{
                pc.get(0) / pc.get(2),
                pc.get(1) / pc.get(2)
        })));

    }


    public SimpleMatrix calcOmega(SimpleMatrix gyro) {
        SimpleMatrix omega = new SimpleMatrix(4,4);
        omega.insertIntoThis(0,0,skewSymmetric(gyro).negative());
        omega.insertIntoThis(3,0,gyro.transpose().negative());
        omega.insertIntoThis(0,3,gyro);
        return omega;
    }


    public void predictNewState(double dt, SimpleMatrix gyro, SimpleMatrix acc) {
        double gyroNorm = gyro.normF();
        SimpleMatrix omega = calcOmega(gyro);

        final String localTag = "predictNewState";


        SimpleMatrix q = stateServer.imuState.orientation;
        SimpleMatrix v = stateServer.imuState.velocity;
        SimpleMatrix p = stateServer.imuState.position;


        SimpleMatrix dqDt, dqDt2;
        if (gyroNorm > 0.00005) {
            dqDt = SimpleMatrix
                    .identity(4)
                    .scale(cos(gyroNorm * dt * 0.5))
                    .plus(omega.scale(1 / gyroNorm * sin(gyroNorm * dt * 0.5)))
                    .mult(q);

            dqDt2 = SimpleMatrix
                    .identity(4)
                    .scale(cos(gyroNorm * dt * 0.25))
                    .plus(omega.scale(1 / gyroNorm * sin(gyroNorm * dt * 0.25)))
                    .mult(q);
        } else {
            dqDt = SimpleMatrix
                    .identity(4)
                    .plus(omega.scale(0.5 * dt))
                    .scale(cos(gyroNorm * dt * 0.5))
                    .mult(q);

            dqDt2 = SimpleMatrix
                    .identity(4)
                    .plus(omega.scale(0.25 * dt))
                    .scale(cos(gyroNorm * dt * 0.25))
                    .mult(q);
        }
        SimpleMatrix drDtTranspose = quaternionToRotation(dqDt).transpose();
        SimpleMatrix drDt2Transpose = quaternionToRotation(dqDt2).transpose();

        // Apply 4th order Runge-Kutta
        // k1 = f(tn, yn)
        SimpleMatrix k1vDot = quaternionToRotation(q)
                .transpose()
                .mult(acc)
                .plus(ImuState.GRAVITY);
        SimpleMatrix k1pDot = v;


        // k2 = f(tn+dt/2, yn+k1*dt/2)
        SimpleMatrix k1v = v.plus(k1vDot.scale(dt).divide(2));
        SimpleMatrix k2vDot = drDt2Transpose.mult(acc).plus(ImuState.GRAVITY);
        SimpleMatrix k2pDot = k1v;

        // k3 = f(tn+dt/2, yn+k2*dt/2)
        SimpleMatrix k2v = v.plus(k2vDot.scale(dt).divide(2));
        SimpleMatrix k3vDot = drDt2Transpose.mult(acc).plus(ImuState.GRAVITY);
        SimpleMatrix k3pDot = k2v;

        // k4 = f(tn+dt, yn+k3*dt)
        SimpleMatrix k3v = v.plus(k3vDot.scale(dt));
        SimpleMatrix k4vDot = drDtTranspose.mult(acc).plus(ImuState.GRAVITY);
        SimpleMatrix k4pDot = k3v;

        // yn+1 = yn + dt/6*(k1+2*k2+2*k3+k4)
        q = quaternionNormalize(dqDt);


        // update the imu state
        v = v.plus(
                    k1vDot
                    .plus(k2vDot.scale(2))
                    .plus(k3vDot.scale(2))
                    .plus(k4vDot)
                    .scale(dt/6));
        p = p.plus(k1pDot.plus(k2pDot.scale(2)).plus(k3pDot.scale(2)).plus(k4pDot).scale(dt/6));

        stateServer.imuState.orientation = q;
        stateServer.imuState.position = p;


        stateServer.imuState.velocity = v;
        debugCt++;

    }

    private int debugCt = 0;



}

