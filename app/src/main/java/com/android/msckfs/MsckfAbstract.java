package com.android.msckfs;
import static com.android.msckfs.utils.MathUtils.deleteColumns;
import static com.android.msckfs.utils.MathUtils.deletedRows;
import static com.android.msckfs.utils.MathUtils.quaternionToRotation;
import static com.android.msckfs.utils.MathUtils.quaternionToVector;
import static com.android.msckfs.utils.MathUtils.rotationToQuaternion;
import static com.android.msckfs.utils.MathUtils.skewSymmetric;
import static com.android.msckfs.utils.MathUtils.smallAngleQuaternion;
import static com.android.msckfs.utils.MathUtils.vectorToQuaternion;
import static org.ejml.dense.row.CommonOps_DDRM.add;
import static org.ejml.dense.row.CommonOps_DDRM.identity;
import static org.ejml.dense.row.CommonOps_DDRM.insert;
import static org.ejml.dense.row.CommonOps_DDRM.mult;
import static org.ejml.dense.row.CommonOps_DDRM.scale;
import static org.ejml.dense.row.CommonOps_DDRM.setIdentity;
import static org.ejml.dense.row.CommonOps_DDRM.solve;
import static org.ejml.dense.row.CommonOps_DDRM.transpose;
import static org.ejml.dense.row.NormOps_DDRM.normP2;

import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import org.apache.commons.numbers.quaternion.Quaternion;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.data.MatrixSparse;
import org.ejml.data.MatrixType;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.chol.CholeskyDecompositionLDL_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.dense.row.linsol.chol.LinearSolverCholLDL_DDRM;
import org.ejml.interfaces.decomposition.CholeskyLDLDecomposition_F64;
import org.ejml.interfaces.decomposition.QRDecomposition;
import org.ejml.interfaces.decomposition.QRSparseDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.DConvertMatrixStruct;
import org.ejml.simple.SimpleMatrix;
import org.ejml.sparse.FillReducing;
import org.ejml.sparse.csc.CommonOps_DSCC;
import org.ejml.sparse.csc.decomposition.qr.QrLeftLookingDecomposition_DSCC;
import org.ejml.sparse.csc.factory.DecompositionFactory_DSCC;
import org.ejml.sparse.csc.factory.LinearSolverFactory_DSCC;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * https://github.com/Edwinem/msckf_tutorial/blob/main/src/msckf.py
 * https://ejml.org/wiki/index.php?title=Example_Kalman_Filter
 * https://github.com/daniilidis-group/msckf_mono/blob/master/include/msckf_mono/msckf.h
 * // TODO: cite paper
 */
public abstract class MsckfAbstract implements Msckf {

    // TODO: MSCKF State:mu_state cam_states imu_cov cam_cov imucam_cov =====
    private List<ImuMessage> imuBuffer; // TODO: initialize

    private StateServer stateServer;


    private final DMatrixRMaj F, G;



    // Matrix calculation buffers
    private DMatrixRMaj f, g;
    private final DMatrixRMaj Fdt, FdtSquare, FdtCube;

    public MsckfAbstract() {
        F = new DMatrixRMaj(15,15);
        G = new DMatrixRMaj(15,12);



    }

    // TODO: calcF and calcG have differently placed submatrices in MSCKF tutorial vs. official MSCKF-S impl.
        // In suspect that's bc. each 3x3 stands for a diff. attribute (pos, vel, att, bias, ...), the positions of which are interchangable, as long as constistent
    /**
     * Computes the transition matrix F for the EKF.
     * The transition matrix contains the jacobians of our process model with respect to our current state.
     */
    public void calcF(DMatrixRMaj gyro, DMatrixRMaj acc) {
        F.zero();

        // f = -skew(gyro)
        f = skewSymmetric(gyro);
        scale(-1,f);
        insert(f,F,0,0);

        // f = eye(3)
        setIdentity(f);
        insert(f,F,12,6);

        // f = -eye(3)
        scale(-1,f);
        insert(f,F,0,3);

        // f = -to_rotation(imu_state.orientation).T
        f = quaternionToRotation(stateServer.imuState.orientation);
        transpose(f);
        scale(-1,f);
        insert(f,F,6,9);

        // f = -to_rotation(imu_state.orientation).T * skew(acc)
        f = mult(f,skewSymmetric(acc), new DMatrixRMaj());
        insert(f,F,6,0);

        // TODO: F in MSCKF-S Python does not look like in Paper? smth. should be at 6-6? AND 6--12?

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
        g = quaternionToRotation(stateServer.imuState.orientation);
        transpose(g);
        scale(-1,g);
        insert(g,G,6,6);

    }


    private static class Config {
        static final int IMU_STATE_SIZE = 15; // TODO: why in tutorial sometimes hardcoded, sometimes not?

        static final double OBSERVATION_NOISE = pow(0.035d, 2);
        static final int MAX_CAM_STATES = 20;
    }

    abstract void featuresLocalization();


    // TODO: call for every ImuMessage in buffer, like in Python tutorial?
    /**
     *
     */
    public void processModel(double time, DMatrixRMaj mGyro, DMatrixRMaj mAcc) {

        ImuState imuState = stateServer.imuState;
        double dt = time - imuState.timestamp;

        // TODO: remove gyro/acc bias, line in MSCKF-S?
        calcF(imu.angularVelocity, imu.linearAcceleration);
        calcG();
        integrate();

        // Approximate matrix exponential to the 3rd order, which can be
        // considered to be accurate enough assuming dt is within 0.01s
        scale(dt, F, Fdt);
        mult(Fdt,Fdt,FdtSquare);
        mult(FdtSquare,Fdt,FdtCube);

        // Phi = np.eye(21) + Fdt + 0.5*Fdt_square + (1./6.)*Fdt_cube
        SimpleMatrix Phi = SimpleMatrix
                .identity(15)
                .plus(SimpleMatrix.wrap(Fdt))
                .plus(SimpleMatrix.wrap(FdtSquare).scale(0.5))
                .plus(SimpleMatrix.wrap(FdtCube).scale(1./6.))
                .getMatrix();

        // Propogate the state covariance matrix.
        SimpleMatrix Gwrapped = SimpleMatrix.wrap(G); // TODO: maybe change F and G to SimpleMatrix?
        SimpleMatrix FWrapped = SimpleMatrix.wrap(G);
        SimpleMatrix Q = Phi
                .mult(Gwrapped)
                .mult(stateServer.covariance)
                .mult(Gwrapped.transpose())
                .mult(Phi.transpose())
                .scale(dt);

        // TODO: add observability constraints?

        // TODO: if len(self.state_server.cam_states) > 0:

        // Update the imu-camera covariance
        stateServer.covariance.setColumn(15,Phi.mult(stateServer.covariance.getColumn(15)));
        stateServer.covariance.setRow(15, stateServer.covariance.getRow(15).mult(Phi.transpose()));

        // Fix the covariance to be symmetric
        stateServer.covariance = stateServer.covariance.plus(stateServer.covariance.transpose()).scale(0.5);


        // Update the state correspondes to null space.
        // TODO?: stateServer.imuState.orientationNull
    }

    public void imuCallback(ImuMessage imuMsg) {
        imuBuffer.add(imuMsg);

        // TODO: (only) MSCKF-S has additional function here that initializes gravity
    }

    private boolean isFirstImage = true;
    public void featureCallback(FeatureMessage featureMsg) {

        // TODO: Tutorial does something else when "first time"?
        if (isFirstImage) {
            isFirstImage = false;
            stateServer.imuState.timestamp = featureMsg.timestamp;
        }

        // Propogate the IMU state.
        // that are received before the image msg.
        batchImuProcessing(featureMsg.timestamp);

        stateAugmentation(featureMsg.timestamp);

        addFeatureObservations(featureMsg);

        // TODO: what is tutorial equivalent for MSCKF-S remove_lost_features and prune_cam_
        // Perform measurement update if necessary.
        // And prune features
        removeLostFeatures();

        // Prune camera states.
        removeOldCamStates();

        // TODO: ...

    }

    /**
     * Basic sliding window which removes the oldest camera state from our state vector.
     */
    private void removeOldCamStates() {
        // TODO: compare remove_old_clones (tutorial) to prune_cam_state_buffer
        if (stateServer.camStates.size() < Config.MAX_CAM_STATES) return;

        List<Integer> idsToRemove = new ArrayList<>();
        int oldestCamId = stateServer.camStates.keySet().stream().min(Integer::compareTo).get();

        // Run the MSCKF update on any features which have this camera state.
        for (Map.Entry<Integer,Feature> entry : stateServer.mapServer.entrySet()) {
            if (entry.getValue().cameraIds.get(0).equals(oldestCamId)) {
                idsToRemove.add(entry.getKey());
            }
        }

        msckf_update(); // TODO

        // Remove the camera states from the state vector
        int camStateStart = 15;
        int camStateEnd = camStateStart + 6;

        SimpleMatrix newCov = deleteColumns(    deletedRows(stateServer.covariance,camStateStart,camStateEnd)   ,camStateStart,camStateEnd);
        assert(newCov.getNumRows() == newCov.getNumCols());
        stateServer.covariance = newCov;

        stateServer.camStates.remove(oldestCamId);

    }


    // TODO: when/how to triangulate the features?

    /**
     *
     * The main purpose of this function is to validate the tracks for the measurement update.
     */
    private void triangulateValidFeatures(List<Feature> features) {
        // TODO: compare msckf_update/update_with_good_ids (tutorial) with remove_lost_features/prune_cam_state_buffer (MSCKF-S Python)

        if (features.isEmpty()) return;

        List<Feature> triangulatedFeatures = new ArrayList<>(features.size());

        // TODO: ...
        for (Feature feature : features) {
            // TODO: ...
            if (!feature.isInitialized) {
                if (!feature.checkMotion(stateServer.camStates)) {
                    // TODO: ...
                    continue;
                }
                boolean ret = feature.initializePosition(stateServer.camStates);
                if (!ret) {
                    // TODO: ...
                    continue;
                }
                // TODO: ...
            }
            triangulatedFeatures.add(feature); // TODO: should I really use a feature that may already have been used in a past iteration?
            // TODO: ..
        }
        // TODO: (skipped some stuff from prune_cam_state_buffer here)


    }

    private void updateWithTriangulatedFeatures(Map<Integer, Point2D> triangulatedFeatures) {
        // TODO: compare update_with_good_ids with remove_lost_features/prune_cam_state_buffer (MSCKF-S)
        if (triangulatedFeatures.isEmpty()) return;

        // TODO: ...
        SimpleMatrix H = new SimpleMatrix(maxPossibleSize, stateServer.)
    }
    private void msckfUpdate() {
        // TODO: MSCKF-S equivalent?
    }
    private void removeLostFeatures() {
        // TODO: is pruning features even in the tutorial?
        // TODO: compare compute_residual_and_jacobian (tutorial) to remove_lost_features
    }

    private void addFeatureObservations(FeatureMessage featureMsg) {
        // TODO: compare add_camera_features (tutorial) to add_feature_observations (MSCKF-S)
    }

    private final LinearSolver<DMatrixRMaj,DMatrixRMaj> denseSolver = new LinearSolverCholLDL_DDRM();
    //private final LinearSolver<DMatrixSparseCSC,DMatrixRMaj> sparseSolver = LinearSolverFactory_DSCC.qr(FillReducing.NONE);
    private final DMatrixRMaj A, X;
    private boolean gatingTest(SimpleMatrix H, SimpleMatrix r, int dof) { // TODO: doch nicht SimpleMatrix?
        DMatrixRMaj P1 = H.mult(stateServer.covariance).mult(H.transpose()).getDDRM();
        DMatrixRMaj P2 = SimpleMatrix.identity(H.getNumRows()).scale(Config.OBSERVATION_NOISE).getDDRM();
        add(P1,P2,A);

        // Solve linear system
        solver.setA(A);
        solver.solve(r.getDDRM(), X); // X is a column vector
        assert(!solver.modifiesB()); // TODO: move assert to constructor.
        double gamma = r.transpose().dot(SimpleMatrix.wrap(X));
        // TODO: are there other calculations with vectors where a different type of calculation (e.g. dot product instead of matrix product) was required?

    }

    private void batchImuProcessing(double timeBound) {
        int usedImuMsgCt = 0;

        // TODO: Synchronisierung imuBuffer?
        for (ImuMessage imuMsg : imuBuffer) {
            if (imuMsg.timestamp < stateServer.imuState.timestamp) {
                usedImuMsgCt++;
                continue;
            }
            if (imuMsg.timestamp > timeBound) break;
            processModel(imuMsg.timestamp, imuMsg.angularVelocity, imuMsg.linearAcceleration);
            usedImuMsgCt++;
        }

        stateServer.imuState.id = ImuState.nextId;
        ImuState.nextId += 1;

        // Remove all used IMU msgs.
        imuBuffer.subList(0,usedImuMsgCt).clear();

    }

    private final DMatrixRMaj J;

    /**
     * Generates a new camera state and adds it to the full state and covariance.
     */
    public void stateAugmentation(long timestamp) {
        // Get the imu_state, rotation from imu to cam0, and translation from cam0 to imu
        SimpleMatrix Ric = stateServer.imuState.rImuCam;
        SimpleMatrix tci = stateServer.imuState.tCamImu;

        // Add a new camera state to the state server.
        SimpleMatrix Rwi = SimpleMatrix.wrap(quaternionToRotation(stateServer.imuState.orientation));
        SimpleMatrix Rwc = Ric.mult(Rwi);
        SimpleMatrix tcw = stateServer.imuState.position.plus(Rwi.transpose().mult(tci));
        CamState camState = new CamState(stateServer.imuState.id, timestamp);
        camState.orientation = rotationToQuaternion(Rwc.getDDRM());
        camState.position.setTo(tcw);
        camState.orientationNull = camState.orientation;
        camState.positionNull.setTo(camState.position);
        stateServer.camStates.put(stateServer.imuState.id, camState);

        // Update the covariance matrix of the state.
        J.zero();
        insert(Ric.getDDRM(),J,0,0);
        insert(identity(3), J,0,15);
        insert(skewSymmetric(Rwi.transpose().mult(tci).getDDRM()), J,3,0); // TODO: different in tutorial and MSCKF-S (Python)
        insert(identity(3),J,3,12);
        insert(Rwi.transpose().getDDRM(),J,3,18);

        // Resize the state covariance matrix.
        int oldSize = stateServer.covariance.getNumRows();
        int newSize = oldSize + 6;
        SimpleMatrix augmentationMatrix = SimpleMatrix.wrap(identity(newSize,oldSize));
        augmentationMatrix.insertIntoThis(oldSize,0,SimpleMatrix.wrap(J));

        SimpleMatrix newCovariance = augmentationMatrix.mult(stateServer.covariance).mult(augmentationMatrix.transpose());

        // Fix the covariance to be symmetric
        stateServer.covariance = newCovariance.plus(newCovariance.transpose()).scale(0.5);


    }

    private void update(List<Integer> trackedFeatureIds) {
        // TODO: ...

        // TODO: why do we iterate over the ids, an not over features? Memory efficiency reasons?
        for (Integer featureId : trackedFeatureIds) {}

        // TODO: ...
    }

    /**
     * Given an IMU measurement, propagates the latest camera pose to the timestamp in measurement
     */


    private final QRSparseDecomposition<DMatrixSparseCSC> QR = DecompositionFactory_DSCC.qr(FillReducing.NONE);

    /**
     * Update the state vector given a deltaX computed from a measurement update.
     */
    public void kalmanUpdate(List<Integer> featureIds, DMatrixRMaj H, SimpleMatrix r) {
        // TODO:  Check if H and r are empty?

        // Decompose the final Jacobian matrix to reduce computational complexity.
        SimpleMatrix Hthin, rThin;
        if (H.getNumRows() > H.getNumCols()) {
            // H_sparse
            DMatrixSparseCSC Hsparse = new DMatrixSparseCSC(H.numRows, H.numCols);
            Hsparse.setTo(H);

            QR.decompose(Hsparse);
            DMatrixSparseCSC Q = QR.getQ(null,true); // Python equivalent: linalg.qr(a, mode='reduced')
            DMatrixSparseCSC R = QR.getR(null,TODO); // TODO
            // TODO: everything else in this bracket logically correct?

            // H_thin
            Hthin = SimpleMatrix.wrap(new DMatrixRMaj(R)); // convert to DMatrixRMaj before wrapping

            // r_thin
            rThin = SimpleMatrix.wrap(new DMatrixRMaj(Q)).transpose().mult(r);
            
        } else {
            Hthin = SimpleMatrix.wrap(H);
            rThin = r;
        }

        // Compute the Kalman gain:

        // P
        SimpleMatrix P = stateServer.covariance;

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
        SimpleMatrix deltaX = K.mult(rThin);

        // Update the IMU state.
        SimpleMatrix deltaXimu = deltaX.rows(0,15); // TODO: is 15 correct? In general, but allso with rows() specificially, or do I need 15+1
        Quaternion dqImu = smallAngleQuaternion(deltaXimu.rows(0,3)); // TODO: same question here ^^

        ImuState imuState = stateServer.imuState;
        imuState.orientation = dqImu.multiply(imuState.orientation);
        imuState.gyroBias = imuState.gyroBias.plus(deltaXimu.rows(3,6));
        imuState.velocity = imuState.velocity.plus(deltaXimu.rows(6,9));
        imuState.accBias = imuState.accBias.plus(deltaXimu.rows(9,12));
        imuState.position = imuState.position.plus(deltaXimu.rows(12,15));
        // TODO: dq_extrinsic = small_angle_quaternion(delta_x_imu[15:18])
        // TODO: what do I do, since mine only size 15? Is it even supposed to be 15?

        // TODO: ...
        for (Map.Entry<Integer,CamState> entry : stateServer.camStates.entrySet()) {
            CamState camState = entry.getValue();
            SimpleMatrix deltaXcam = deltaX.rows(15 + entry.getKey() * 6, 21 + entry.getKey() * 6); // TODO: are these coords accurate for mono? Where does the 27 in MSCKF-S here come from? Smth. about 21+6?
            Quaternion dqCam = smallAngleQuaternion(deltaXcam.rows(0,3));
            camState.orientation = dqCam.multiply(camState.orientation);
            camState.position = camState.position.plus(deltaXcam.rows(3,deltaXcam.getNumRows()));
        }

        // Update state covariance
        SimpleMatrix IKH = SimpleMatrix
                .identity(K.getNumRows())
                .minus(
                    K.mult(Hthin));
        SimpleMatrix stateCov = IKH.mult(stateServer.covariance);

        // Fix the covariance to be symmetric
        stateServer.covariance = stateCov.plus(stateCov.transpose()).scale(0.5);

    }

    public void triangulate() { //TODO: MATLAB and MSCKF Tutorial, input params?

    }

    /**
     * Compute the jacobian and the residual of a 3D point.
     */
    abstract void calcResAndJ();


    abstract void calcPrHAndRes();

    abstract void computeResidualAndJacobian();



    public SimpleMatrix calcOmega(DMatrixRMaj gyro) {
        SimpleMatrix omega = SimpleMatrix.identity(4);
        omega.insertIntoThis(0,0, SimpleMatrix.wrap(skewSymmetric(gyro)).scale(-1));
        omega.insertIntoThis(3,0,SimpleMatrix.wrap(gyro).scale(-1));
        omega.insertIntoThis(0,3,SimpleMatrix.wrap(gyro));
        return omega;
    }
    public void integrate(double dt, DMatrixRMaj gyro, DMatrixRMaj acc) {
        double gyroNorm = normP2(gyro);
        SimpleMatrix omega = calcOmega(gyro);

        Quaternion q = stateServer.imuState.orientation;
        SimpleMatrix v = SimpleMatrix.wrap(stateServer.imuState.velocity);
        SimpleMatrix p = SimpleMatrix.wrap(stateServer.imuState.position);

        Quaternion dqDt, dqDt2;
        SimpleMatrix qVec = quaternionToVector(q);
        if (gyroNorm > 0.00005) {
            dqDt = vectorToQuaternion(SimpleMatrix
                    .identity(4)
                    .scale(cos(gyroNorm * dt * 0.5))
                    .plus(omega.scale(1 / gyroNorm * sin(gyroNorm * dt * 0.5)))
                    .mult(qVec)
            );

            dqDt2 = vectorToQuaternion(SimpleMatrix
                    .identity(4)
                    .scale(cos(gyroNorm * dt * 0.25))
                    .plus(omega.scale(1 / gyroNorm * sin(gyroNorm * dt * 0.25)))
                    .mult(qVec)
            );
        } else {
            dqDt = vectorToQuaternion(SimpleMatrix
                    .identity(4)
                    .plus(omega.scale(0.5 * dt))
                    .scale(cos(gyroNorm * dt * 0.5))
                    .mult(qVec)
            );

            dqDt2 = vectorToQuaternion(SimpleMatrix
                    .identity(4)
                    .plus(omega.scale(0.25 * dt))
                    .scale(cos(gyroNorm * dt * 0.25))
                    .mult(qVec)
            );
        }
        SimpleMatrix drDtTranspose = SimpleMatrix.wrap(quaternionToRotation(dqDt)).transpose();
        SimpleMatrix drDt2Transpose = SimpleMatrix.wrap(quaternionToRotation(dqDt2)).transpose();

        // Apply 4th order Runge-Kutta
        // k1 = f(tn, yn)
        SimpleMatrix accWrapped = SimpleMatrix.wrap(acc);
        SimpleMatrix gravityWrapped = SimpleMatrix.wrap(ImuState.GRAVITY);
        // TODO: check if correct copied
        SimpleMatrix k1vDot = SimpleMatrix.wrap(
                quaternionToRotation(q))
                .transpose().mult(accWrapped)
                .plus(gravityWrapped);
        SimpleMatrix k1pDot = v;

        // k2 = f(tn+dt/2, yn+k1*dt/2)
        SimpleMatrix k1v = v.plus(k1vDot.scale(dt/2));
        SimpleMatrix k2vDot = drDt2Transpose.mult(accWrapped).plus(gravityWrapped);
        SimpleMatrix k2pDot = k1v;

        // k3 = f(tn+dt/2, yn+k2*dt/2)
        SimpleMatrix k2v = v.plus(k2vDot).scale(dt/2);
        SimpleMatrix k3vDot = drDt2Transpose.mult(accWrapped).plus(gravityWrapped);
        SimpleMatrix k3pDot = k2v;

        // k4 = f(tn+dt, yn+k3*dt)
        SimpleMatrix k3v = v.plus(k3vDot.scale(dt));
        SimpleMatrix k4vDot = drDtTranspose.mult(accWrapped).plus(gravityWrapped);
        SimpleMatrix k4pDot = k3v;

        // yn+1 = yn + dt/6*(k1+2*k2+2*k3+k4)#
        q = dqDt.normalize();

        // update the imu state
        v = v.plus(k1vDot.plus(k2vDot.scale(2)).plus(k3vDot.scale(2)).plus(k4vDot).scale(dt/6));
        p = p.plus(k1pDot.plus(k2pDot.scale(2)).plus(k3pDot.scale(2)).plus(k4pDot).scale(dt/6));

        stateServer.imuState.orientation = q;
        stateServer.imuState.position = v.getMatrix();
        stateServer.imuState.velocity = p.getMatrix();

    }



}
