package com.android.msckfs;

import static com.android.msckfs.utils.MathUtils.quaternionToRotation;

import static org.ejml.dense.row.CommonOps_DDRM.add;
import static org.ejml.dense.row.CommonOps_DDRM.elementMult;
import static org.ejml.dense.row.CommonOps_DDRM.elementSum;
import static org.ejml.dense.row.CommonOps_DDRM.identity;
import static org.ejml.dense.row.CommonOps_DDRM.insert;
import static org.ejml.dense.row.CommonOps_DDRM.mult;
import static org.ejml.dense.row.CommonOps_DDRM.scale;
import static org.ejml.dense.row.CommonOps_DDRM.transpose;
import static org.ejml.dense.row.CommonOps_DDRM.subtract;
import static org.ejml.dense.row.NormOps_DDRM.normP2;


import static java.lang.Double.max;
import static java.lang.Double.min;

import com.android.msckfs.utils.MathUtils;

import org.apache.commons.numbers.quaternion.Quaternion;
import org.ddogleg.struct.Tuple2;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.linsol.chol.LinearSolverCholLDL_DDRM;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * https://github.com/Edwinem/msckf_tutorial/blob/main/src/msckf_types.py
 * https://github.com/rohiitb/msckf_vio_python/blob/main/feature.py
 * https://github.com/daniilidis-group/msckf_mono/blob/d51c9eef620b001a4a7014dd027fc0e2486b5cd6/include/msckf_mono/msckf.h#L1126
 */
public class Feature {

    public final int id;

    // TODO: add observations to list.
    // TODO: do observations get REMOVED? So that list doesn't grow infinitely?

    public boolean isInitialized = false;
    /**
     * Associates IDs of camera states in which the features has been observed and the
     * Image coordinates at which the feature has been observed. Inserted chronologically.
     */
    public final List<  Tuple2<Integer,DMatrixRMaj>    > observations;
   //  public List<Integer> cameraIds = new ArrayList<>(); // IDs of camera states in which the features has been observed. Inserted chronologically.

    private final LinearSolver<DMatrixRMaj,DMatrixRMaj> denseSolver = new LinearSolverCholLDL_DDRM();

    private final DMatrixRMaj position = new DMatrixRMaj();
    public Feature(int id, Integer cameraId) {
        this.observations = new LinkedList<>();
        this.id = id;
        this.cameraIds.add(cameraId);
    }

    public boolean checkMotion(Map<Integer, CamState> camStates) {
        if (observations.size() < 2) return false;

        Integer firstId = observations.get(0).d0;
        Integer lastId = observations.get(observations.size()-1).d0;


        Isometry3D firstCamPose = new Isometry3D(
                transpose(quaternionToRotation(camStates.get(firstId).orientation), null),
                        camStates.get(firstId).position);

        Isometry3D lastCamPose = new Isometry3D(
                transpose(quaternionToRotation(camStates.get(lastId).orientation), null),
                camStates.get(lastId).position);

        // Get the direction of the feature when it is first observed.
        // This direction is represented in the world frame.
        DMatrixRMaj featureDirection = new DMatrixRMaj(new double[] {observations.get(0).d1.getX(), observations.get(0).d1.getY(), 1.0});
        scale((1.0 / normP2(featureDirection)), featureDirection);
        featureDirection = mult(firstCamPose.R, featureDirection, null);

        // Compute the translation between the first frame and the last frame.
        // We assume the first frame and the last frame will provide the
        // largest motion to speed up the checking process
        DMatrixRMaj translation = subtract(lastCamPose.t, firstCamPose.t, null);
        DMatrixRMaj parallel = mult(translation, featureDirection,null);
        DMatrixRMaj orthogonalTranslation = subtract(
                translation,
                mult(parallel, featureDirection,null), null); // vector

        return (normP2(orthogonalTranslation) > Config.TRANSLATION_THRESHOLD);
    }

    private static class Config {
        static final double TRANSLATION_THRESHOLD = 0.2; // 0.05? 0.4?
        static final double INITIAL_DAMPING = 1e-3;

        static final int OUTER_LOOP_MAX_ITERATION = 5; // 10
        static final int INNER_LOOP_MAX_ITERATION = 5; // 10

        static final double ESTIMATION_PRECISION = 5e-7;

        static final double HUBER_EPSILON = 0.01;
    }

    /**
        Intialize the feature position based on all current available
        measurements.

        The computed 3d position is used to set the position member variable.
        Note the resulted position is in world frame.

        Arguments:
        @param camStates: A map containing the camera poses with its ID as the
                associated key value. (map of <CAMStateID, CAMState>)

     * @return True if the estimated 3d position of the feature is valid. (bool)
     */
    public boolean initializePosition(Map<Integer, CamState> camStates) {
        List<DMatrixRMaj> measurements = new ArrayList<>(observations.size());
        List<Isometry3D> camPoses = new ArrayList<>(observations.size());

        // TODO: do I need to return when not enough observations?
        // TODO: if I follow tutorial, the keypoints need to be normalized b4 this method.

        for (Tuple2<Integer, DMatrixRMaj> o : observations) {
            if (!camStates.containsKey(o.d0)) continue;
            CamState camState = camStates.get(o.d0);

            measurements.add(o.d1);

            // This camera pose will take a vector from this camera frame
            // to the world frame.
            Isometry3D camPose = new Isometry3D(
                    transpose(quaternionToRotation(camState.orientation), null),
                    camState.position);
            camPoses.add(camPose);
        }
        // All camera poses should be modified such that it takes a vector
        // from the first camera frame in the buffer to this camera frame.
        Isometry3D Tc0w = camPoses.get(0);
        List<Isometry3D> camPosesTmp = new ArrayList<>(camPoses.size());
        for (Isometry3D pose : camPoses) {
            camPosesTmp.add(pose.inverse().mult(Tc0w));
        }
        camPoses = camPosesTmp;

        // Generate initial guess
        DMatrixRMaj initialPosition = generateInitialGuess(
                camPoses.get(camPoses.size()-1),
                measurements.get(0),
                measurements.get(measurements.size()-1)

        );

        DMatrixRMaj solution = new DMatrixRMaj(new double[] {
                initialPosition.get(0) / initialPosition.get(2),
                initialPosition.get(1) / initialPosition.get(2),
                1.0 / initialPosition.get(2)});

        // Apply Levenberg-Marquart method to solve for the 3d position.
        double lambd = Config.INITIAL_DAMPING;
        int innerLoopCount = 0;
        int outerLoopCount = 0;
        boolean isCostReduced = false;
        double deltaNorm = 0; // TODO: is "inf" in Python

        // Compute the initial cost.
        double totalCost = 0.0;
        for (int i = 0; i < camPoses.size(); i++) { // camPoses.size() == measurements.size()
            totalCost += cost(camPoses.get(i), solution, measurements.get(i));
        }

        // Outer loop
        while (outerLoopCount < Config.OUTER_LOOP_MAX_ITERATION && deltaNorm > Config.ESTIMATION_PRECISION) {
            SimpleMatrix A = new SimpleMatrix(3, 3);
            SimpleMatrix b = new SimpleMatrix(3, 1);

            for (int i = 0; i < camPoses.size(); i++) { // camPoses.size() == measurements.size()
                SimpleMatrix J = new SimpleMatrix(2, 3);
                SimpleMatrix r = new SimpleMatrix(2, 1);
                double w = calcJacobian(camPoses.get(i), solution, measurements.get(i), J, r);
                if (w == 1.0) {
                    A = A.plus(J.transpose().mult(J));
                    b = b.plus(J.transpose().mult(r));
                } else {
                    A = A.plus(J.transpose().mult(J)).scale(w * w);
                    b = b.plus(J.transpose().mult(r)).scale(w * w);
                }
            }
            // Inner loop.
            // Solve for the delta that can reduce the total cost.
            while (innerLoopCount < Config.INNER_LOOP_MAX_ITERATION && !isCostReduced) {
                final DMatrixRMaj delta = new DMatrixRMaj();
                {
                    final DMatrixRMaj damper = MathUtils.scale(lambd, identity(3));
                    final DMatrixRMaj a = add(A.getDDRM(), damper, null);
                    denseSolver.setA(a);
                    denseSolver.solve(b.getDDRM(), delta);
                }
                DMatrixRMaj newSolution = subtract(solution, delta, null);
                double deltaNorm = norm(delta); // TODO: which norm was it again?

                double newCost = 0.0;
                for (int i = 0; i < camPoses.size(); i++) {
                    newCost += cost(camPoses.get(i), newSolution, measurements.get(i));
                }

                if (newCost < totalCost) {
                    isCostReduced = true;
                    solution = newSolution;
                    totalCost = newCost;
                    lambd = max(lambd/10.0, 1e-10);
                } else {
                    isCostReduced = false;
                    lambd = min(lambd*10.0, 1e12);

                }
                innerLoopCount++;
            } // Inner loop.
            innerLoopCount = 0;
            outerLoopCount++;
        } // Outer loop.

        // Covert the feature position from inverse depth
        // representation to its 3d coordinate.
        DMatrixRMaj finalPosition = new DMatrixRMaj(new double[]{
                solution.get(0) / solution.get(2),
                solution.get(1) / solution.get(2),
                1.0 / solution.get(2)});

        // Check if the solution is valid. Make sure the feature
        // is in front of every camera frame observing it.
        boolean isValidSolution = true;
        for (Isometry3D pose : camPoses) {
            DMatrixRMaj buffer = mult(pose.R, finalPosition, null);
            add(buffer, pose.t, this.position);
            if (this.position.get(2) <= 0) {
                isValidSolution = false;
                break;
            }
        }

        // Convert the feature position to the world frame.
        {
            DMatrixRMaj buffer = mult(Tc0w.R, finalPosition, null);
            add(buffer, Tc0w.t, this.position);
        }

        this.isInitialized = isValidSolution;
        return isValidSolution;
    }


    /**
     * Compute the Jacobian of the camera observation.
     * @param Tcci A rigid body transformation takes a vector in c0 frame
     *                 to ci frame. (Isometry3d.) Not modified.
     * @param x The current estimation. (vec3.) Not modified.
     * @param z The ith measurement of the feature j in ci frame. (vec2.) Not modified.
     */ // TODO: complete documentation
    private double calcJacobian(Isometry3D Tcci, DMatrixRMaj x, DMatrixRMaj z, SimpleMatrix J, SimpleMatrix r) {
        // Compute hi1, hi2, and hi3 as Equation (37).
        double alpha = x.get(0);
        double beta = x.get(1);
        double rho = x.get(2);

        DMatrixRMaj h = add(
                mult(Tcci.R, new DMatrixRMaj(new double[]{alpha, beta, 1.0}),null),
                MathUtils.scale(rho, Tcci.t),
                null);
        final double h1 = h.get(0);
        final double h2 = h.get(1);
        final double h3 = h.get(2);

        // Compute the Jacobian
        SimpleMatrix W = new SimpleMatrix(3,3);
        W.setColumn(0, SimpleMatrix.wrap(Tcci.R).getColumn(0));  // TODO: use insert instead
        W.setColumn(1, SimpleMatrix.wrap(Tcci.R).getColumn(1));
        W.setColumn(2, SimpleMatrix.wrap(Tcci.t));

        J.zero();
        J.setRow(0, W.getRow(0).scale(1 / h3)
                .minus(W.getRow(2).scale(h1 / (h3 * h3))));
        J.setRow(1, W.getRow(1).scale(1 / h3)
                .minus(W.getRow(2).scale(h2 / (h3 * h3))));

        // Compute the residual
        DMatrixRMaj zHat = new DMatrixRMaj(new double[] {
                h1 / h3,
                h2 / h3
        });
        r.setTo(SimpleMatrix.wrap(subtract(zHat,z,null)));

        // Compute the weight based on the residual.
        double e =; // TODO: which norm again?
        if (e <= Config.HUBER_EPSILON) {
            return 1.0;
        } else {
            return Config.HUBER_EPSILON / (2*e);
        }



    }


    // TODO: add "final" to more params?
    /**
     * Compute the cost of the camera observations
     * @param Tcci A rigid body transformation takes a vector in c0 frame to ci frame. (Isometry3d)
     * @param x The current estimation. (vec3)
     * @param z The ith measurement of the feature j in ci frame. (vec2)
     * @return The cost of this observation. (double)
     */
    private double cost(Isometry3D Tcci, DMatrixRMaj x, DMatrixRMaj z) {
        // Compute hi1, hi2, and hi3 as Equation (37).
        double alpha = x.get(0);
        double beta = x.get(1);
        double rho = x.get(2);

        DMatrixRMaj h = add(
                mult(Tcci.R, new DMatrixRMaj(new double[]{alpha, beta, 1.0}),null),
                MathUtils.scale(rho, Tcci.t),
                null);

        // Predict the feature observation in ci frame.
        DMatrixRMaj zHat = new DMatrixRMaj(new double[]{
                h.get(0) / h.get(2),
                h.get(1) / h.get(2)
        }); // vec2

        // Compute the residual
        double e;
        {
            DMatrixRMaj diff = subtract(zHat, z, null);
            DMatrixRMaj squared = elementMult(diff, diff, null);
            e = elementSum(squared);
        }
        return e;

    }

    /**
     * Compute the initial guess of the feature's 3d position using
     * only two views.
     * @param Tc1c2 A rigid body transformation taking a vector from c2 frame to c1 frame. (Isometry3d)
     * @param z1 feature observation in c1 frame. (vec2)
     * @param z2 feature observation in c2 frame. (vec2)
     * @return Computed feature position in c1 frame. (vec3)
     */
    private DMatrixRMaj generateInitialGuess(Isometry3D Tc1c2, DMatrixRMaj z1, DMatrixRMaj z2) {
        // Construct a least square problem to solve the depth.
        DMatrixRMaj m = mult(
                Tc1c2.R,
                new DMatrixRMaj(new double[]{z1.get(0), z1.get(1), 1.0}),
                null); // vec3

        SimpleMatrix A = SimpleMatrix.wrap(new DMatrixRMaj(new double[] {
                m.get(0) - z2.get(0) * m.get(2),
                m.get(1) - z2.get(1) * m.get(2),
        }));

        SimpleMatrix b = SimpleMatrix.wrap(new DMatrixRMaj(new double[] {
                z2.get(0) * Tc1c2.t.get(2) - Tc1c2.t.get(0),
                z2.get(1) * Tc1c2.t.get(2) - Tc1c2.t.get(1)
        }));

        // Solve for the depth
        double depth = A.transpose()
                .mult(A)
                .invert()
                .mult(A.transpose())
                .mult(b)
                .get(0);

        return new DMatrixRMaj(new double[]{
                z1.get(0) * depth,
                z1.get(1) * depth,
                depth
        });
    }

}
