package com.pedropathing.geometry;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.MatrixUtil;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import static com.pedropathing.math.AbstractBijectiveMap.NumericBijectiveMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * This is the BezierCurve class. This class handles the creation of Bezier curves, which are used
 * as the basis of the path for the Path class. Bezier curves are parametric curves defined by a set
 * of control points. So, they take in one input, t, that ranges from [0, 1] and that returns a point
 * on the curve. Essentially, Bezier curves are a way of defining a parametric line easily. You can
 * read more on Bezier curves here: <a href="https://en.wikipedia.org/wiki/Bézier_curve">...</a>
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author William Phomphakdee - 7462 Not to Scale Alumni
 * @version 1.0, 3/5/2024
 */
public class BezierCurve implements Curve {
    private ArrayList<Pose> controlPoints;

    protected ArrayList<FuturePose> futureControlPoints = new ArrayList<>();

    protected boolean initialized = false;

    private Vector endTangent = new Vector();

    protected final int APPROXIMATION_STEPS = 1000;

    private final int DASHBOARD_DRAWING_APPROXIMATION_STEPS = 100;

    private double[][] panelsDrawingPoints;

    private double length;

    private Matrix cachedMatrix = new Matrix();

    private int[][] diffPowers;
    private int[][] diffCoefficients;
    protected PathConstraints pathConstraints;

    protected NumericBijectiveMap completionMap = new NumericBijectiveMap();

    /**
     * This is the default constructor for the BezierCurve class. It initializes an empty BezierCurve.
     * This is not recommended to use, as it does not set any control points or path constraints.
     */
    public BezierCurve() {
    }

    /**
     * This constructor creates a BezierCurve with the specified control points and path constraints.
     * @param controlPoints the control points for the BezierCurve, which must be at least 3 points.
     * @param constraints the path constraints for the BezierCurve.
     */
    public BezierCurve(List<Pose> controlPoints, PathConstraints constraints){
        this.pathConstraints = constraints;
        if (controlPoints.size()<3) {
            try {
                throw new Exception("Too few control points");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        this.controlPoints = new ArrayList<>(controlPoints);
        initialize();
    }

    /**
     * This constructor creates a BezierCurve with the specified control points and path constraints.
     * @param constraints the path constraints for the BezierCurve.
     * @param controlPoints the control points for the BezierCurve, which must be at least 3 points.
     */
    protected BezierCurve(PathConstraints constraints, List<FuturePose> controlPoints) {
        this.pathConstraints = constraints;
        if (controlPoints.size()<3) {
            try {
                throw new Exception("Too few control points");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        boolean lazyInitialize = false;
        ArrayList<Pose> initializedControlPoints = new ArrayList<>();
        for (FuturePose pose : controlPoints) {
            if (!pose.initialized()) {
                lazyInitialize = true;
                break;
            }

            initializedControlPoints.add(pose.getPose());
        }

        if (lazyInitialize) {
            this.controlPoints = new ArrayList<>();
            this.futureControlPoints = new ArrayList<>(controlPoints);
        } else {
            this.controlPoints = initializedControlPoints;
            initialize();
        }
    }

    /**
     * This constructor creates a BezierCurve with the specified control points and the default path constraints.
     * @param controlPoints the control points for the BezierCurve, which must be at least 3 points.
     */
    public BezierCurve(List<Pose> controlPoints) {
        this(controlPoints, PathConstraints.defaultConstraints);
    }

    /**
     * This constructor creates a BezierCurve with the specified control points and path constraints.
     * @param constraints the path constraints for the BezierCurve.
     * @param controlPoints the control points for the BezierCurve, which must be at least 3 points.
     */
    public BezierCurve(PathConstraints constraints, FuturePose... controlPoints) {
        this(constraints, new ArrayList<>(Arrays.asList(controlPoints)));
    }

    /**
     * This constructor creates a BezierCurve with the specified control points and the default path constraints.
     * @param controlPoints the control points for the BezierCurve, which must be at least 3 points.
     */
    public BezierCurve(FuturePose... controlPoints) {
        this(PathConstraints.defaultConstraints, controlPoints);
    }

    /**
     * This handles most of the initialization of the BezierCurve that is called from the constructor.
     */
    public void initialize() {
        if (initialized) return; // If already initialized, do nothing
        if (controlPoints.isEmpty() && !futureControlPoints.isEmpty()) {
            for (FuturePose pose : futureControlPoints) {
                controlPoints.add(pose.getPose());
            }
            futureControlPoints.clear();
        }
        initialized = true;
        generateBezierCurve();
        length = approximateLength();
        endTangent.setOrthogonalComponents(controlPoints.get(controlPoints.size()-1).getX()-controlPoints.get(controlPoints.size()-2).getX(),
                controlPoints.get(controlPoints.size()-1).getY()-controlPoints.get(controlPoints.size()-2).getY());
        endTangent = endTangent.normalize();
        initializePanelsDrawingPoints();
    }

    /**
     * This creates the Array that holds the Points to draw on Panels.
     */
    public void initializePanelsDrawingPoints() {
        this.panelsDrawingPoints = new double[2][this.DASHBOARD_DRAWING_APPROXIMATION_STEPS + 1];
        for (int i = 0; i <= this.DASHBOARD_DRAWING_APPROXIMATION_STEPS; i++) {
            Pose currentPoint = this.getPose(i/(double) (this.DASHBOARD_DRAWING_APPROXIMATION_STEPS));
            this.panelsDrawingPoints[0][i] = currentPoint.getX();
            this.panelsDrawingPoints[1][i] = currentPoint.getY();
        }
    }

    /**
     * This returns a 2D Array of doubles containing the x and y positions of points to draw on Panels.
     * @return returns the 2D Array to draw on Panels
     */
    public double[][] getPanelsDrawingPoints() {
        return this.panelsDrawingPoints;
    }

    /**
     * This generates the Bezier curve. It assumes that the ArrayList of control points has been set.
     * This caches the matrix generated by multiplying the characteristic matrix and the matrix where each control
     * point is a row vector.
     */
    public void generateBezierCurve() {
        Matrix controlPointMatrix = new Matrix(this.controlPoints.size(), 2);
        for (int i = 0; i < this.controlPoints.size(); i++) {
            Pose p = this.controlPoints.get(i);
            controlPointMatrix.set(i, new double[]{p.getX(), p.getY()});
        }
        this.cachedMatrix = CharacteristicMatrixSupplier.getBezierCharacteristicMatrix(this.controlPoints.size() - 1).multiply(controlPointMatrix);
        initializeDegreeArray();
        initializeCoefficientArray();
    }

    /**
     * This returns the unit tangent Vector at the end of the BezierCurve.
     *
     * @return returns the end tangent Vector.
     */
    public Vector getEndTangent() {
        return endTangent.copy();
    }

    /**
     * This approximates the length of the BezierCurve in APPROXIMATION_STEPS number of steps. It's
     * like a Riemann's sum, but for a parametric function's arc length.
     *
     * @return returns the approximated length of the BezierCurve.
     */
    public double approximateLength() {
        Pose previousPoint = getPose(0);
        Pose currentPoint;
        double approxLength = 0;
        for (int i = 1; i <= APPROXIMATION_STEPS; i++) {
            double t = i/(double)APPROXIMATION_STEPS;
            currentPoint = getPose(t);
            approxLength += previousPoint.distanceFrom(currentPoint);
            previousPoint = currentPoint;
            completionMap.put(t, approxLength);
        }
        return approxLength;
    }

    /**
     * Initializes the degree/power array (for later processing) and cache them
     */
    public void initializeDegreeArray(){
        int deg = this.controlPoints.size() - 1;
        // for now, cache position, velocity, and acceleration powers (thus 3) per bezier obj (change to global caching later)
        this.diffPowers = new int[3][this.controlPoints.size()];

        for (int i = 0; i < this.diffPowers.length; i++) {
            this.diffPowers[i] = BezierCurve.genDiff(deg, i);
        }
    }

    /**
     * Generate and return a polynomial's powers at the differentiation level
     * @param deg degree of poly
     * @param diffLevel number of differentiations
     * @return powers of each term in integers
     */
    private static int[] genDiff(int deg, int diffLevel){
        int[] output = new int[deg + 1];

        for (int i = diffLevel; i < output.length; i++) {
            output[i] = i - diffLevel;
        }

        return output;
    }

    /**
     * Initializes the coefficient array (for later processing) and cache them.
     * Each row is a different level of differentiation.
     */
    public void initializeCoefficientArray(){
        // for now, cache coefficients for the 0th, 1st, and 2nd derivatives (change to global caching later)
        this.diffCoefficients = new int[3][this.controlPoints.size()];

        Arrays.fill(this.diffCoefficients[0], 1);

        for (int row = 1; row < this.diffCoefficients.length; row++) {
            for (int col = 0; col < this.diffCoefficients[0].length; col++) {
                this.diffCoefficients[row][col] = this.diffCoefficients[row - 1][col] * this.diffPowers[row - 1][col];
            }
        }
    }

    /**
     * This method gets the t-vector at the specified differentiation level.
     * @param t t value of the parametric curve; [0, 1]
     * @param diffLevel specifies how many differentiations are done
     * @return t vector
     */
    public double[] getTVector(double t, int diffLevel){
        int[] degrees = this.diffPowers[diffLevel];
        double[] powers = new double[this.controlPoints.size()];

        powers[0] = 1;
        for (int i = 1; i < powers.length; i++) {
            powers[i] = t * powers[i - 1];
        }

        double[] output = new double[powers.length];

        for (int i = 0; i < degrees.length; i++) {
            output[i] = powers[degrees[i]] * this.diffCoefficients[diffLevel][i];
        }

        return output;
    }

    /**
     * This returns the point on the Bezier curve that is specified by the parametric t value. A
     * Bezier curve is a parametric function that returns points along it with t ranging from [0, 1],
     * with 0 being the beginning of the curve and 1 being at the end. The Follower will follow
     * BezierCurves from 0 to 1, in terms of t.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the point requested.
     */
    public Pose getPose(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Matrix outPos = (new Matrix(new double[][]{getTVector(t, 0)})).multiply(this.cachedMatrix);
        return new Pose(outPos.get(0, 0), outPos.get(0, 1));
    }

    /**
     * This returns the curvature of the Bezier curve at a specified t-value.
     *
     * @param t the parametric t input.
     * @return returns the curvature.
     */
    public double getCurvature(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Vector derivative = getDerivative(t);
        Vector secondDerivative = getSecondDerivative(t);

        if (derivative.getMagnitude() == 0) return 0;
        return (derivative.cross(secondDerivative))/Math.pow(derivative.getMagnitude(),3);
    }

    /**
     * This returns the derivative on the BezierCurve that is specified by the parametric t value.
     * This is returned as a Vector, and this Vector is the tangent to the BezierCurve.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the derivative requested.
     */
    public Vector getDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Matrix outVel = (new Matrix(new double[][]{getTVector(t, 1)})).multiply(this.cachedMatrix);
        Vector output = new Vector();
        output.setOrthogonalComponents(outVel.get(0, 0), outVel.get(0, 1));
        return output;
    }

    /**
     * This returns the second derivative on the BezierCurve that is specified by the parametric t value.
     * This is returned as a Vector, and this Vector is the acceleration on the BezierCurve.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the second derivative requested.
     */
    public Vector getSecondDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Matrix outAccel = (new Matrix(new double[][]{getTVector(t, 2)})).multiply(this.cachedMatrix);
        Vector output = new Vector();
        output.setOrthogonalComponents(outAccel.get(0, 0), outAccel.get(0, 1));
        return output;
    }

    /**
     * This method calculates the position, velocity, and acceleration and puts them into a matrix
     * as row vectors.
     * @param t t value of the parametric curve; [0, 1]
     * @return matrix with row vectors corresponding to position, velocity, and acceleration at the requested t value
     */
    public Matrix getPointCharacteristics(double t){
        t = MathFunctions.clamp(t, 0, 1);

        return new Matrix(new double[][]{
                getTVector(t, 0),
                getTVector(t, 1),
                getTVector(t, 2)
        }).multiply(this.cachedMatrix);
    }

    /**
     * This gets an approximate second derivative essentially using the limit method. This is used for heading only
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the approximated second derivative.
     */
    @Override
    public Vector getNormalVector(double t) {
        double current = getDerivative(t).getTheta();
        double deltaCurrent = getDerivative(t + 0.0001).getTheta();

        return new Vector(1, deltaCurrent - current);
    }

    /**
     * Returns the ArrayList of control points for this BezierCurve.
     *
     * @return This returns the control points.
     */
    public ArrayList<Pose> getControlPoints() {
        return controlPoints;
    }

    /**
     * Returns the first control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getFirstControlPoint() {
        return controlPoints.get(0);
    }

    /**
     * Returns the second control point, or the one after the start, for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getSecondControlPoint() {
        return controlPoints.get(1);
    }

    /**
     * Returns the second to last control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getSecondToLastControlPoint() {
        return controlPoints.get(controlPoints.size()-2);
    }

    /**
     * Returns the last control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getLastControlPoint() {
        return controlPoints.get(controlPoints.size()-1);
    }

    /**
     * Returns the approximate length of this BezierCurve.
     *
     * @return This returns the length.
     */
    public double length() {
        return length;
    }

    /**
     * Returns the type of path. This is used in case we need to identify the type of BezierCurve
     * this is.
     *
     * @return returns the type of path.
     */
    public String pathType() {
        return "curve";
    }

    /**
     * Returns a new BezierCurve with the control points reversed.
     *
     * @return a new BezierCurve with reversed control points.
     */
    public BezierCurve getReversed() {
        ArrayList<Pose> reversedControlPoints = new ArrayList<>(controlPoints);
        Collections.reverse(reversedControlPoints);
        BezierCurve reversedCurve = new BezierCurve(reversedControlPoints);
        reversedCurve.initialize();
        return reversedCurve;
    }

    /**
     * Returns the closest point t-value to the specified pose.
     * @param pose the pose to find the closest point to
     * @return the closest point t-value
     */
    public double getClosestPoint(Pose pose, int searchLimit, double initialTValueGuess) {
        double[] searchEstimates = new double[]{0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, initialTValueGuess};
        double closestDist = 1e9;
        double bestGuess = 0;

        for (double t : searchEstimates) {
            double dist = getPose(t).distSquared(pose);

            if (dist < closestDist) {
                closestDist = dist;
                bestGuess = t;
            }
        }

        initialTValueGuess = bestGuess;

        for (int i = 0; i < searchLimit; i++) {
            Matrix pointAtT = this.getPointCharacteristics(initialTValueGuess);
            Pose lastPos = new Pose(pointAtT.get(0, 0), pointAtT.get(0, 1));
            /*
            Resultant matrix is:
            [...] // first row is ignored
            [(lastPos.x - pose.x) * x'(t) + (lastPos.y - pose.y) * y'(t)]
            [(lastPos.x - pose.x) * x''(t) + (lastPos.y - pose.y) * y''(t)]
             */
            Matrix resultant = pointAtT.multiply(new Matrix(new double[][]{
                    {(lastPos.getX() - pose.getX())},
                    {(lastPos.getY() - pose.getY())}
            }));

            double firstDerivative = 2 * resultant.get(1, 0);
            double secondDerivative = 2 * (resultant.get(2, 0) + (pointAtT.get(1, 0) * pointAtT.get(1, 0) + (pointAtT.get(1, 1) * pointAtT.get(1, 1))));

            initialTValueGuess = MathFunctions.clamp(initialTValueGuess - firstDerivative / (secondDerivative + 1e-9), 0, 1);
            if (getPose(initialTValueGuess).distanceFrom(lastPos) < 0.1) break;
        }

        return initialTValueGuess;
    }

    /**
     * Returns the closest point t-value to the specified pose.
     * @param pose the pose to find the closest point to
     * @return the closest point t-value
     */
    public double getClosestPoint(Pose pose, double initialTValueGuess) {
        return getClosestPoint(pose, getPathConstraints().getBEZIER_CURVE_SEARCH_LIMIT(), initialTValueGuess);
    }

    /**
     * Returns whether the t value is at the end of the parametric curve.
     * @param t the t value of the parametric curve; [0, 1]
     * @return true if at the end, false otherwise
     */
    public boolean atParametricEnd(double t) {
        return t >= pathConstraints.getTValueConstraint();
    }

    /**
     * Sets the control points for this BezierCurve.
     * @param controlPoints the new control points to set
     */
    public void setControlPoints(ArrayList<Pose> controlPoints) {
        this.controlPoints = controlPoints;
    }

    /**
     * Sets the path constraints for this BezierCurve.
     * @param pathConstraints the new path constraints to set
     */
    public void setPathConstraints(PathConstraints pathConstraints) {
        this.pathConstraints = pathConstraints;
    }

    @Override
    public PathConstraints getPathConstraints() {
        return pathConstraints;
    }

    /**
     * Returns the path completion at a given t value.
     * This is used to get the percentage of the path that has been completed.
     *
     * @param t the t value of the parametric curve; [0, 1]
     * @return returns the path completion as a decimal on the range [0,1].
     */
    public double getPathCompletion(double t) {
        if (length == 0) return 0.0;
        return completionMap.interpolateKey(t) / length;
    }

    /**
     * Returns the t value corresponding to a given path completion percentage.
     * @param pathCompletion the path completion percentage; [0, 1]
     * @return returns the t value corresponding to the path completion percentage.
     */
    public double getT(double pathCompletion) {
        return completionMap.interpolateValue(pathCompletion);
    }

    /**
     * Returns whether the BezierCurve has been initialized.
     * @return true if the BezierCurve has been initialized, false otherwise.
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Generates a BezierCurve that passes through the given points
     * @param startPoint the initial point the curve passes through
     * @param midpoint a point in the middle for the curve to pass through
     * @param endPoint the final point the curve passes through
     * @return the BezierCurve passing through the points
     */
    public static BezierCurve through(Pose startPoint, Pose midpoint, Pose endPoint) {
        double cx = 2 * midpoint.getX() - (startPoint.getX() + endPoint.getX()) / 2.0;
        double cy = 2 * midpoint.getY() - (startPoint.getY() + endPoint.getY()) / 2.0;
        Pose controlPoint = new Pose(cx, cy);
        return new BezierCurve(startPoint, controlPoint, endPoint);
    }

    /**
     * Generates a BezierCurve that passes through the given points
     * @param startPoint the first point the curve passes through
     * @param midPoint1 the second point the curve passes through
     * @param midPoint2 the third point the curve passes through
     * @param endPoint the fourth point the curve passes through
     * @return the BezierCurve passing through the points
     */
    public static BezierCurve through(Pose startPoint, Pose midPoint1, Pose midPoint2, Pose endPoint) {
        double distance1 = Math.pow(midPoint1.distanceFrom(startPoint), 0.5);
        double distance2 = Math.pow(midPoint2.distanceFrom(midPoint1), 0.5);
        double distance3 = Math.pow(endPoint.distanceFrom(midPoint2), 0.5);
        double sqDistance1 = distance1 * distance1;
        double sqDistance2 = distance2 * distance2;
        double sqDistance3 = distance3 * distance3;
        double t1 = (2 * distance1 * distance1) + (3 * distance1 * distance2) + (distance2 * distance2);
        double t2 = 3 * distance1 * (distance1 + distance2);
        double t3 = (2 * distance3 * distance3) + (3 * distance3 * distance2) + (distance2 * distance2);
        double t4 = 3 * distance3 * (distance3 + distance2);

        Pose controlPoint1 = new Pose((sqDistance1 * midPoint2.getX() - sqDistance2 * startPoint.getX() + t1 * midPoint1.getX()) / t2,
                (sqDistance1 * midPoint2.getY() - sqDistance2 * startPoint.getY() + t1 * midPoint1.getY()) / t2);
        Pose controlPoint2 = new Pose((sqDistance3 * midPoint1.getX() - sqDistance2 * endPoint.getX() + t3 * midPoint2.getX()) / t4,
                (sqDistance3 * midPoint1.getY() - sqDistance2 * endPoint.getY() + t3 * midPoint2.getY()) / t4);

        return new BezierCurve(startPoint, controlPoint1, controlPoint2, endPoint);
    }

    /**
     * Generates a BezierCurve that passes through the given points
     * @param points vararg of points; requirements more than two points
     * @return the BezierCurve passing through the points
     * @author William Phomphakdee - 7462 Not to Scale Alumni
     */
    public static BezierCurve through(Pose... points){
        double[] tValues = new double[points.length];
        tValues[points.length - 1] = 1;
        double increment = 1d / (points.length - 1);
        for (int i = 1; i < tValues.length - 1; i++) {
            tValues[i] = tValues[i - 1] + increment;
        }

        TVector tVectorGen = new TVector(points.length);
        Matrix tMatrix = new Matrix(points.length, points.length);
        for (int i = 0; i < tMatrix.getRows(); i++) {
            tMatrix.setRow(i, tVectorGen.getRowVector(tValues[i], 0).getRow(0));
        }

        Matrix bezier = CharacteristicMatrixSupplier.getBezierCharacteristicMatrix(points.length - 1);

        Matrix targetPointMatrix = new Matrix(points.length, 2);
        for (int i = 0; i < points.length; i++) {
            targetPointMatrix.setRow(i, points[i].getX(), points[i].getY());
        }

        Matrix outputControlPoints = Matrix.rref(tMatrix.multiply(bezier), MatrixUtil.eye(points.length))[1].multiply(targetPointMatrix);
        Pose[] output = new Pose[points.length];

        for (int i = 0; i < outputControlPoints.getRows(); i++) {
            output[i] = new Pose(outputControlPoints.get(i, 0), outputControlPoints.get(i, 1));
        }

        return new BezierCurve(output);
    }
}