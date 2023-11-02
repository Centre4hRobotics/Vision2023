package frc.robot;
import java.lang.Math.*;

import edu.wpi.first.math.geometry.*;


public class CC4HTriangulationImplementation {
    public class CC4HInternalPolarVector {
        public double r;
        public double theta; //if theta is the angle from the forward vector the calculations become easier, we use that here

        public CC4HInternalPolarVector(double radius, double angle) {
            r = radius;
            theta = angle;
        }

        /*
         * Converts a polar vector to a cartesian one, with theta configured as the interpretation written above.
         */
        public CC4HInternalCartesianVector convertPolarToCartesian() {
            return new CC4HInternalCartesianVector(this.r * Math.sin( this.theta ), this.r * Math.cos( this.theta ));
        }
    };

    public class CC4HInternalCartesianVector {
        public double x;
        public double y;

        /*
         * Constructor for the vector (cartesian edition)
         * 
         * @param xval    This is the x-value of the vector.
         * @param yval    This is the y-value of the vector.
         */
        public CC4HInternalCartesianVector(double xval, double yval) {
            x = xval;
            y = yval;
        }
        
        public Translation2d convertCartesianToTranslation2d() {
            return new Translation2d(this.x, this.y);
        }
    };

    private class CC4HInternalVxVyInfo {
        public CC4HInternalCartesianVector firstAprilTagLocalPosition;
        public CC4HInternalCartesianVector firstAprilTagAbsolutePosition;
        public CC4HInternalCartesianVector secondAprilTagLocalPosition;
        public CC4HInternalCartesianVector secondAprilTagAbsolutePosition;
        
        /*
         * Constructor for CC4HInternalVxVyInfo!
         * 
         * @param firstLocalPosition    The local position of the first AprilTag.
         * @param firstAprilTagInput    Used for the absolute position of the first AprilTag.
         * @param secondLocalPosition    The local position of the second AprilTag.
         * @param secondAprilTagInput    Used for the absolute position of the second AprilTag.
         */
        public CC4HInternalVxVyInfo(CC4HInternalCartesianVector firstLocalPosition, TriangulationInputInfo firstAprilTagInput, CC4HInternalCartesianVector secondLocalPosition, TriangulationInputInfo secondAprilTagInput) {
            firstAprilTagLocalPosition = firstLocalPosition;
            firstAprilTagAbsolutePosition = firstAprilTagInput.aprilTagAbsolutePosition;
            secondAprilTagLocalPosition = secondLocalPosition;
            secondAprilTagAbsolutePosition = secondAprilTagInput.aprilTagAbsolutePosition;
        }
    };

    /*
     * Solves for Vx (the unit vector moving rightwards relative to the robot) in absolute coordinates.
     * 
     * @param VxSolverParameter     Parameters to feed into the solver.
     */
    private CC4HTriangulationImplementation.CC4HInternalCartesianVector VxSolver(CC4HInternalVxVyInfo VxSolverParameters) {
        double a1 = VxSolverParameters.firstAprilTagLocalPosition.x;
        double b1 = VxSolverParameters.firstAprilTagLocalPosition.y;
        double a2 = VxSolverParameters.secondAprilTagLocalPosition.x;
        double b2 = VxSolverParameters.secondAprilTagLocalPosition.y;

        double x1 = VxSolverParameters.firstAprilTagAbsolutePosition.x;
        double y1 = VxSolverParameters.firstAprilTagAbsolutePosition.y;
        double x2 = VxSolverParameters.secondAprilTagAbsolutePosition.x;
        double y2 = VxSolverParameters.secondAprilTagAbsolutePosition.y;

        CC4HInternalCartesianVector Vx = new CC4HInternalCartesianVector( (b1*x2-b2*x1)/(b1*a2-b2*a1) , (b1*y2-b2*y1)/(b1*a2-b2*a1) );
        return Vx;
    }

    /*
     * Solves for Vy (the unit vector moving forward relative to the robot) in absolute coordinates.
     * 
     * @param VySolverParameter     Parameters to feed into the solver.
     */
    private CC4HTriangulationImplementation.CC4HInternalCartesianVector VySolver(CC4HInternalVxVyInfo VySolverParameters) {
        double a1 = VySolverParameters.firstAprilTagLocalPosition.x;
        double b1 = VySolverParameters.firstAprilTagLocalPosition.y;
        double a2 = VySolverParameters.secondAprilTagLocalPosition.x;
        double b2 = VySolverParameters.secondAprilTagLocalPosition.y;

        double x1 = VySolverParameters.firstAprilTagAbsolutePosition.x;
        double y1 = VySolverParameters.firstAprilTagAbsolutePosition.y;
        double x2 = VySolverParameters.secondAprilTagAbsolutePosition.x;
        double y2 = VySolverParameters.secondAprilTagAbsolutePosition.y;

        CC4HInternalCartesianVector Vy = new CC4HInternalCartesianVector( (a1*x2-a2*x1)/(a1*b2-a2*b1) , (a1*y2-a2*y1)/(a1*b2-a2*b1) );
        return Vy;
    }

    /*
     * Finds the angle between the absolute vertical unit vector (0,1) and the relative vertical unit vector Vy.
     * 
     * @param Vy     The forward vector Vy relative to the robot in absolute coordinates.
     */
    private double angleBetweenAbsoluteYRelativeYSolver(CC4HInternalCartesianVector Vy) {
        // arccos( ([0 1] · Vy) / (|[0 1]| |Vy|) ) = arccos(Vy.y / |Vy|)
        double length = Math.sqrt(Vy.x*Vy.x + Vy.y*Vy.y);
        return Math.acos(Vy.y / length);
    }

    public class TriangulationInputInfo {
        public CC4HInternalCartesianVector aprilTagAbsolutePosition;
        public CC4HInternalPolarVector aprilTagDistanceAndYaw;

        /*
        * Class constructor.
        * 
        * @param absolute     Absolute position of AprilTag.
        * @param distanceyaw  Local (polar) position of the AprilTag.
        */
        public TriangulationInputInfo(CC4HInternalCartesianVector absolute, CC4HInternalPolarVector distanceyaw) {
            aprilTagAbsolutePosition = absolute;
            aprilTagDistanceAndYaw = distanceyaw;
        }
    };

    /*
     * Creates the input info using external types (ew, disgusting).
     * 
     * @param absolute       The absolute AprilTag position as a Translation2d.
     * @param distanceFlattened       "Flattened" distance of AprilTag from robot.
     * @param yaw   Yaw of AprilTag from PhotonVision.
     */
    public TriangulationInputInfo NastyInputInfoCavemanBrainedHack(Translation2d absolute, double distanceFlattened, double yaw) {
        return new TriangulationInputInfo(new CC4HInternalCartesianVector(absolute.getX(), absolute.getY()), 
                                          new CC4HInternalPolarVector(distanceFlattened, yaw) );
    }

    /*
     * The one function to kick the whole process off.
     * 
     * @param firstTriangulationInfo       The input information for the first AprilTag.
     * @param secondTriangulationInfo       The input information for the second AprilTag.
     */
    public Transform2d calculateTriangulationVector(TriangulationInputInfo firstTriangulationInfo, TriangulationInputInfo secondTriangulationInfo) {
        CC4HInternalCartesianVector firstAprilTagLocalCartesian = firstTriangulationInfo.aprilTagDistanceAndYaw.convertPolarToCartesian();
        CC4HInternalCartesianVector secondAprilTagLocalCartesian = secondTriangulationInfo.aprilTagDistanceAndYaw.convertPolarToCartesian();
        CC4HInternalVxVyInfo VxVySolvingVectors = new CC4HInternalVxVyInfo(firstAprilTagLocalCartesian, firstTriangulationInfo, secondAprilTagLocalCartesian, secondTriangulationInfo);
        
        //CC4HInternalCartesianVector Vx = VxSolver(VxVySolvingVectors); //this might be needed in the future
        CC4HInternalCartesianVector Vy = VySolver(VxVySolvingVectors);
        
        double theta = angleBetweenAbsoluteYRelativeYSolver(Vy);
        double x1 = firstTriangulationInfo.aprilTagAbsolutePosition.x;
        double y1 = firstTriangulationInfo.aprilTagAbsolutePosition.y;
        double d = firstTriangulationInfo.aprilTagDistanceAndYaw.r;
        double alpha = firstTriangulationInfo.aprilTagDistanceAndYaw.theta;

        CC4HInternalCartesianVector robotPosition = new CC4HInternalCartesianVector(x1 - d * Math.sin(theta + alpha),  y1 - d * Math.cos(theta + alpha));
        return new Transform2d(robotPosition.convertCartesianToTranslation2d(), new Rotation2d(theta));
    }
}
