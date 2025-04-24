package frc.robot.vision;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.LimelightHelpers.RawFiducial;
import redrocklib.logging.SmartDashboardNumber;

public class Localization {
    private static boolean isSim = false;

    private static Pose2d storedPose;

    private static int[][] validIDs;
    private static String[] limeLightNames;
    private static double[][] limeLightStdvs;
    private static double[] ambiguityThresholds;
    private static double[] distanceThresholds;

    private static PoseEstimate[] estimates;

    private static SmartDashboardNumber kStdvDemoninator = new SmartDashboardNumber("localization/stdv-denom-scale", 30);

    // Must be called before use!
    public static void initialize() {
        Localization.initialize(
            new int[][] {{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}}, // Tags
            new String[] {"climb", "front"},                            // Names
            new double[][] {{0.9, 0.9, 9999}, {1.2, 1.2, 9999}},        // Stdvs
            new double[] {0.2, 0.1},                                    // Ambiguity
            new double[] {5, 0.5}                                       // Distance
        );
    }

    public static void initialize(int[][] vIDs, String[] llNames, double[][] llStdvs, double[] aThresholds, double[] dThresholds) {
        Localization.limeLightNames = llNames;

        if(vIDs.length < llNames.length)
        {
            Localization.validIDs = new int[llNames.length][];
            for(int i = 0; i < llNames.length; i++)
                Localization.validIDs[i] = vIDs[0];
        }
        else
            Localization.validIDs = vIDs;
            
        if(llStdvs.length < llNames.length)
        {
            Localization.limeLightStdvs = new double[llNames.length][];
            for(int i = 0; i < llNames.length; i++)
                Localization.limeLightStdvs[i] = llStdvs[0];
        }
        else
            Localization.limeLightStdvs = llStdvs;
            
        if(aThresholds.length < llNames.length)
        {
            Localization.ambiguityThresholds = new double[llNames.length];
            for(int i = 0; i < llNames.length; i++)
                Localization.ambiguityThresholds[i] = aThresholds[0];
        }
        else
            Localization.ambiguityThresholds = aThresholds;
            
        if(dThresholds.length < llNames.length)
        {
            Localization.distanceThresholds = new double[llNames.length];
            for(int i = 0; i < llNames.length; i++)
                Localization.distanceThresholds[i] = dThresholds[0];
        }
        else
            Localization.distanceThresholds = dThresholds;

        estimates = new PoseEstimate[Localization.limeLightNames.length];
        for (int i = 0; i < Localization.limeLightNames.length; i++) {
            LimelightHelpers.SetFiducialIDFiltersOverride(limeLightNames[i], validIDs[i]);
        }

        isSim = Utils.isSimulation();
    }

    public static void updateHeading(double headingDegrees, double rotationRateDegrees) {
        if(estimates == null)
            Localization.initialize();

        for(int i = 0; i < limeLightNames.length; i++){
            String name = "limelight-" + limeLightNames[i];
            SmartDashboard.putNumber("localization/"+name+"/heading", headingDegrees);
            LimelightHelpers.SetRobotOrientation(name, headingDegrees, rotationRateDegrees, 0, 0, 0, 0);
            PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            if (estimate != null){
                Logger.recordOutput("limelight/" + limeLightNames[i] + "/avgTagDist", Localization.getAvgDistance(estimate, i));
                Logger.recordOutput("limelight/" + limeLightNames[i] + "/estimate/avgTagDist", Localization.getAvgDistance(estimate, i));
                Logger.recordOutput("limelight/" + limeLightNames[i] + "/estimate/tagCount", estimate.tagCount);
                Logger.recordOutput("limelight/" + limeLightNames[i] + "/estimate/avgAmbiguity", Localization.getAvgAmbiguity(estimate, i));
                Logger.recordOutput("limelight/" + limeLightNames[i] + "/estimate/belowAmbiguityThreshold", Localization.belowAmbiguityThreshold(estimate, i));
                Logger.recordOutput("limelight/" + limeLightNames[i] + "/nullEstimate", false);
                Pose2d mt1Pose = LimelightHelpers.getBotPose2d_wpiBlue(name);
                Logger.recordOutput("limelight/" + limeLightNames[i] + "/MT1", mt1Pose==null?new Pose2d():mt1Pose);
                Logger.recordOutput("limelight/" + limeLightNames[i] + "/MT2", estimate==null?new Pose2d():estimate.pose);
            }
            else
                Logger.recordOutput("limelight/" + limeLightNames[i] + "/nullEstimate", true);

            Localization.estimates[i] = estimate;
        }

        if(storedPose != null)
        {
            SmartDashboard.putNumber("localization/pose/x", storedPose.getX());
            SmartDashboard.putNumber("localization/pose/y", storedPose.getY());
            SmartDashboard.putNumber("localization/pose/heading", storedPose.getRotation().getDegrees());
            Logger.recordOutput("localization/storedPose", storedPose);
        }
    }

    public static PoseEstimate[] getPoseEstimates() {
        return estimates;
    }

    public static void setPose(Pose2d pose) {
        storedPose = pose;
    }

    public static Pose2d getPose() {
        return storedPose;
    }

    public static double getMT1Heading() {
        if(limeLightNames.length == 0)
        {
            Logger.recordOutput("localization/exception", "No limelights found for MT1 heading");
            return -1;
        }
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-" + limeLightNames[0]).pose.getRotation().getDegrees();
    }

    public static String getLLName(int index) {
        return limeLightNames[index];
    }

    public static Matrix<N3, N1> getStdvs(double distanceToTarget, int index) {
        if(index > limeLightStdvs.length && limeLightStdvs.length > 0)
            index = 0;
        
        if(index < 0 || limeLightStdvs.length == 0)
        {
            Logger.recordOutput("localization/exception", "Stdv index out of bounds! (" + index + "/" + limeLightStdvs.length + ")");
            return VecBuilder.fill(0,0,0);
        }

        return VecBuilder.fill(
            adjustStdv(limeLightStdvs[index][0], distanceToTarget),
            adjustStdv(limeLightStdvs[index][1], distanceToTarget),
            adjustStdv(limeLightStdvs[index][2], distanceToTarget)
        );
    }

    private static double adjustStdv(double stdv, double distanceToTarget) {
        return stdv + stdv * (distanceToTarget * distanceToTarget) / kStdvDemoninator.getNumber();
    }

    public static double getAvgAmbiguity(PoseEstimate estimate, int index) {
        double ambiguitySum = 0;
        int validTags = 0;
        for (RawFiducial fiducial : estimate.rawFiducials)
            for(int i : validIDs[validIDs.length>index&&index>=0?index:0])
                if(i == fiducial.id)
                {
                    ambiguitySum += fiducial.ambiguity;
                    validTags++;
                }
        
        return ambiguitySum / validTags;
    }

    public static boolean belowAmbiguityThreshold(PoseEstimate estimate, int index) {
        return Localization.getAvgAmbiguity(estimate, index) < ambiguityThresholds[ambiguityThresholds.length>index&&index>=0?index:0];
    }

    public static double getAvgDistance(PoseEstimate estimate, int index) {
        double distanceSum = 0;
        int validTags = 0;
        for (RawFiducial fiducial : estimate.rawFiducials)
            for(int i : validIDs[validIDs.length>index&&index>=0?index:0])
                if(i == fiducial.id)
                {
                    distanceSum += fiducial.distToCamera;
                    validTags++;
                }
        
        return distanceSum / validTags;
    }

    public static boolean withinRejectionDistance(PoseEstimate estimate, int index) {
        
        return Localization.getAvgDistance(estimate, index) < distanceThresholds[distanceThresholds.length>index&&index>=0?index:0];
    }
}


/*

// Proper use in CommandSwerveDrivetrain:

import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.Localization;

public void setHeadingFromMegatag1() {
    double deg = 0;
    try {
        deg = Localization.getMT1Heading();
    } catch (Exception e) {
        Logger.recordOutput("localization/exception", e.getMessage());
        deg = 0;
    }
    if(deg < 0)
        deg = 0;
    this.targetHeadingDegrees = deg;
    this.resetPose(
        new Pose2d(
            this.getPose().getX(),
            this.getPose().getY(),
            Rotation2d.fromDegrees(deg)
        )
    );
}

public void periodic() {
    Localization.updateHeading(this.getHeadingDegrees(), this.getRotationRateDegrees()); // Update ll values every cycle
    if (visionEnabled.getValue()) updateVisionMeasurements(); // Only use ll for pose if vision is enabled
    Localization.setPose(this.getPose()); // Update stored pose
}

public void updateVisionMeasurements() {
    LimelightHelpers.PoseEstimate[] estimates = Localization.getPoseEstimates();
    for(int i = 0; i < estimates.length; i++) {
        String name = Localization.getLLName(i);
        if(poseEstimateIsValid(estimates[i], i))
        {
            this.addVisionMeasurement(
                estimates[i].pose,
                Utils.getCurrentTimeSeconds() - estimates[i].latency * 0.001,
                Localization.getStdvs(estimates[i].avgTagDist, i));
            
            SmartDashboard.putBoolean("localization/" + name + "-vision-accepted", true);
            SmartDashboard.putNumber("localization/" + name + "-latency", estimates[i].latency * 0.001);
            Logger.recordOutput("localization/" + name + "/visionAccepted", true);
            Logger.recordOutput("localization/" + name + "/latency", estimates[i].latency * 0.001);
        }
        else
        {
            SmartDashboard.putBoolean("localization/" + name + "-vision-accepted", false);
            Logger.recordOutput("localization/" + name + "/visionAccepted", false);
        }
    }
}

private boolean poseEstimateIsValid(LimelightHelpers.PoseEstimate estimate, int index) {
    return 
        LimelightHelpers.validPoseEstimate(estimate) &&                                                 // Estimate exists and has tags
        (Double.compare(estimate.pose.getX(), 0) > 0 && Double.compare(estimate.pose.getY(), 0) > 0) && // Estimate is not (0,0)
        Localization.belowAmbiguityThreshold(estimate, index) &&                                        // Estimate has low ambiguity
        Localization.withinRejectionDistance(estimate, index) &&                                        // Tags are not too far away
        Math.abs(this.getRotationRateDegrees()) < this.kRejectionRotationRate.getNumber();              // Robot rotation is slow
}

*/