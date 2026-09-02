package frc.robot.subsystems.object_detection;

public interface ObjectDetectionIO{
    class ObjectDetectionIOInputs {
        public double[] x;
        public double[] y;
        public double[] z;
        public double[] pitch;
        public double[] yaw;
        public double[] roll;
        public double[] width;
        public double[] height;
    }

    default void updateInputs(ObjectDetectionIOInputs inputs) {}

    default void threadFn() {}
}
