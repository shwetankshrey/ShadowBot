using System;
using System.Linq;
using System.IO;
using System.Windows;
using System.Windows.Media.Media3D;
using Microsoft.Kinect;

namespace ShadowBot_Kinect
{
    public partial class MainWindow : Window
    {
        private KinectSensor sensor;
        public MainWindow()
        {
            InitializeComponent();
        }
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }
            if (null != this.sensor)
            {
                this.sensor.SkeletonStream.Enable();
                this.sensor.SkeletonFrameReady += Sensor_SkeletonFrameReady;
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }
            if (null == this.sensor)
            {
                MessageBox.Show("Kinect not Found!", "Error");
            }
        }
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }
        private void Sensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[6];
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame == null)
                {
                    return;
                }
                skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                skeletonFrame.CopySkeletonDataTo(skeletons);
            }
            Skeleton skel = (from trackskeleton in skeletons
                             where trackskeleton.TrackingState == SkeletonTrackingState.Tracked
                             select trackskeleton).FirstOrDefault();

            if (skel == null)
            {
                return;
            }
            if (skel.Joints[JointType.ShoulderRight].TrackingState == JointTrackingState.Tracked &&
                skel.Joints[JointType.ElbowRight].TrackingState == JointTrackingState.Tracked &&
                skel.Joints[JointType.WristRight].TrackingState == JointTrackingState.Tracked)
            {
                double[] ang = PrintAngles(skel);
                this.elb.Content = ang[0];
                this.sho1.Content = ang[1];
                this.sho2.Content = ang[2];
            }
        }
        private double[] PrintAngles(Skeleton skeleton)
        {
            Vector3D YVector = new Vector3D(0.0, 1.0, 0.0);
            Vector3D ZVector = new Vector3D(0.0, 0.0, 1.0);
            Vector3D RightShoulder = new Vector3D(skeleton.Joints[JointType.ShoulderRight].Position.X, skeleton.Joints[JointType.ShoulderRight].Position.Y, skeleton.Joints[JointType.ShoulderRight].Position.Z);
            Vector3D RightElbow = new Vector3D(skeleton.Joints[JointType.ElbowRight].Position.X, skeleton.Joints[JointType.ElbowRight].Position.Y, skeleton.Joints[JointType.ElbowRight].Position.Z);
            Vector3D RightWrist = new Vector3D(skeleton.Joints[JointType.WristRight].Position.X, skeleton.Joints[JointType.WristRight].Position.Y, skeleton.Joints[JointType.WristRight].Position.Z);
            double AngleRightElbow = AngleBetweenTwoVectors(RightElbow - RightShoulder, RightElbow - RightWrist);
            double AngleRightShoulderY = AngleBetweenTwoVectors(YVector, RightShoulder - RightElbow);
            double AngleRightShoulderZ = AngleBetweenTwoVectors(ZVector, RightShoulder - RightElbow);
            double[] angle = new double[3];
            angle[0] = AngleRightElbow;
            angle[1] = AngleRightShoulderY;
            angle[2] = AngleRightShoulderZ;
            return angle;
        }
        private double AngleBetweenTwoVectors(Vector3D vectorA, Vector3D vectorB)
        {
            double angle;
            vectorA.Normalize();
            vectorB.Normalize();
            double dp = Vector3D.DotProduct(vectorA, vectorB);
            angle = (Math.Acos(dp) / Math.PI) * 180;
            return angle;
        }
    }
}
