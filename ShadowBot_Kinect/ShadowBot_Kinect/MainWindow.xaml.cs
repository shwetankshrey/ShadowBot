/*
 * Code made by Shwetank Shrey
 * Project - Introduction to Engg Design
 * Shadow Bot
 * Description - This project aims to create Humanoid Robots who will replicate the actions of 
 * a person standing in front of a Kinect sensors.
 * The following code is the WPF application which records joint angles from a Kinect sensor and
 * sends it to the Arduino via Bluetooth.
 */

 /*
  * Issues remaining:
  * Work on bluetooth remaining
  */

// Loading basic C# references used
using System;
using System.Linq;
using System.IO;
using System.Windows;
using System.Windows.Media.Media3D;
// Loading Kinect references from the Microsoft Kinect SDK
using Microsoft.Kinect;

// Start of Code
namespace ShadowBot_Kinect
{
    // Main Window
    public partial class MainWindow : Window
    {
        private KinectSensor sensor;
        public MainWindow()
        {
            InitializeComponent();
        }
        // Actions performed on start of application
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Initialising of Kinect
            // For all available Kinect sensors, choose a sensor as the one used
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    sensor = potentialSensor;
                    break;
                }
            }
            // Further code will be only executed if Kinect is available
            if (null != sensor)
            {
                // Enables Skeleton Stream from Kinect sensor
                sensor.SkeletonStream.Enable();
                sensor.SkeletonFrameReady += Sensor_SkeletonFrameReady;
                try
                {
                    sensor.Start();
                }
                catch (IOException)
                {
                    sensor = null;
                }
            }
            // If Kinect sensor not found
            if (null == sensor)
            {
                MessageBox.Show("Kinect not Found!", "Error");
            }
        }
        // Actions performed on close of application
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            // Sensors to be closed if open
            if (null != sensor)
            {
                sensor.Stop();
            }
        }
        // Skeleton Stream Actions
        private void Sensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            // Initialise skeleton array to store available skeletons (maximum 6)
            Skeleton[] skeletons = new Skeleton[6];
            // Store skeletons from available skeleton frame to the array
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame == null)
                {
                    return;
                }
                skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                skeletonFrame.CopySkeletonDataTo(skeletons);
            }
            // Choose the first/nearest skeleton as the main skeleton
            Skeleton skel = (from trackskeleton in skeletons
                             where trackskeleton.TrackingState == SkeletonTrackingState.Tracked
                             select trackskeleton).FirstOrDefault();
            // Code to go ahead if any skeletons are available
            if (skel == null)
            {
                return;
            }
            // If skeletons are being tracked in real time in synchronisation, process angles
            if (skel.Joints[JointType.ShoulderRight].TrackingState == JointTrackingState.Tracked &&
                skel.Joints[JointType.ElbowRight].TrackingState == JointTrackingState.Tracked &&
                skel.Joints[JointType.WristRight].TrackingState == JointTrackingState.Tracked)
            {
                // Print angles on the WPF application's labels
                double[] ang = PrintAngles(skel);
                ls1.Content = ang[0];
                ls2.Content = ang[1];
                le.Content = ang[2];
                rs1.Content = ang[3];
                rs2.Content = ang[4];
                re.Content = ang[5];
                lh.Content = ang[6];
                rh.Content = ang[7];
                lk.Content = ang[8];
                lk.Content = ang[9];
            }
        }
        // Method to print certain angles in a skeleton
        private double[] PrintAngles(Skeleton skeleton)
        {
            // Unit vectors
            Vector3D XVector = new Vector3D(1.0, 0.0, 0.0);
            Vector3D YVector = new Vector3D(0.0, -1.0, 0.0);
            Vector3D ZVector = new Vector3D(0.0, 0.0, 1.0);
            // Major required vectors within the skeleton
            Vector3D LeftShoulder = new Vector3D(skeleton.Joints[JointType.ShoulderLeft].Position.X, skeleton.Joints[JointType.ShoulderLeft].Position.Y, skeleton.Joints[JointType.ShoulderLeft].Position.Z);
            Vector3D LeftShoulderXY = new Vector3D(skeleton.Joints[JointType.ShoulderLeft].Position.X, skeleton.Joints[JointType.ShoulderLeft].Position.Y, 0);
            Vector3D LeftShoulderYZ = new Vector3D(0, skeleton.Joints[JointType.ShoulderLeft].Position.Y, skeleton.Joints[JointType.ShoulderLeft].Position.Z);
            Vector3D RightShoulder = new Vector3D(skeleton.Joints[JointType.ShoulderRight].Position.X, skeleton.Joints[JointType.ShoulderRight].Position.Y, skeleton.Joints[JointType.ShoulderRight].Position.Z);
            Vector3D RightShoulderXY = new Vector3D(skeleton.Joints[JointType.ShoulderRight].Position.X, skeleton.Joints[JointType.ShoulderRight].Position.Y, 0);
            Vector3D RightShoulderYZ = new Vector3D(0, skeleton.Joints[JointType.ShoulderRight].Position.Y, skeleton.Joints[JointType.ShoulderRight].Position.Z);
            Vector3D LeftElbow = new Vector3D(skeleton.Joints[JointType.ElbowLeft].Position.X, skeleton.Joints[JointType.ElbowLeft].Position.Y, skeleton.Joints[JointType.ElbowLeft].Position.Z);
            Vector3D LeftElbowXY = new Vector3D(skeleton.Joints[JointType.ElbowLeft].Position.X, skeleton.Joints[JointType.ElbowLeft].Position.Y, 0);
            Vector3D LeftElbowYZ = new Vector3D(0, skeleton.Joints[JointType.ElbowLeft].Position.Y, skeleton.Joints[JointType.ElbowLeft].Position.Z);
            Vector3D RightElbow = new Vector3D(skeleton.Joints[JointType.ElbowRight].Position.X, skeleton.Joints[JointType.ElbowRight].Position.Y, skeleton.Joints[JointType.ElbowRight].Position.Z);
            Vector3D RightElbowXY = new Vector3D(skeleton.Joints[JointType.ElbowRight].Position.X, skeleton.Joints[JointType.ElbowRight].Position.Y, 0);
            Vector3D RightElbowYZ = new Vector3D(0, skeleton.Joints[JointType.ElbowRight].Position.Y, skeleton.Joints[JointType.ElbowRight].Position.Z);
            Vector3D LeftWrist = new Vector3D(skeleton.Joints[JointType.WristLeft].Position.X, skeleton.Joints[JointType.WristLeft].Position.Y, skeleton.Joints[JointType.WristLeft].Position.Z);
            Vector3D RightWrist = new Vector3D(skeleton.Joints[JointType.WristRight].Position.X, skeleton.Joints[JointType.WristRight].Position.Y, skeleton.Joints[JointType.WristRight].Position.Z);
            Vector3D LeftHip = new Vector3D(0, skeleton.Joints[JointType.HipLeft].Position.Y, skeleton.Joints[JointType.HipLeft].Position.Z);
            Vector3D RightHip = new Vector3D(0, skeleton.Joints[JointType.HipRight].Position.Y, skeleton.Joints[JointType.HipLeft].Position.Z);
            Vector3D LeftKnee = new Vector3D(0, skeleton.Joints[JointType.KneeLeft].Position.Y, skeleton.Joints[JointType.KneeLeft].Position.Z);
            Vector3D RightKnee = new Vector3D(0, skeleton.Joints[JointType.KneeRight].Position.Y, skeleton.Joints[JointType.KneeRight].Position.Z);
            Vector3D LeftAnkle = new Vector3D(0, skeleton.Joints[JointType.AnkleLeft].Position.Y, skeleton.Joints[JointType.AnkleLeft].Position.Z);
            Vector3D RightAnkle = new Vector3D(0, skeleton.Joints[JointType.AnkleRight].Position.Y, skeleton.Joints[JointType.AnkleRight].Position.Z);
            // Major joint angles
            double AngleLeftShoulder1 = AngleBetweenTwoVectors(LeftShoulderXY - LeftElbowXY, YVector);
            double AngleLeftShoulder2 = AngleBetweenTwoVectors(LeftShoulderYZ - LeftElbowYZ, YVector);
            double AngleRightShoulder1 = AngleBetweenTwoVectors(RightShoulderXY - RightElbowXY, YVector);
            double AngleRightShoulder2 = AngleBetweenTwoVectors(RightShoulderYZ - RightElbowYZ, YVector);
            double AngleLeftElbow = AngleBetweenTwoVectors(LeftElbow - LeftShoulder, LeftElbow - LeftWrist);
            double AngleRightElbow = AngleBetweenTwoVectors(RightElbow - RightShoulder, RightElbow - RightWrist);
            double AngleLeftHip = AngleBetweenTwoVectors(LeftHip - LeftKnee, YVector);
            double AngleRightHip = AngleBetweenTwoVectors(RightHip - RightKnee, YVector);
            double AngleLeftKnee = AngleBetweenTwoVectors(LeftKnee - LeftHip, LeftKnee - LeftAnkle);
            double AngleRightKnee = AngleBetweenTwoVectors(RightKnee - RightHip, RightKnee - RightAnkle);
            // Joint angle array to be returned to the calling statement
            double[] angle = new double[10];
            angle[0] = AngleLeftShoulder1;
            angle[1] = AngleLeftShoulder2;
            angle[2] = AngleLeftElbow;
            angle[3] = AngleRightShoulder1;
            angle[4] = AngleRightShoulder2;
            angle[5] = AngleRightElbow;
            angle[6] = AngleLeftHip;
            angle[7] = AngleLeftKnee;
            angle[8] = AngleRightHip;
            angle[9] = AngleRightKnee;
            return angle;
        }
        // Method to calculate angle between two 3D vectors
        private double AngleBetweenTwoVectors(Vector3D vectorA, Vector3D vectorB)
        {
            // Calculates angles by normalising vectors to unit vectors and computing the cosine using dot products
            double angle;
            vectorA.Normalize();
            vectorB.Normalize();
            double dp = Vector3D.DotProduct(vectorA, vectorB);
            angle = (Math.Acos(dp) / Math.PI) * 180;
            return angle;
        }
    }
}