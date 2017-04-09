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
using System.ComponentModel;
using System.Linq;
using System.IO;
using System.Windows;
using System.Windows.Media.Media3D;
// Loading Kinect references from the Microsoft Kinect SDK
using Microsoft.Kinect;
// Loading Bluetooth references from the 32feet.net SDK
using InTheHand.Net.Sockets;
using InTheHand.Net.Bluetooth;

// Start of Code
namespace ShadowBot_Kinect
{
    // Main Window
    public partial class MainWindow : Window
    {
        private BluetoothClient bluetooth;
        private Stream bluetoothStream;
        private KinectSensor sensor;
        public MainWindow()
        {
            InitializeComponent();
        }
        // Actions performed on start of application
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Initialising Bluetooth on start of application
            // Checks if Bluetooth is on
             if (BluetoothRadio.PrimaryRadio.Mode == RadioMode.PowerOff)
            {
                lblStatus.Content = "Bluetooth is off";
                btnSearchDevice.IsEnabled = false;
            }
            // Initialising of Kinect
            // For all available Kinect sensors, choose a sensor as the one used
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }
            // Further code will be only executed if Kinect is available
            if (null != this.sensor)
            {
                // Enables Skeleton Stream from Kinect sensor
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
            // If Kinect sensor not found
            if (null == this.sensor)
            {
                MessageBox.Show("Kinect not Found!", "Error");
            }
        }
        // Actions performed on close of application
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            // Sensors and Bluetooth stream to be closed if open
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
            if (bluetoothStream != null)
            {
                bluetoothStream.Close();
                bluetoothStream.Dispose();
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
                this.elb.Content = ang[0];
                this.sho1.Content = ang[1];
                this.sho2.Content = ang[2];
            }
        }
        // Method to print certain angles in a skeleton
        private double[] PrintAngles(Skeleton skeleton)
        {
            // Unit vectors
            Vector3D XVector = new Vector3D(1.0, 0.0, 0.0);
            Vector3D YVector = new Vector3D(0.0, 1.0, 0.0);
            Vector3D ZVector = new Vector3D(0.0, 0.0, 1.0);
            // Major required vectors within the skeleton
            Vector3D ShoulderCenter = new Vector3D(skeleton.Joints[JointType.ShoulderCenter].Position.X, skeleton.Joints[JointType.ShoulderCenter].Position.Y, skeleton.Joints[JointType.ShoulderCenter].Position.Z);
            Vector3D LeftShoulder = new Vector3D(skeleton.Joints[JointType.ShoulderLeft].Position.X, skeleton.Joints[JointType.ShoulderLeft].Position.Y, skeleton.Joints[JointType.ShoulderLeft].Position.Z);
            Vector3D RightShoulder = new Vector3D(skeleton.Joints[JointType.ShoulderRight].Position.X, skeleton.Joints[JointType.ShoulderRight].Position.Y, skeleton.Joints[JointType.ShoulderRight].Position.Z);
            Vector3D LeftElbow = new Vector3D(skeleton.Joints[JointType.ElbowLeft].Position.X, skeleton.Joints[JointType.ElbowLeft].Position.Y, skeleton.Joints[JointType.ElbowLeft].Position.Z);
            Vector3D RightElbow = new Vector3D(skeleton.Joints[JointType.ElbowRight].Position.X, skeleton.Joints[JointType.ElbowRight].Position.Y, skeleton.Joints[JointType.ElbowRight].Position.Z);
            Vector3D LeftWrist = new Vector3D(skeleton.Joints[JointType.WristLeft].Position.X, skeleton.Joints[JointType.WristLeft].Position.Y, skeleton.Joints[JointType.WristLeft].Position.Z);
            Vector3D RightWrist = new Vector3D(skeleton.Joints[JointType.WristRight].Position.X, skeleton.Joints[JointType.WristRight].Position.Y, skeleton.Joints[JointType.WristRight].Position.Z);
            Vector3D LeftHip = new Vector3D(skeleton.Joints[JointType.HipLeft].Position.X, skeleton.Joints[JointType.HipLeft].Position.Y, skeleton.Joints[JointType.HipLeft].Position.Z);
            Vector3D RightHip = new Vector3D(skeleton.Joints[JointType.HipRight].Position.X, skeleton.Joints[JointType.HipRight].Position.Y, skeleton.Joints[JointType.HipRight].Position.Z);
            Vector3D LeftKnee = new Vector3D(skeleton.Joints[JointType.KneeLeft].Position.X, skeleton.Joints[JointType.KneeLeft].Position.Y, skeleton.Joints[JointType.KneeLeft].Position.Z);
            Vector3D RigtKnee = new Vector3D(skeleton.Joints[JointType.KneeRight].Position.X, skeleton.Joints[JointType.KneeRight].Position.Y, skeleton.Joints[JointType.KneeRight].Position.Z);
            Vector3D LeftAnkle = new Vector3D(skeleton.Joints[JointType.AnkleLeft].Position.X, skeleton.Joints[JointType.AnkleLeft].Position.Y, skeleton.Joints[JointType.AnkleLeft].Position.Z);
            Vector3D RightAnkle = new Vector3D(skeleton.Joints[JointType.AnkleRight].Position.X, skeleton.Joints[JointType.AnkleRight].Position.Y, skeleton.Joints[JointType.AnkleRight].Position.Z);
            // Major joint angles
            double AngleRightElbow = AngleBetweenTwoVectors(RightElbow - RightShoulder, RightElbow - RightWrist);
            double AngleRightShoulderY = AngleBetweenTwoVectors(YVector, RightShoulder - RightElbow);
            double AngleRightShoulderZ = AngleBetweenTwoVectors(ZVector, RightShoulder - RightElbow);
            // Joint angle array to be returned to the calling statement
            double[] angle = new double[3];
            angle[0] = AngleRightElbow;
            angle[1] = AngleRightShoulderY;
            angle[2] = AngleRightShoulderZ;
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
        // Method called when "Search for Devices" button is clicked
        private void btnSearchDevice_Click(object sender, RoutedEventArgs e)
        {
            lblStatus.Content = "Searching for devices...";
            btnSearchDevice.IsEnabled = false;
            BackgroundWorker bwDiscoverDevices = new BackgroundWorker();
            // New event subscribed to search for the HC-05 module of the Arduino
            bwDiscoverDevices.DoWork += new DoWorkEventHandler(bwDiscoverDevices_DoWork);
            // New event subscribed to check if module is connected
            bwDiscoverDevices.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bwDiscoverDevices_RunWorkerCompleted);
            bwDiscoverDevices.RunWorkerAsync();
        }
        // Method to search for the bluetooth module of Arduino
        private void bwDiscoverDevices_DoWork(object sender, DoWorkEventArgs e)
        {
            try
            {
                // Initialise new bluetooth client
                bluetooth = new BluetoothClient();
                // Search for the given device and selects the first available device with the name
                var device = bluetooth.DiscoverDevices().Where(d => d.DeviceName == "Moto").FirstOrDefault();
                // If device found, connects to the bluetooth device 
                if (device != null)
                {
                    bluetooth.Connect(device.DeviceAddress, BluetoothService.SerialPort);
                    // Sets stream as the available bluetooth stream
                    bluetoothStream = bluetooth.GetStream();
                    e.Result = true;
                }
                else
                {
                    e.Result = false;
                }
            }
            catch (Exception)
            {
                e.Result = false;
            }
        }
        // Checks if device is connected via bluetooth
        private void bwDiscoverDevices_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            btnSearchDevice.IsEnabled = true;
            // Checks if device is connected
            var deviceFound = (bool)e.Result;
            if (!deviceFound)
            {
                lblStatus.Content = "No device found";
            }
            // Returns status as connected and allows data to be sent
            else
            {
                lblStatus.Content = "Connected to Moto";
                btnSendMessage.IsEnabled = true;
            }
        }
        // Sends data upon click of send button
        private void btnSendMessage_Click(object sender, RoutedEventArgs e)
        {
            // Checks if bluetooth connection is up and stream is working
            if (bluetooth.Connected && bluetoothStream != null)
            {
                var buffer1 = System.Text.Encoding.UTF8.GetBytes((string)elb.Content);
                bluetoothStream.Write(buffer1, 0, buffer1.Length);
                var buffer2 = System.Text.Encoding.UTF8.GetBytes((string)sho1.Content);
                bluetoothStream.Write(buffer2, 0, buffer2.Length);
                var buffer3 = System.Text.Encoding.UTF8.GetBytes((string)sho2.Content);
                bluetoothStream.Write(buffer3, 0, buffer3.Length);
            }
        }
    }
}
