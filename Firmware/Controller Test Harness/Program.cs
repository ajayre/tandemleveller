using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using Controller;
using System.Threading;

namespace Controller_Test_Harness
{
    internal class Program
    {
        static void Main(string[] args)
        {
            //SensorFusor_Tests tests = new SensorFusor_Tests();
            //bool allPassed = tests.Run();

            OGController Controller = new OGController();
            Controller.OnControllerLost += Controller_OnControllerLost;
            Controller.OnEmergencyStop += Controller_OnEmergencyStop;
            Controller.OnFrontSlaveOffsetChanged += Controller_OnFrontSlaveOffsetChanged;
            Controller.OnRearSlaveOffsetChanged += Controller_OnRearSlaveOffsetChanged;
            Controller.OnFrontBladeAutoChanged += Controller_OnFrontBladeAutoChanged;
            Controller.OnRearBladeAutoChanged += Controller_OnRearBladeAutoChanged;
            Controller.OnFrontBladeDirectionChanged += Controller_OnFrontBladeDirectionChanged;
            Controller.OnRearBladeDirectionChanged += Controller_OnRearBladeDirectionChanged;
            Controller.OnTractorIMUChanged += Controller_OnTractorIMUChanged;
            Controller.OnFrontIMUChanged += Controller_OnFrontIMUChanged;
            Controller.OnRearIMUChanged += Controller_OnRearIMUChanged;
            Controller.OnFrontBladeHeightChanged += Controller_OnFrontBladeHeightChanged;
            Controller.OnRearBladeHeightChanged += Controller_OnRearBladeHeightChanged;

            Controller.Connect("COM12");

            BladeConfiguration FrontBladeConfig = new BladeConfiguration();
            FrontBladeConfig.PWMGainUp          = 4;
            FrontBladeConfig.PWMGainDown        = 3;
            FrontBladeConfig.PWMMinUp           = 50;
            FrontBladeConfig.PWMMinDown         = 50;
            FrontBladeConfig.PWMMaxUp           = 180;
            FrontBladeConfig.PWMMaxDown         = 180;
            FrontBladeConfig.IntegralMultiplier = 20;
            FrontBladeConfig.Deadband           = 3;
            Controller.SetFrontBladeConfiguration(FrontBladeConfig);

            BladeConfiguration RearBladeConfig = new BladeConfiguration();
            RearBladeConfig.PWMGainUp          = 4;
            RearBladeConfig.PWMGainDown        = 3;
            RearBladeConfig.PWMMinUp           = 50;
            RearBladeConfig.PWMMinDown         = 50;
            RearBladeConfig.PWMMaxUp           = 180;
            RearBladeConfig.PWMMaxDown         = 180;
            RearBladeConfig.IntegralMultiplier = 20;
            RearBladeConfig.Deadband           = 3;
            Controller.SetRearBladeConfiguration(RearBladeConfig);

            Controller.FrontBladeAtZero();
            Controller.RearBladeAtZero();

            DateTime TxTime = DateTime.Now.AddMilliseconds(500);

            while (true)
            {
                if (TxTime < DateTime.Now)
                {
                    TxTime = DateTime.Now.AddMilliseconds(500);

                    Controller.SetFrontCutValve(90);
                }

                Thread.Sleep(10);
            }
        }

        /// <summary>
        /// Raised when the rear blade height changes
        /// </summary>
        /// <param name="Height">New blade height</param>
        private static void Controller_OnRearBladeHeightChanged(int Height)
        {
            Console.WriteLine(string.Format("Rear blade height: {0} mm", Height));
        }

        /// <summary>
        /// Raised when the front blade height changes
        /// </summary>
        /// <param name="Height">New blade height</param>
        private static void Controller_OnFrontBladeHeightChanged
            (
            int Height
            )
        {
            Console.WriteLine(string.Format("Front blade height: {0} mm", Height));
        }

        /// <summary>
        /// Raised when the rear scraper IMU has changed
        /// </summary>
        /// <param name="Value">New IMU values</param>
        private static void Controller_OnRearIMUChanged
            (
            IMUValue Value
            )
        {
            //Console.WriteLine("Rear scraper pitch: {0:0.00} deg, roll: {1:0.00} deg, yaw: {2:0.00} deg", Value.Pitch, Value.Roll, Value.Yaw);
        }

        /// <summary>
        /// Raised when the front scraper IMU has changed
        /// </summary>
        /// <param name="Value">New IMU values</param>
        private static void Controller_OnFrontIMUChanged
            (
            IMUValue Value
            )
        {
            //Console.WriteLine("Front scraper pitch: {0:0.00} deg, roll: {1:0.00} deg, yaw: {2:0.00} deg", Value.Pitch, Value.Roll, Value.Yaw);
        }

        /// <summary>
        /// Raised when the tractor IMU has changed
        /// </summary>
        /// <param name="Value">New IMU values</param>
        private static void Controller_OnTractorIMUChanged
            (
            IMUValue Value
            )
        {
            //Console.WriteLine("Tractor pitch: {0:0.00} deg, roll: {1:0.00} deg, yaw: {2:0.00} deg", Value.Pitch, Value.Roll, Value.Yaw);
        }

        /// <summary>
        /// Raised when the rear blade changes direction
        /// </summary>
        /// <param name="IsMovingUp">true if blade is moving up</param>
        private static void Controller_OnRearBladeDirectionChanged
            (
            bool IsMovingUp
            )
        {
            Console.WriteLine("Rear blade moving: {0}", IsMovingUp? "Up" : "Down");
        }

        /// <summary>
        /// Raised when the front blade changes direction
        /// </summary>
        /// <param name="IsMovingUp">true if blade is moving up</param>
        private static void Controller_OnFrontBladeDirectionChanged
            (
            bool IsMovingUp
            )
        {
            Console.WriteLine("Front blade moving: {0}", IsMovingUp ? "Up" : "Down");
        }

        /// <summary>
        /// Raised when rear blade has entered or exited auto mode
        /// </summary>
        /// <param name="IsAuto">true if blade is in auto mode</param>
        private static void Controller_OnRearBladeAutoChanged
            (
            bool IsAuto
            )
        {
            Console.WriteLine("Rear blade auto: {0}", IsAuto);
        }

        /// <summary>
        /// Raised when front blade has entered or exited auto mode
        /// </summary>
        /// <param name="IsAuto">true if blade is in auto mode</param>
        private static void Controller_OnFrontBladeAutoChanged
            (
            bool IsAuto
            )
        {
            Console.WriteLine("Front blade auto: {0}", IsAuto);
        }

        /// <summary>
        /// Raised when the rear slave offset is changed
        /// </summary>
        /// <param name="Offset">New offset</param>
        private static void Controller_OnRearSlaveOffsetChanged
            (
            int Offset
            )
        {
            Console.WriteLine("Rear blade offset (slave): " + Offset.ToString());
        }

        /// <summary>
        /// Raised when the front slave offset is changed
        /// </summary>
        /// <param name="Offset">New offset</param>
        private static void Controller_OnFrontSlaveOffsetChanged
            (
            int Offset
            )
        {
            Console.WriteLine("Front blade offset (slave): " + Offset.ToString());
        }

        /// <summary>
        /// Raised when an emergency stop happens
        /// </summary>
        private static void Controller_OnEmergencyStop()
        {
            Console.WriteLine("EMERGENCY STOP!");
        }

        /// <summary>
        /// Raised when controller has stopped responding
        /// </summary>
        private static void Controller_OnControllerLost()
        {
            Console.WriteLine("CONTROLLER NOT FOUND");
        }
    }
}
