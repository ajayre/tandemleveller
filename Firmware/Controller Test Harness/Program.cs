using SerialTransfer;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;

namespace Controller_Test_Harness
{
    internal class Program
    {
        enum PGNValues
        {
            // misc
            PGN_ESTOP                    = 0x0000,
            PGN_RESET                    = 0x0001,
            PGN_OG3D_STARTED             = 0x0002,

            // blade control
            PGN_CUT_VALVE                = 0x1000,   // 0 - 200
            PGN_BLADE_OFFSET             = 0x1001,

            // blade configuration
            PGN_FRONT_PWM_GAIN_UP        = 0x2002,
            PGN_FRONT_PWM_GAIN_DOWN      = 0x2003,
            PGN_FRONT_PWM_MIN_UP         = 0x2004,
            PGN_FRONT_PWM_MIN_DOWN       = 0x2005,
            PGN_FRONT_PWM_MAX_UP         = 0x2006,
            PGN_FRONT_PWM_MAX_DOWN       = 0x2007,
            PGN_FRONT_INTEGRAL_MULTPLIER = 0x2008,
            PGN_FRONT_DEADBAND           = 0x2009,
            PGN_REAR_PWM_GAIN_UP         = 0x200A,
            PGN_REAR_PWM_GAIN_DOWN       = 0x200B,
            PGN_REAR_PWM_MIN_UP          = 0x200C,
            PGN_REAR_PWM_MIN_DOWN        = 0x200D,
            PGN_REAR_PWM_MAX_UP          = 0x200E,
            PGN_REAR_PWM_MAX_DOWN        = 0x200F,
            PGN_REAR_INTEGRAL_MULTPLIER  = 0x2010,
            PGN_REAR_DEADBAND            = 0x2011,

            // autosteer control
            PGN_AUTOSTEER_RELAY          = 0x3000,
            PGN_AUTOSTEER_SPEED          = 0x3001,
            PGN_AUTOSTEER_DISTANCE       = 0x3002,
            PGN_AUTOSTEER_ANGLE          = 0x3003,

            // autosteer configuration
            PGN_AUTOSTEER_KP             = 0x4000,
            PGN_AUTOSTEER_KI             = 0x4001,
            PGN_AUTOSTEER_KD             = 0x4002,
            PGN_AUTOSTEER_KO             = 0x4003,
            PGN_AUTOSTEER_OFFSET         = 0x4004,
            PGN_AUTOSTEER_MIN_PWM        = 0x4005,
            PGN_AUTOSTEER_MAX_INTEGRAL   = 0x4006,
            PGN_AUTOSTEER_COUNTS_PER_DEG = 0x4007,

            // controller status
            PGN_FRONT_BLADE_OFFSET_SLAVE = 0x5000,
            PGN_FRONT_BLADE_PWMVALUE     = 0x5001,
            PGN_FRONT_BLADE_DIRECTION    = 0x5002,
            PGN_FRONT_BLADE_AUTO         = 0x5003,
            PGN_REAR_BLADE_OFFSET_SLAVE  = 0x5004,
            PGN_REAR_BLADE_PWMVALUE      = 0x5005,
            PGN_REAR_BLADE_DIRECTION     = 0x5006,
            PGN_REAR_BLADE_AUTO          = 0x5007
        }

        struct ControllerCommand
        {
            public PGNValues PGN;
            public UInt32 Value;
        }

        struct ControllerStatus
        {
            public PGNValues PGN;
            public UInt32 Value;
        }

        private static SerialTransfer.SerialTransfer Controller;

        static void Main(string[] args)
        {
            // Create and configure SerialPort
            SerialPort port = new SerialPort
            {
                PortName = "COM12",
                BaudRate = 38400,
                DataBits = 8,
                Parity = Parity.None,
                StopBits = StopBits.One
            };

            Controller = new SerialTransfer.SerialTransfer();
            Controller.Begin(port);

            // tell controller we are now running
            SendControllerCommand(PGNValues.PGN_OG3D_STARTED, 0);

            // send configuration
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_GAIN_UP, 4);
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_GAIN_DOWN, 3);
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_MIN_UP, 50);
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_MIN_DOWN, 50);
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_MAX_UP, 180);
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_MAX_DOWN, 180);
            SendControllerCommand(PGNValues.PGN_FRONT_INTEGRAL_MULTPLIER, 20);
            SendControllerCommand(PGNValues.PGN_FRONT_DEADBAND, 3);

            SendControllerCommand(PGNValues.PGN_REAR_PWM_GAIN_UP, 4);
            SendControllerCommand(PGNValues.PGN_REAR_PWM_GAIN_DOWN, 3);
            SendControllerCommand(PGNValues.PGN_REAR_PWM_MIN_UP, 50);
            SendControllerCommand(PGNValues.PGN_REAR_PWM_MIN_DOWN, 50);
            SendControllerCommand(PGNValues.PGN_REAR_PWM_MAX_UP, 180);
            SendControllerCommand(PGNValues.PGN_REAR_PWM_MAX_DOWN, 180);
            SendControllerCommand(PGNValues.PGN_REAR_INTEGRAL_MULTPLIER, 20);
            SendControllerCommand(PGNValues.PGN_REAR_DEADBAND, 3);

            DateTime TxTime = DateTime.Now.AddMilliseconds(500);

            while (true)
            {
                if (TxTime < DateTime.Now)
                {
                    TxTime = DateTime.Now.AddMilliseconds(500);

                    ControllerCommand TxCmd;
                    TxCmd.PGN = PGNValues.PGN_CUT_VALVE;
                    TxCmd.Value = 90;
                    SendControllerCommand(TxCmd);
                }

                if (Controller.Available() > 0)
                {
                    ControllerStatus Stat = GetControllerStatus();
                    switch (Stat.PGN)
                    {
                        case PGNValues.PGN_ESTOP:
                            Console.WriteLine("EMERGENCY STOP!");
                            break;

                        case PGNValues.PGN_FRONT_BLADE_OFFSET_SLAVE:
                            Console.WriteLine("Front blade offset (slave): " + ((int)(Stat.Value)).ToString());
                            break;

                        case PGNValues.PGN_REAR_BLADE_OFFSET_SLAVE:
                            Console.WriteLine("Rear blade offset (slave): " + ((int)Stat.Value).ToString());
                            break;
                    }
                }
            }
        }

        /// <summary>
        /// Resets the controller which in turn will reset all nodes on the network
        /// WARNING: this will cause the COM port to disappear and then return
        /// </summary>
        private static void ResetController
            (
            )
        {
            SendControllerCommand(PGNValues.PGN_RESET, 0);
        }

        private static void SendControllerCommand
            (
            PGNValues PGN,
            UInt32 Value
            )
        {
            ControllerCommand Cmd;
            Cmd.PGN = PGN;
            Cmd.Value = Value;
            SendControllerCommand(Cmd);
        }

        private static void SendControllerCommand
            (
            ControllerCommand Cmd
            )
        {
            Controller.Packet.TxBuff[0] = (byte)((UInt16)Cmd.PGN & 0xFF);
            Controller.Packet.TxBuff[1] = (byte)(((UInt16)Cmd.PGN >> 8) & 0xFF);
            Controller.Packet.TxBuff[2] = (byte)(Cmd.Value & 0xFF);
            Controller.Packet.TxBuff[3] = (byte)((Cmd.Value >> 8) & 0xFF);
            Controller.Packet.TxBuff[4] = (byte)((Cmd.Value >> 16) & 0xFF);
            Controller.Packet.TxBuff[5] = (byte)((Cmd.Value >> 24) & 0xFF);
            Controller.SendData(6);
            Console.WriteLine(string.Format("Tx: {0}:0x{1:X8}", Cmd.PGN, Cmd.Value));
        }

        private static ControllerStatus GetControllerStatus
            (
            )
        {
            ControllerStatus Stat;

            Stat.PGN = (PGNValues)(((UInt16)Controller.Packet.RxBuff[1] << 8) | Controller.Packet.RxBuff[0]);
            Stat.Value = (UInt32)(((UInt32)Controller.Packet.RxBuff[5] << 24) |
                ((UInt32)Controller.Packet.RxBuff[4] << 16) |
                ((UInt32)Controller.Packet.RxBuff[3] << 8) |
                Controller.Packet.RxBuff[2]);

            Console.WriteLine(string.Format("Rx: {0}:0x{1:X8}", Stat.PGN, Stat.Value));

            return Stat;
        }
    }
}
