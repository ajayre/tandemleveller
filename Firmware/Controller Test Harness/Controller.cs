using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.ComponentModel;
using System.Runtime.InteropServices;

namespace Controller
{
    public struct BladeConfiguration
    {
        public uint PWMGainUp;
        public uint PWMGainDown;
        public uint PWMMinUp;
        public uint PWMMinDown;
        public uint PWMMaxUp;
        public uint PWMMaxDown;
        public uint IntegralMultiplier;
        public uint Deadband;
    };

    public struct IMUValue
    {
        public double Pitch;
        public double Yaw;
        public double Roll;
    }

    public class OGController
    {
        public const uint CUTVALVE_MIN = 0;
        public const uint CUTVALVE_MAX = 200;

        public delegate void ControllerLost();
        public event ControllerLost OnControllerLost = null;

        public delegate void EmergencyStop();
        public event EmergencyStop OnEmergencyStop = null;

        public delegate void SlaveOffsetChanged(int Offset);
        public event SlaveOffsetChanged OnFrontSlaveOffsetChanged = null;
        public event SlaveOffsetChanged OnRearSlaveOffsetChanged = null;

        public delegate void BladeAutoChanged(bool IsAuto);
        public event BladeAutoChanged OnFrontBladeAutoChanged = null;
        public event BladeAutoChanged OnRearBladeAutoChanged = null;

        public delegate void BladeDirectionChanged(bool IsMovingUp);
        public event BladeDirectionChanged OnFrontBladeDirectionChanged = null;
        public event BladeDirectionChanged OnRearBladeDirectionChanged = null;

        public delegate void IMUChanged(IMUValue Value);
        public event IMUChanged OnTractorIMUChanged = null;
        public event IMUChanged OnFrontIMUChanged = null;
        public event IMUChanged OnRearIMUChanged = null;

        public delegate void BladeHeightChanged(int Height);
        public event BladeHeightChanged OnFrontBladeHeightChanged = null;
        public event BladeHeightChanged OnRearBladeHeightChanged = null;

        private enum PGNValues
        {
            // misc
            PGN_ESTOP                    = 0x0000,
            PGN_RESET                    = 0x0001,
            PGN_OG3D_STARTED             = 0x0002,
            PGN_PING                     = 0x0003,

            // blade control
            PGN_FRONT_CUT_VALVE          = 0x1000,   // CUTVALVE_MIN -> CUTVALVE_MAX
            PGN_REAR_CUT_VALVE           = 0x1001,   // CUTVALVE_MIN -> CUTVALVE_MAX
            PGN_FRONT_ZERO_BLADE_HEIGHT  = 0x1002,
            PGN_REAR_ZERO_BLADE_HEIGHT   = 0x1003,

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

            // blade status
            PGN_FRONT_BLADE_OFFSET_SLAVE = 0x5000,
            PGN_FRONT_BLADE_PWMVALUE     = 0x5001,
            PGN_FRONT_BLADE_DIRECTION    = 0x5002,
            PGN_FRONT_BLADE_AUTO         = 0x5003,
            PGN_REAR_BLADE_OFFSET_SLAVE  = 0x5004,
            PGN_REAR_BLADE_PWMVALUE      = 0x5005,
            PGN_REAR_BLADE_DIRECTION     = 0x5006,
            PGN_REAR_BLADE_AUTO          = 0x5007,
            PGN_FRONT_BLADE_HEIGHT       = 0x5008,
            PGN_REAR_BLADE_HEIGHT        = 0x5009,

            // IMU
            PGN_TRACTOR_PITCH            = 0x6000,
            PGN_TRACTOR_ROLL             = 0x6001,
            PGN_TRACTOR_YAW              = 0x6002,
            PGN_FRONT_PITCH              = 0x6003,
            PGN_FRONT_ROLL               = 0x6004,
            PGN_FRONT_YAW                = 0x6005,
            PGN_REAR_PITCH               = 0x6006,
            PGN_REAR_ROLL                = 0x6007,
            PGN_REAR_YAW                 = 0x6008,
        }

        private struct ControllerCommand
        {
            public PGNValues PGN;
            public UInt32 Value;
        }

        private struct ControllerStatus
        {
            public PGNValues PGN;
            public UInt32 Value;
        }

        // time between transmit of pings in milliseconds
        private const int PING_PERIOD_MS = 1000;

        // maximum time to wait for a ping before determining controller has stopped working
        private const int PING_TIMEOUT_PERIOD_MS = 3000;

        // speed of communication with controller
        private const int BAUDRATE = 38400;

        private SerialTransfer ControllerChannel;
        private IMUValue TractorIMU = new IMUValue();
        private IMUValue FrontScraperIMU = new IMUValue();
        private IMUValue RearScraperIMU = new IMUValue();

        private DateTime LastRxPingTime;
        private bool ControllerFound;
        private SerialPort Port;
        private BackgroundWorker WorkThread = null;
        private DateTime PingTime;

        /// <summary>
        /// Connect to controller
        /// </summary>
        /// <param name="SerialPort">COM port to use e.g. "COM1"</param>
        public void Connect
            (
            string SerialPort
            )
        {
            // Create and configure SerialPort
            Port = new SerialPort
            {
                PortName = SerialPort,
                BaudRate = BAUDRATE,
                DataBits = 8,
                Parity = Parity.None,
                StopBits = StopBits.One
            };

            ControllerChannel = new SerialTransfer();
            ControllerChannel.Begin(Port);

            // tell controller we are now running
            SendControllerCommand(PGNValues.PGN_OG3D_STARTED, 0);

            PingTime = DateTime.Now.AddMilliseconds(PING_PERIOD_MS);

            LastRxPingTime = DateTime.Now;
            ControllerFound = true;

            WorkThread = new BackgroundWorker();
            WorkThread.DoWork += WorkThread_DoWork;
            WorkThread.WorkerSupportsCancellation = true;
            WorkThread.RunWorkerAsync();
        }

        public void Disconnect
            (
            )
        {
            if (WorkThread != null)
            {
                WorkThread.CancelAsync();
            }
            WorkThread = null;
        }

        /// <summary>
        /// Sets the front blade configuration
        /// </summary>
        /// <param name="Config">New configuration</param>
        public void SetFrontBladeConfiguration
            (
            BladeConfiguration Config
            )
        {
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_GAIN_UP,        Config.PWMGainUp);
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_GAIN_DOWN,      Config.PWMGainDown);
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_MIN_UP,         Config.PWMMinUp);
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_MIN_DOWN,       Config.PWMMinDown);
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_MAX_UP,         Config.PWMMaxUp);
            SendControllerCommand(PGNValues.PGN_FRONT_PWM_MAX_DOWN,       Config.PWMMaxDown);
            SendControllerCommand(PGNValues.PGN_FRONT_INTEGRAL_MULTPLIER, Config.IntegralMultiplier);
            SendControllerCommand(PGNValues.PGN_FRONT_DEADBAND,           Config.Deadband);
        }

        /// <summary>
        /// Sets the rear blade configuration
        /// </summary>
        /// <param name="Config">New configuration</param>
        public void SetRearBladeConfiguration
            (
            BladeConfiguration Config
            )
        {
            SendControllerCommand(PGNValues.PGN_REAR_PWM_GAIN_UP,        Config.PWMGainUp);
            SendControllerCommand(PGNValues.PGN_REAR_PWM_GAIN_DOWN,      Config.PWMGainDown);
            SendControllerCommand(PGNValues.PGN_REAR_PWM_MIN_UP,         Config.PWMMinUp);
            SendControllerCommand(PGNValues.PGN_REAR_PWM_MIN_DOWN,       Config.PWMMinDown);
            SendControllerCommand(PGNValues.PGN_REAR_PWM_MAX_UP,         Config.PWMMaxUp);
            SendControllerCommand(PGNValues.PGN_REAR_PWM_MAX_DOWN,       Config.PWMMaxDown);
            SendControllerCommand(PGNValues.PGN_REAR_INTEGRAL_MULTPLIER, Config.IntegralMultiplier);
            SendControllerCommand(PGNValues.PGN_REAR_DEADBAND,           Config.Deadband);
        }

        /// <summary>
        /// Sets the current front blade position to zero above ground
        /// Send this after lowering the blade to the ground using GNSS height
        /// </summary>
        public void FrontBladeAtZero
            (
            )
        {
            SendControllerCommand(PGNValues.PGN_FRONT_ZERO_BLADE_HEIGHT, 0);
        }

        /// <summary>
        /// Sets the current rear blade position to zero above ground
        /// Send this after lowering the blade to the ground using GNSS height
        /// </summary>
        public void RearBladeAtZero
            (
            )
        {
            SendControllerCommand(PGNValues.PGN_REAR_ZERO_BLADE_HEIGHT, 0);
        }

        /// <summary>
        /// Communicates with the controller
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void WorkThread_DoWork(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker WorkThread = sender as BackgroundWorker;

            while (!WorkThread.CancellationPending)
            {
                // send ping to controller to tell it we are alive
                if (PingTime < DateTime.Now)
                {
                    PingTime = DateTime.Now.AddMilliseconds(PING_PERIOD_MS);

                    SendControllerCommand(PGNValues.PGN_PING, 0);
                }

                // controller has disappeared
                if ((DateTime.Now >= LastRxPingTime.AddMilliseconds(PING_TIMEOUT_PERIOD_MS)) && ControllerFound)
                {
                    ControllerFound = false;

                    OnControllerLost?.Invoke();
                }

                // message waiting from controller
                if (ControllerChannel.Available() > 0)
                {
                    ControllerStatus Stat = GetControllerStatus();
                    switch (Stat.PGN)
                    {
                        // misc
                        case PGNValues.PGN_ESTOP:
                            OnEmergencyStop?.Invoke();
                            break;

                        case PGNValues.PGN_PING:
                            LastRxPingTime = DateTime.Now;
                            break;

                        // slave offsets
                        case PGNValues.PGN_FRONT_BLADE_OFFSET_SLAVE:
                            OnFrontSlaveOffsetChanged?.Invoke((int)(Stat.Value));
                            break;

                        case PGNValues.PGN_REAR_BLADE_OFFSET_SLAVE:
                            OnRearSlaveOffsetChanged?.Invoke((int)(Stat.Value));
                            break;

                        // blade heights
                        case PGNValues.PGN_FRONT_BLADE_HEIGHT:
                            OnFrontBladeHeightChanged?.Invoke((int)(Stat.Value));
                            break;

                        case PGNValues.PGN_REAR_BLADE_HEIGHT:
                            OnRearBladeHeightChanged?.Invoke((int)(Stat.Value));
                            break;

                        // IMU
                        case PGNValues.PGN_TRACTOR_PITCH:
                            TractorIMU.Pitch = ((Int32)Stat.Value) / 100.0;
                            OnTractorIMUChanged?.Invoke(TractorIMU);
                            break;
                        case PGNValues.PGN_TRACTOR_ROLL:
                            TractorIMU.Roll = ((Int32)Stat.Value) / 100.0;
                            OnTractorIMUChanged?.Invoke(TractorIMU);
                            break;
                        case PGNValues.PGN_TRACTOR_YAW:
                            TractorIMU.Yaw = ((Int32)Stat.Value) / 100.0;
                            OnTractorIMUChanged?.Invoke(TractorIMU);
                            break;

                        case PGNValues.PGN_FRONT_PITCH:
                            FrontScraperIMU.Pitch = ((Int32)Stat.Value) / 100.0;
                            OnFrontIMUChanged?.Invoke(FrontScraperIMU);
                            break;
                        case PGNValues.PGN_FRONT_ROLL:
                            FrontScraperIMU.Roll = ((Int32)Stat.Value) / 100.0;
                            OnFrontIMUChanged?.Invoke(FrontScraperIMU);
                            break;
                        case PGNValues.PGN_FRONT_YAW:
                            FrontScraperIMU.Yaw = ((Int32)Stat.Value) / 100.0;
                            OnFrontIMUChanged?.Invoke(FrontScraperIMU);
                            break;

                        case PGNValues.PGN_REAR_PITCH:
                            RearScraperIMU.Pitch = ((Int32)Stat.Value) / 100.0;
                            OnRearIMUChanged?.Invoke(RearScraperIMU);
                            break;
                        case PGNValues.PGN_REAR_ROLL:
                            RearScraperIMU.Roll = ((Int32)Stat.Value) / 100.0;
                            OnRearIMUChanged?.Invoke(RearScraperIMU);
                            break;
                        case PGNValues.PGN_REAR_YAW:
                            RearScraperIMU.Yaw = ((Int32)Stat.Value) / 100.0;
                            OnRearIMUChanged?.Invoke(RearScraperIMU);
                            break;

                        // blade auto flags
                        case PGNValues.PGN_FRONT_BLADE_AUTO:
                            bool Auto = Stat.Value == 1 ? true : false;
                            OnFrontBladeAutoChanged?.Invoke(Auto);
                            break;

                        case PGNValues.PGN_REAR_BLADE_AUTO:
                            Auto = Stat.Value == 1 ? true : false;
                            OnRearBladeAutoChanged?.Invoke(Auto);
                            break;

                        // blade direction
                        case PGNValues.PGN_FRONT_BLADE_DIRECTION:
                            bool Up = Stat.Value == 1 ? true : false;
                            OnFrontBladeDirectionChanged?.Invoke(Up);
                            break;

                        case PGNValues.PGN_REAR_BLADE_DIRECTION:
                            Up = Stat.Value == 1 ? true : false;
                            OnRearBladeDirectionChanged?.Invoke(Up);
                            break;
                    }
                }

                Thread.Sleep(1);
            }
        }

        /// <summary>
        /// Sets the front cut valve
        /// </summary>
        /// <param name="Value">CUTVALVE_MIN -> CUTVALVE_MAX with 100 = at target height</param>
        public void SetFrontCutValve
            (
            uint Value
            )
        {
            if (Value > CUTVALVE_MAX) Value = CUTVALVE_MAX;

            ControllerCommand TxCmd;
            TxCmd.PGN = PGNValues.PGN_FRONT_CUT_VALVE;
            TxCmd.Value = Value;
            SendControllerCommand(TxCmd);
        }

        /// <summary>
        /// Sets the rear cut valve
        /// </summary>
        /// <param name="Value">CUTVALVE_MIN -> CUTVALVE_MAX with 100 = at target height</param>
        public void SetRearCutValve
            (
            uint Value
            )
        {
            if (Value > CUTVALVE_MAX) Value = CUTVALVE_MAX;

            ControllerCommand TxCmd;
            TxCmd.PGN = PGNValues.PGN_REAR_CUT_VALVE;
            TxCmd.Value = Value;
            SendControllerCommand(TxCmd);
        }

        /// <summary>
        /// Resets the controller which in turn will reset all nodes on the network
        /// WARNING: this will cause the COM port to disappear and then return
        /// </summary>
        public void ResetController
            (
            )
        {
            SendControllerCommand(PGNValues.PGN_RESET, 0);
        }

        private void SendControllerCommand
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

        private void SendControllerCommand
            (
            ControllerCommand Cmd
            )
        {
            ControllerChannel.Packet.TxBuff[0] = (byte)((UInt16)Cmd.PGN & 0xFF);
            ControllerChannel.Packet.TxBuff[1] = (byte)(((UInt16)Cmd.PGN >> 8) & 0xFF);
            ControllerChannel.Packet.TxBuff[2] = (byte)(Cmd.Value & 0xFF);
            ControllerChannel.Packet.TxBuff[3] = (byte)((Cmd.Value >> 8) & 0xFF);
            ControllerChannel.Packet.TxBuff[4] = (byte)((Cmd.Value >> 16) & 0xFF);
            ControllerChannel.Packet.TxBuff[5] = (byte)((Cmd.Value >> 24) & 0xFF);
            ControllerChannel.SendData(6);
            //Console.WriteLine(string.Format("Tx: {0}:0x{1:X8}", Cmd.PGN, Cmd.Value));
        }

        private ControllerStatus GetControllerStatus
            (
            )
        {
            ControllerStatus Stat;

            Stat.PGN = (PGNValues)(((UInt16)ControllerChannel.Packet.RxBuff[1] << 8) | ControllerChannel.Packet.RxBuff[0]);
            Stat.Value = (UInt32)(((UInt32)ControllerChannel.Packet.RxBuff[5] << 24) |
                ((UInt32)ControllerChannel.Packet.RxBuff[4] << 16) |
                ((UInt32)ControllerChannel.Packet.RxBuff[3] << 8) |
                ControllerChannel.Packet.RxBuff[2]);

            //Console.WriteLine(string.Format("Rx: {0}:0x{1:X8}", Stat.PGN, Stat.Value));

            return Stat;
        }
    }
}
