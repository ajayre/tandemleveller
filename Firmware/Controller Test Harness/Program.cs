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
            PGN_EStop = 0x0000
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
                PortName = "COM14",
                BaudRate = 38400,
                DataBits = 8,
                Parity = Parity.None,
                StopBits = StopBits.One
            };

            Controller = new SerialTransfer.SerialTransfer();
            Controller.Begin(port);

            uint x = 0;
            DateTime TxTime = DateTime.Now.AddMilliseconds(500);

            while (true)
            {
                if (TxTime < DateTime.Now)
                {
                    TxTime = DateTime.Now.AddMilliseconds(500);

                    ControllerCommand TxCmd;
                    TxCmd.PGN = 0;
                    TxCmd.Value = x++;
                    SendControllerCommand(TxCmd);
                }

                if (Controller.Available() > 0)
                {
                    ControllerStatus Stat = GetControllerStatus();
                }
            }
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
