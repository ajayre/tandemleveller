using System;
using System.IO.Ports;
using System.Threading;

namespace SerialTransfer
{

    /// <summary>
    /// C# port of SerialTransfer class for packet-based serial communication
    /// using the standard System.IO.Ports.SerialPort class
    /// </summary>
    /// <summary>
    /// C# port of SerialTransfer class for packet-based serial communication
    /// using the standard System.IO.Ports.SerialPort class
    /// </summary>
    public class SerialTransfer
    {
        // Public members
        public Packet Packet { get; } = new Packet();
        public byte BytesRead { get; private set; } = 0;
        public int Status { get; private set; } = 0;

        // Private members
        private SerialPort _port;
        private uint _timeout = 50; // Default timeout value

        /// <summary>
        /// Advanced initializer for the SerialTransfer Class
        /// </summary>
        /// <param name="port">SerialPort to communicate over</param>
        /// <param name="configs">Configuration struct</param>
        public void Begin(SerialPort port, ConfigST configs)
        {
            _port = port;

            // Ensure port is open
            if (!_port.IsOpen)
                _port.Open();

            // Configure timeouts
            _port.ReadTimeout = (int)configs.Timeout;
            _port.WriteTimeout = (int)configs.Timeout;

            _timeout = configs.Timeout;
            Packet.Begin(configs);
        }

        /// <summary>
        /// Simple initializer for the SerialTransfer Class
        /// </summary>
        /// <param name="port">SerialPort to communicate over</param>
        /// <param name="debug">Whether to print debug messages</param>
        /// <param name="debugPort">Stream for debug output</param>
        /// <param name="timeout">Communication timeout in milliseconds</param>
        public void Begin(
            SerialPort port,
            bool debug = true,
            System.IO.Stream debugPort = null,
            uint? timeout = null)
        {
            _port = port;
            _timeout = timeout ?? 50; // Default 50ms timeout

            // Ensure port is open
            if (!_port.IsOpen)
                _port.Open();

            // Configure timeouts
            _port.ReadTimeout = (int)_timeout;
            _port.WriteTimeout = (int)_timeout;

            Packet.Begin(debug, debugPort ?? Console.OpenStandardOutput(), _timeout);
        }

        /// <summary>
        /// Send a specified number of bytes in packetized form
        /// </summary>
        /// <param name="messageLen">Number of bytes in the payload</param>
        /// <param name="packetId">The packet identifier</param>
        /// <returns>Number of payload bytes included in packet</returns>
        public byte SendData(ushort messageLen, byte packetId = 0)
        {
            try
            {
                byte numBytesIncl = Packet.ConstructPacket(messageLen, packetId);

                // Write the complete packet to the serial port
                _port.Write(Packet.Preamble, 0, Packet.Preamble.Length);
                _port.Write(Packet.TxBuff, 0, numBytesIncl);
                _port.Write(Packet.Postamble, 0, Packet.Postamble.Length);

                return numBytesIncl;
            }
            catch (TimeoutException)
            {
                Status = (int)PacketStatus.StalePacketError;
                return 0;
            }
            catch (InvalidOperationException)
            {
                // Port not opened or already closed
                Status = (int)PacketStatus.PayloadError;
                return 0;
            }
            catch (Exception)
            {
                Status = (int)PacketStatus.PayloadError;
                return 0;
            }
        }

        /// <summary>
        /// Parse incoming data and report errors/successful packet reception
        /// </summary>
        /// <returns>Number of bytes in RX buffer</returns>
        public byte Available()
        {
            if (!_port.IsOpen)
            {
                Status = (int)PacketStatus.PayloadError;
                return 0;
            }

            bool valid = false;
            byte recChar = 0xFF;

            try
            {
                // Check if data is available in the serial port
                if (_port.BytesToRead > 0)
                {
                    valid = true;
                    while (_port.BytesToRead > 0)
                    {
                        recChar = (byte)_port.ReadByte();
                        BytesRead = Packet.Parse(recChar, valid);
                        Status = Packet.Status;

                        if (Status != (int)PacketStatus.Continue)
                        {
                            if (Status < 0)
                                Reset();
                            break;
                        }
                    }
                }
                else
                {
                    BytesRead = Packet.Parse(recChar, valid);
                    Status = Packet.Status;

                    if (Status < 0)
                        Reset();
                }

                return BytesRead;
            }
            catch (TimeoutException)
            {
                // Handle timeout
                Status = (int)PacketStatus.StalePacketError;
                Reset();
                return 0;
            }
            catch (InvalidOperationException)
            {
                // Port not opened or already closed
                Status = (int)PacketStatus.PayloadError;
                return 0;
            }
            catch (Exception)
            {
                // Handle any other errors
                Status = (int)PacketStatus.PayloadError;
                Reset();
                return 0;
            }
        }

        /// <summary>
        /// Checks if any packets have been fully parsed
        /// </summary>
        /// <returns>Whether a full packet has been parsed</returns>
        public bool Tick()
        {
            return Available() > 0;
        }

        /// <summary>
        /// Copy an object into the transmit buffer
        /// </summary>
        /// <typeparam name="T">Type of object to transmit</typeparam>
        /// <param name="val">Object to transmit</param>
        /// <param name="index">Starting index in the transmit buffer</param>
        /// <param name="len">Number of bytes to transmit</param>
        /// <returns>Index after the transmitted object</returns>
        public ushort TxObj<T>(T val, ushort index = 0, ushort? len = null) where T : struct
        {
            return Packet.TxObj(val, index, len);
        }

        /// <summary>
        /// Copy bytes from the receive buffer into an object
        /// </summary>
        /// <typeparam name="T">Type of object to receive into</typeparam>
        /// <param name="val">Object to receive into</param>
        /// <param name="index">Starting index in the receive buffer</param>
        /// <param name="len">Number of bytes to receive</param>
        /// <returns>Index after the received object</returns>
        public ushort RxObj<T>(ref T val, ushort index = 0, ushort? len = null) where T : struct
        {
            return Packet.RxObj(ref val, index, len);
        }

        /// <summary>
        /// Pack an object and send it in a single call
        /// </summary>
        /// <typeparam name="T">Type of object to send</typeparam>
        /// <param name="val">Object to send</param>
        /// <param name="len">Number of bytes to send</param>
        /// <returns>Number of payload bytes included in packet</returns>
        public byte SendDatum<T>(T val, ushort? len = null) where T : struct
        {
            if (len == null)
            {
                len = (ushort)System.Runtime.InteropServices.Marshal.SizeOf(val);
            }

            //len ??= (ushort)System.Runtime.InteropServices.Marshal.SizeOf(val);
            return SendData(Packet.TxObj(val, 0, len));
        }

        /// <summary>
        /// Returns the ID of the last parsed packet
        /// </summary>
        /// <returns>ID of the last parsed packet</returns>
        public byte CurrentPacketId() => Packet.CurrentPacketId();

        /// <summary>
        /// Clear buffers and reset the parser state
        /// </summary>
        public void Reset()
        {
            try
            {
                // Clear the SerialPort buffer if port is open
                if (_port != null && _port.IsOpen)
                {
                    _port.DiscardInBuffer();
                    _port.DiscardOutBuffer();
                }
            }
            catch (Exception)
            {
                // Ignore errors while resetting
            }

            Packet.Reset();
            Status = Packet.Status;
        }

        /// <summary>
        /// Properly close and dispose the serial port connection
        /// </summary>
        public void Close()
        {
            if (_port != null)
            {
                try
                {
                    if (_port.IsOpen)
                    {
                        _port.Close();
                    }
                    _port.Dispose();
                }
                catch (Exception)
                {
                    // Ignore errors during close
                }
            }
        }

        /// <summary>
        /// Set up the SerialPort for use with SerialTransfer
        /// </summary>
        /// <param name="portName">COM port name</param>
        /// <param name="baudRate">Baud rate</param>
        /// <param name="parity">Parity setting</param>
        /// <param name="dataBits">Data bits</param>
        /// <param name="stopBits">Stop bits</param>
        /// <returns>Configured SerialPort</returns>
        public static SerialPort ConfigurePort(
            string portName,
            int baudRate = 9600,
            Parity parity = Parity.None,
            int dataBits = 8,
            StopBits stopBits = StopBits.One)
        {
            var port = new SerialPort
            {
                PortName = portName,
                BaudRate = baudRate,
                Parity = parity,
                DataBits = dataBits,
                StopBits = stopBits,
                ReadTimeout = 500,  // Default timeout, will be overridden in Begin()
                WriteTimeout = 500  // Default timeout, will be overridden in Begin()
            };

            return port;
        }
    }
}