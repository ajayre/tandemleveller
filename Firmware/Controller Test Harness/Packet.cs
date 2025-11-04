using System;
using System.IO;

namespace Controller
{
    internal enum PacketStatus
    {
        Continue = 3,
        NewData = 2,
        NoData = 1,
        CrcError = 0,
        PayloadError = -1,
        StopByteError = -2,
        StalePacketError = -3
    }

    internal struct ConfigST
    {
        public Stream DebugPort { get; set; }
        public bool Debug { get; set; }
        public Action[] Callbacks { get; set; }
        public byte CallbacksLength { get; set; }
        public uint Timeout { get; set; }

        public ConfigST(
            Stream debugPort = null,
            bool debug = true,
            Action[] callbacks = null,
            byte callbacksLength = 0,
            uint timeout = uint.MaxValue)
        {
            DebugPort = debugPort ?? Console.OpenStandardOutput();
            Debug = debug;
            Callbacks = callbacks ?? Array.Empty<Action>();
            CallbacksLength = callbacksLength;
            Timeout = timeout;
        }
    }

    internal class Packet
    {
        // Constants
        private const byte StartByte = 0x7E;
        private const byte StopByte = 0x81;
        private const byte PreambleSize = 4;
        private const byte PostambleSize = 2;
        private const byte MaxPacketSize = 0xFE;
        private const byte DefaultTimeout = 50;

        // Added dependency on PacketCRC from previous translation
        private PacketCRC _crc = new PacketCRC();

        // Public buffers and state
        public byte[] TxBuff { get; } = new byte[MaxPacketSize];
        public byte[] RxBuff { get; } = new byte[MaxPacketSize];
        public byte[] Preamble { get; } = new byte[PreambleSize] { StartByte, 0, 0, 0 };
        public byte[] Postamble { get; } = new byte[PostambleSize] { 0, StopByte };

        public byte BytesRead { get; private set; }
        public int Status { get; private set; }

        // Private state machine and tracking
        private enum PacketState
        {
            FindStartByte,
            FindIdByte,
            FindOverheadByte,
            FindPayloadLen,
            FindPayload,
            FindCrc,
            FindEndByte
        }

        private PacketState _state = PacketState.FindStartByte;

        // Private members
        private Action[] _callbacks;
        private byte _callbacksLen;
        private Stream _debugPort;
        private bool _debug;

        private byte _bytesToRec;
        private byte _payIndex;
        private byte _idByte;
        private byte _overheadByte;
        private byte _recOverheadByte;

        private uint _packetStart;
        private uint _timeout;

        // Constructors
        public void Begin(ConfigST configs)
        {
            _debugPort = configs.DebugPort;
            _debug = configs.Debug;
            _callbacks = configs.Callbacks;
            _callbacksLen = configs.CallbacksLength;
            _timeout = configs.Timeout;
        }

        public void Begin(bool debug = true, Stream debugPort = null, uint timeout = DefaultTimeout)
        {
            Begin(new ConfigST(
                debugPort: debugPort ?? Console.OpenStandardOutput(),
                debug: debug,
                timeout: timeout
            ));
        }

        // Template methods translation using generics
        public ushort TxObj<T>(T val, ushort index = 0, ushort? len = null) where T : struct
        {
            if (len == null)
            {
                len = (ushort)System.Runtime.InteropServices.Marshal.SizeOf(val);
            }

            //len ??= (ushort)System.Runtime.InteropServices.Marshal.SizeOf(val);
            var ptr = System.Runtime.InteropServices.Marshal.AllocHGlobal(len.Value);

            try
            {
                System.Runtime.InteropServices.Marshal.StructureToPtr(val, ptr, false);

                ushort maxIndex = (ushort)(len.Value + index > MaxPacketSize ? MaxPacketSize : len.Value + index);

                for (ushort i = index; i < maxIndex; i++)
                {
                    TxBuff[i] = System.Runtime.InteropServices.Marshal.ReadByte(ptr, i - index);
                }

                return maxIndex;
            }
            finally
            {
                System.Runtime.InteropServices.Marshal.FreeHGlobal(ptr);
            }
        }

        public ushort RxObj<T>(ref T val, ushort index = 0, ushort? len = null) where T : struct
        {
            if (len == null)
            {
                len = (ushort)System.Runtime.InteropServices.Marshal.SizeOf(val);
            }

            //len ??= (ushort)System.Runtime.InteropServices.Marshal.SizeOf(val);
            var ptr = System.Runtime.InteropServices.Marshal.AllocHGlobal(len.Value);

            try
            {
                ushort maxIndex = (ushort)(len.Value + index > MaxPacketSize ? MaxPacketSize : len.Value + index);

                for (ushort i = index; i < maxIndex; i++)
                {
                    System.Runtime.InteropServices.Marshal.WriteByte(ptr, i - index, RxBuff[i]);
                }

                val = (T)System.Runtime.InteropServices.Marshal.PtrToStructure(ptr, typeof(T));
                return maxIndex;
            }
            finally
            {
                System.Runtime.InteropServices.Marshal.FreeHGlobal(ptr);
            }
        }

        // Implementation of ConstructPacket method
        public byte ConstructPacket(ushort messageLen, byte packetId = 0)
        {
            byte processedLen;

            if (messageLen > MaxPacketSize)
            {
                CalcOverhead(TxBuff, MaxPacketSize);
                StuffPacket(TxBuff, MaxPacketSize);
                byte crcVal = _crc.Calculate(TxBuff, MaxPacketSize);

                Preamble[1] = packetId;
                Preamble[2] = _overheadByte;
                Preamble[3] = MaxPacketSize;

                Postamble[0] = crcVal;

                processedLen = MaxPacketSize;
            }
            else
            {
                CalcOverhead(TxBuff, (byte)messageLen);
                StuffPacket(TxBuff, (byte)messageLen);
                byte crcVal = _crc.Calculate(TxBuff, (byte)messageLen);

                Preamble[1] = packetId;
                Preamble[2] = _overheadByte;
                Preamble[3] = (byte)messageLen;

                Postamble[0] = crcVal;

                processedLen = (byte)messageLen;
            }

            return processedLen;
        }

        // Implementation of Parse method (simplified due to lack of direct Arduino equivalents)
        public byte Parse(byte recChar, bool valid = true)
        {
            bool packetFresh = _packetStart == 0 || Environment.TickCount - _packetStart < _timeout;

            if (!packetFresh)
            {
                if (_debug)
                    Console.WriteLine("ERROR: STALE PACKET");

                BytesRead = 0;
                _state = PacketState.FindStartByte;
                Status = (int)PacketStatus.StalePacketError;
                _packetStart = 0;

                return BytesRead;
            }

            if (valid)
            {
                switch (_state)
                {
                    case PacketState.FindStartByte:
                        if (recChar == StartByte)
                        {
                            _state = PacketState.FindIdByte;
                            _packetStart = (uint)Environment.TickCount;
                        }
                        break;

                    case PacketState.FindIdByte:
                        _idByte = recChar;
                        _state = PacketState.FindOverheadByte;
                        break;

                    case PacketState.FindOverheadByte:
                        _recOverheadByte = recChar;
                        _state = PacketState.FindPayloadLen;
                        break;

                    case PacketState.FindPayloadLen:
                        if (recChar > 0 && recChar <= MaxPacketSize)
                        {
                            _bytesToRec = recChar;
                            _payIndex = 0;
                            _state = PacketState.FindPayload;
                        }
                        else
                        {
                            BytesRead = 0;
                            _state = PacketState.FindStartByte;
                            Status = (int)PacketStatus.PayloadError;

                            if (_debug)
                                Console.WriteLine("ERROR: PAYLOAD_ERROR");

                            Reset();
                            return BytesRead;
                        }
                        break;

                    case PacketState.FindPayload:
                        if (_payIndex < _bytesToRec)
                        {
                            RxBuff[_payIndex] = recChar;
                            _payIndex++;

                            if (_payIndex == _bytesToRec)
                                _state = PacketState.FindCrc;
                        }
                        break;

                    case PacketState.FindCrc:
                        byte calcCrc = _crc.Calculate(RxBuff, _bytesToRec);

                        if (calcCrc == recChar)
                            _state = PacketState.FindEndByte;
                        else
                        {
                            BytesRead = 0;
                            _state = PacketState.FindStartByte;
                            Status = (int)PacketStatus.CrcError;

                            if (_debug)
                                Console.WriteLine("ERROR: CRC_ERROR");

                            Reset();
                            return BytesRead;
                        }
                        break;

                    case PacketState.FindEndByte:
                        _state = PacketState.FindStartByte;

                        if (recChar == StopByte)
                        {
                            UnpackPacket(RxBuff);
                            BytesRead = _bytesToRec;
                            Status = (int)PacketStatus.NewData;

                            if (_callbacks != null)
                            {
                                if (_idByte < _callbacksLen)
                                    _callbacks[_idByte]?.Invoke();
                                //else if (_debug)
                                    // Console.WriteLine($"ERROR: No callback available for packet ID {_idByte}");
                            }
                            _packetStart = 0;
                            return _bytesToRec;
                        }

                        BytesRead = 0;
                        Status = (int)PacketStatus.StopByteError;

                        if (_debug)
                            Console.WriteLine("ERROR: STOP_BYTE_ERROR");

                        Reset();
                        return BytesRead;

                    default:
                        if (_debug)
                            Console.WriteLine($"ERROR: Undefined state {_state}");

                        Reset();
                        BytesRead = 0;
                        _state = PacketState.FindStartByte;
                        break;
                }
            }
            else
            {
                BytesRead = 0;
                Status = (int)PacketStatus.NoData;
                return BytesRead;
            }

            BytesRead = 0;
            Status = (int)PacketStatus.Continue;
            return BytesRead;
        }

        // Implementation of CurrentPacketID method
        public byte CurrentPacketId() => _idByte;

        // Implementation of CalcOverhead method
        private void CalcOverhead(byte[] arr, byte len)
        {
            _overheadByte = 0xFF;

            for (byte i = 0; i < len; i++)
            {
                if (arr[i] == StartByte)
                {
                    _overheadByte = i;
                    break;
                }
            }
        }

        // Implementation of FindLast method
        private short FindLast(byte[] arr, byte len)
        {
            for (byte i = (byte)(len - 1); i != 0xFF; i--)
                if (arr[i] == StartByte)
                    return i;

            return -1;
        }

        // Implementation of StuffPacket method
        private void StuffPacket(byte[] arr, byte len)
        {
            short refByte = FindLast(arr, len);

            if (refByte != -1)
            {
                for (byte i = (byte)(len - 1); i != 0xFF; i--)
                {
                    if (arr[i] == StartByte)
                    {
                        arr[i] = (byte)(refByte - i);
                        refByte = i;
                    }
                }
            }
        }

        // Implementation of UnpackPacket method
        private void UnpackPacket(byte[] arr)
        {
            byte testIndex = _recOverheadByte;
            byte delta = 0;

            if (testIndex <= MaxPacketSize)
            {
                while (arr[testIndex] != 0)
                {
                    delta = arr[testIndex];
                    arr[testIndex] = StartByte;
                    testIndex += delta;
                }
                arr[testIndex] = StartByte;
            }
        }

        // Implementation of Reset method
        public void Reset()
        {
            Array.Clear(TxBuff, 0, TxBuff.Length);
            Array.Clear(RxBuff, 0, RxBuff.Length);

            BytesRead = 0;
            _packetStart = 0;
        }
    }
}
