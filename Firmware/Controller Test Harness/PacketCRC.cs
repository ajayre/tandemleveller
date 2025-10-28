using System;

namespace SerialTransfer
{

    public class PacketCRC
    {
        public byte Poly { get; private set; }
        private ushort TableLen { get; set; }
        private byte CrcLen { get; set; }
        private byte[] CsTable { get; set; }

        public PacketCRC(byte polynomial = 0x9B, byte crcLen = 8)
        {
            Poly = polynomial;
            CrcLen = crcLen;
            TableLen = (ushort)Math.Pow(2, crcLen);
            CsTable = new byte[TableLen];
            GenerateTable();
        }

        public void GenerateTable()
        {
            for (ushort i = 0; i < TableLen; ++i)
            {
                int curr = i;
                for (int j = 0; j < 8; ++j)
                {
                    if ((curr & 0x80) != 0)
                        curr = curr << 1 ^ Poly;
                    else
                        curr <<= 1;
                }
                CsTable[i] = (byte)curr;
            }
        }

        public void PrintTable()
        {
            for (ushort i = 0; i < TableLen; i++)
            {
                Console.Write($"{CsTable[i]:X2} ");
                if ((i + 1) % 16 == 0)
                    Console.WriteLine();
            }
        }

        public byte Calculate(byte val)
        {
            return val < TableLen ? CsTable[val] : (byte)0;
        }

        public byte Calculate(byte[] arr)
        {
            return Calculate(arr, (byte)arr.Length);
        }

        public byte Calculate(byte[] arr, byte len)
        {
            byte crc = 0;
            for (int i = 0; i < len; i++)
                crc = CsTable[crc ^ arr[i]];
            return crc;
        }
    }

}