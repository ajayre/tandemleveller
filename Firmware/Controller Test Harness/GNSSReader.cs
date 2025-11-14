using System;
using System.IO.Ports;
using System.Threading;

namespace Controller
{
    /// <summary>
    /// Reads GNSS data from a COM port and parses $GNGGA NMEA sentences.
    /// </summary>
    public class GNSSReader
    {
        private const int GNSS_IMMINENT_WINDOW_MS = 70;

        // speed of communication with rover
        private const int BAUDRATE = 115200;

        private SerialPort _serialPort;
        private Thread _workerThread;
        private volatile bool _isRunning;
        private string _comPortName;
        
        private long _gnssFixPeriod; // Measured time between fix strings in milliseconds
        private DateTime _lastFixTime;
        private bool _fixPeriodObtained;
        private object _lockObject = new object();

        /// <summary>
        /// Event raised when a GNSS fix is successfully parsed from a $GNGGA sentence.
        /// </summary>
        public event Action<GNSSFix> OnFixReceived;

        /// <summary>
        /// Event raised when a GNSS vector is successfully parsed from a $GPVTG or $GNVTG sentence.
        /// </summary>
        public event Action<GNSSVector> OnVectorReceived;

        /// <summary>
        /// Event raised when (GNSSFixPeriod - GNSS_IMMINENT_WINDOW_MS) milliseconds have passed since the last $GNGGA line.
        /// Only raised after GNSSFixPeriod has been determined.
        /// </summary>
        public event Action OnGNSSFixImminent;

        /// <summary>
        /// Gets the measured time between fix strings in milliseconds.
        /// Returns 0 if the period has not yet been determined.
        /// </summary>
        public long GNSSFixPeriod
        {
            get
            {
                lock (_lockObject)
                {
                    return _gnssFixPeriod;
                }
            }
            private set
            {
                lock (_lockObject)
                {
                    _gnssFixPeriod = value;
                }
            }
        }

        /// <summary>
        /// Initializes a new instance of the GNSSReader class.
        /// </summary>
        public GNSSReader()
        {
            _comPortName = null;
            _gnssFixPeriod = 0;
            _fixPeriodObtained = false;
            _isRunning = false;
        }

        /// <summary>
        /// Configures the COM port name and baud rate for the connection.
        /// </summary>
        /// <param name="comPortName">The name of the COM port (e.g., "COM1", "COM3")</param>
        public void Connect(string comPortName)
        {
            if (string.IsNullOrWhiteSpace(comPortName))
            {
                throw new ArgumentException("COM port name cannot be null or empty", nameof(comPortName));
            }

            if (_isRunning)
            {
                throw new InvalidOperationException("Cannot change connection settings while GNSSReader is running");
            }

            _comPortName = comPortName;
        }

        /// <summary>
        /// Opens the COM port and starts reading lines in a worker thread.
        /// </summary>
        public void Start()
        {
            if (_isRunning)
            {
                throw new InvalidOperationException("GNSSReader is already running");
            }

            if (string.IsNullOrWhiteSpace(_comPortName))
            {
                throw new InvalidOperationException("Must call Connect() before calling Start()");
            }

            try
            {
                // Create and configure SerialPort
                _serialPort = new SerialPort
                {
                    PortName = _comPortName,
                    BaudRate = BAUDRATE,
                    DataBits = 8,
                    Parity = Parity.None,
                    StopBits = StopBits.One,
                    Handshake = Handshake.None,
                    ReadTimeout = 100, // Reduced timeout to allow more frequent imminent event checks
                    NewLine = "\r\n" // NMEA sentences typically end with \r\n
                };

                _serialPort.Open();

                _isRunning = true;
                _fixPeriodObtained = false;
                _gnssFixPeriod = 0;

                // Start worker thread
                _workerThread = new Thread(WorkerThreadProc)
                {
                    IsBackground = true,
                    Name = "GNSSReader_WorkerThread"
                };
                _workerThread.Start();
            }
            catch (Exception ex)
            {
                _isRunning = false;
                if (_serialPort != null && _serialPort.IsOpen)
                {
                    _serialPort.Close();
                }
                throw new InvalidOperationException($"Failed to start GNSSReader on {_comPortName}", ex);
            }
        }

        /// <summary>
        /// Stops the worker thread and closes the COM port.
        /// </summary>
        public void Stop()
        {
            if (!_isRunning)
            {
                return;
            }

            _isRunning = false;

            // Close the serial port (this will cause ReadLine to throw, which is handled)
            if (_serialPort != null && _serialPort.IsOpen)
            {
                try
                {
                    _serialPort.Close();
                }
                catch (Exception)
                {
                    // Ignore errors during close
                }
            }

            // Wait for worker thread to finish (with timeout)
            if (_workerThread != null && _workerThread.IsAlive)
            {
                if (!_workerThread.Join(2000))
                {
                    // Thread didn't finish in time, but we'll continue anyway
                }
            }

            _serialPort?.Dispose();
            _serialPort = null;
        }

        /// <summary>
        /// Worker thread procedure that reads lines from the serial port.
        /// </summary>
        private void WorkerThreadProc()
        {
            DateTime? lastGNGGATime = null;
            DateTime lastImminentCheck = DateTime.MinValue;
            bool imminentEventRaised = false;
            long imminentCheckInterval = 10; // Check every 10ms for imminent fix

            try
            {
                while (_isRunning && _serialPort != null && _serialPort.IsOpen)
                {
                    try
                    {
                        string line = _serialPort.ReadLine();
                        
                        if (string.IsNullOrWhiteSpace(line))
                        {
                            // Still check for imminent event even if line is empty
                            CheckForImminentEvent(ref lastGNGGATime, ref lastImminentCheck, ref imminentEventRaised, imminentCheckInterval);
                            continue;
                        }

                        // Check if this is a $GNGGA line
                        if (line.StartsWith("$GNGGA"))
                        {
                            DateTime currentTime = DateTime.Now;

                            // Update fix period if we have a previous fix time
                            if (lastGNGGATime.HasValue)
                            {
                                long timeSinceLastFix = (long)(currentTime - lastGNGGATime.Value).TotalMilliseconds;
                                
                                if (!_fixPeriodObtained)
                                {
                                    // First time we've measured the period
                                    GNSSFixPeriod = timeSinceLastFix;
                                    _fixPeriodObtained = true;
                                }
                                else
                                {
                                    // Update period (could use average or just use latest)
                                    GNSSFixPeriod = timeSinceLastFix;
                                }
                            }

                            lastGNGGATime = currentTime;
                            _lastFixTime = currentTime;
                            lastImminentCheck = currentTime; // Reset imminent check timer
                            imminentEventRaised = false; // Reset flag for new period

                            // Try to parse the NMEA sentence
                            try
                            {
                                GNSSFix fix = GNSSFix.ParseNMEA(line);
                                
                                // Raise OnFixReceived event
                                OnFixReceived?.Invoke(fix);
                            }
                            catch (NMEAParseException)
                            {
                                // Failed to parse, but we still update timing
                                // Don't raise event for invalid sentences
                            }
                        }
                        // Check if this is a VTG line ($GPVTG or $GNVTG)
                        else if (line.StartsWith("$GPVTG") || line.StartsWith("$GNVTG"))
                        {
                            // Try to parse the VTG NMEA sentence
                            try
                            {
                                GNSSVector vector = GNSSVector.ParseNMEA(line);
                                
                                // Raise OnVectorReceived event
                                OnVectorReceived?.Invoke(vector);
                            }
                            catch (NMEAParseException)
                            {
                                // Failed to parse - don't raise event for invalid sentences
                            }
                        }

                        // Check if we should raise OnGNSSFixImminent event
                        CheckForImminentEvent(ref lastGNGGATime, ref lastImminentCheck, ref imminentEventRaised, imminentCheckInterval);
                    }
                    catch (TimeoutException)
                    {
                        // ReadLine timeout - check for imminent event and continue loop
                        CheckForImminentEvent(ref lastGNGGATime, ref lastImminentCheck, ref imminentEventRaised, imminentCheckInterval);
                        continue;
                    }
                    catch (InvalidOperationException)
                    {
                        // Port was closed - exit loop
                        break;
                    }
                    catch (Exception)
                    {
                        // Other exceptions - continue if still running
                        if (!_isRunning)
                        {
                            break;
                        }
                    }
                }
            }
            finally
            {
                // Ensure we mark as not running
                _isRunning = false;
            }
        }

        /// <summary>
        /// Checks if the OnGNSSFixImminent event should be raised.
        /// </summary>
        private void CheckForImminentEvent(ref DateTime? lastGNGGATime, ref DateTime lastImminentCheck, ref bool imminentEventRaised, long checkInterval)
        {
            if (!_fixPeriodObtained || GNSSFixPeriod <= 0 || !lastGNGGATime.HasValue || imminentEventRaised)
            {
                return;
            }

            DateTime now = DateTime.Now;
            
            // Only check periodically to avoid excessive CPU usage
            if ((now - lastImminentCheck).TotalMilliseconds >= checkInterval)
            {
                lastImminentCheck = now;
                
                long timeSinceLastFix = (long)(now - lastGNGGATime.Value).TotalMilliseconds;
                long imminentThreshold = GNSSFixPeriod - GNSS_IMMINENT_WINDOW_MS;

                if (timeSinceLastFix >= imminentThreshold)
                {
                    // Raise the event
                    OnGNSSFixImminent?.Invoke();
                    imminentEventRaised = true; // Mark as raised to avoid multiple raises
                }
            }
        }

        /// <summary>
        /// Gets a value indicating whether the GNSSReader is currently running.
        /// </summary>
        public bool IsRunning
        {
            get { return _isRunning; }
        }
    }
}

