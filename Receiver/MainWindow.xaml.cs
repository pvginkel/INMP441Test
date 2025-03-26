using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Timers;
using System.Windows;
using System.Windows.Forms;
using NAudio.CoreAudioApi;
using NAudio.Wave;
using Timer = System.Threading.Timer;

namespace Receiver
{
    public partial class MainWindow
    {
        private const string PlaybackDevice = "Luidsprekers (Realtek(R) Audio)";
        private const int ListenPort = 55581;

        private const int SampleRate = 16000;
        private const int BitRate = 16;
        private const int ChannelCount = 1;

        private WasapiOut? _waveOut;
        private BufferedWaveProvider? _bufferedWaveProvider;
        private UdpClient? _udpClient;
        private IPEndPoint? _remoteEP;
        private Timer? _timer;
        private volatile int _bytesReceived;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            var device = GetMMDevice(PlaybackDevice, DataFlow.Render);

            _bufferedWaveProvider = new BufferedWaveProvider(
                new WaveFormat(SampleRate, BitRate, ChannelCount)
            )
            {
                DiscardOnBufferOverflow = false
            };

            _waveOut = new WasapiOut(device, AudioClientShareMode.Shared, true, 10);
            _waveOut.Init(_bufferedWaveProvider);
            _waveOut.Play();

            _udpClient = new UdpClient(ListenPort);
            _remoteEP = new IPEndPoint(IPAddress.Any, ListenPort);

            _udpClient.BeginReceive(ReceiveCallback, null);

            _timer = new Timer(
                _ =>
                {
                    if (_bytesReceived > 0)
                    {
                        Dispatcher.BeginInvoke(() =>
                        {
                            _label.Content = $"{_bytesReceived:n0} bytes received";
                        });
                    }
                },
                null,
                TimeSpan.Zero,
                TimeSpan.FromSeconds(1)
            );
        }

        private void ReceiveCallback(IAsyncResult ar)
        {
            byte[] buffer = _udpClient!.EndReceive(ar, ref _remoteEP);

            Interlocked.Add(ref _bytesReceived, buffer.Length);

            _bufferedWaveProvider!.AddSamples(buffer, 0, buffer.Length);

            _udpClient!.BeginReceive(ReceiveCallback, null);
        }

        private MMDevice? GetMMDevice(string friendlyName, DataFlow dataFlow)
        {
            var enumerator = new MMDeviceEnumerator();
            var names = new List<string>();

            foreach (var wasapi in enumerator.EnumerateAudioEndPoints(dataFlow, DeviceState.Active))
            {
                names.Add(wasapi.FriendlyName);

                if (friendlyName == wasapi.FriendlyName)
                    return wasapi;
            }

            throw new InvalidOperationException(
                $"Cannot find device '{friendlyName}'; options are '{String.Join("', '", names)}'"
            );
        }
    }
}
