import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

// UUIDs matching the firmware's custom audio service
final Guid audioServiceUuid = Guid('12345678-1234-5678-1234-56789abcdef0');
final Guid audioDataUuid    = Guid('12345678-1234-5678-1234-56789abcdef1');
final Guid batteryUuid      = Guid('12345678-1234-5678-1234-56789abcdef2');
final Guid passwordUuid     = Guid('12345678-1234-5678-1234-56789abcdef3');

const String kPairPasskey = '123456';

void main() {
  FlutterBluePlus.setLogLevel(LogLevel.warning);
  runApp(const AudioApp());
}

class AudioApp extends StatelessWidget {
  const AudioApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Audio Recorder',
      theme: ThemeData(
        colorSchemeSeed: Colors.indigo,
        useMaterial3: true,
        brightness: Brightness.dark,
      ),
      home: const HomePage(),
    );
  }
}

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

enum AppState { idle, scanning, connecting, connected, needPassword }

class _HomePageState extends State<HomePage> {
  AppState _state = AppState.idle;
  BluetoothDevice? _device;
  BluetoothCharacteristic? _audioChar;
  BluetoothCharacteristic? _batteryChar;
  BluetoothCharacteristic? _passwordChar;

  StreamSubscription? _scanSub;
  StreamSubscription? _connSub;
  StreamSubscription? _audioSub;
  StreamSubscription? _batterySub;
  Timer? _reconnectTimer;
  Timer? _statsTimer;

  // Battery
  int _batteryMv = 0;
  DateTime? _batteryTime;

  // Audio stats
  int _packetsReceived = 0;
  int _bytesReceived = 0;
  DateTime? _streamStart;
  double _currentBps = 0;
  int _lastBytesSnapshot = 0;
  DateTime? _lastStatsTime;
  bool _isRecording = false;
  DateTime? _lastPacketTime;

  String _statusText = 'Готово';
  String _deviceName = '';
  bool _autoConnect = true;

  // Password
  bool _isAuthed = false;
  final TextEditingController _passCtrl = TextEditingController();

  @override
  void initState() {
    super.initState();
    _requestPermissions().then((_) {
      if (_autoConnect) _startScan();
    });
  }

  Future<void> _requestPermissions() async {
    await [
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.location,
    ].request();
  }

  // ---------------- Scanning & connecting ----------------

  Future<void> _startScan() async {
    if (!mounted) return;
    if (_state == AppState.scanning ||
        _state == AppState.connecting ||
        _state == AppState.connected ||
        _state == AppState.needPassword) {
      return;
    }

    setState(() {
      _state = AppState.scanning;
      _statusText = 'Поиск устройства…';
    });

    try {
      _scanSub?.cancel();
      _scanSub = FlutterBluePlus.onScanResults.listen((results) {
        for (final r in results) {
          final uuids = r.advertisementData.serviceUuids;
          final name = r.device.platformName;
          final matchesUuid = uuids.any((u) =>
              u.toString().toLowerCase() ==
              audioServiceUuid.toString().toLowerCase());
          final matchesName = name.toLowerCase().contains('audiorec');
          if ((matchesUuid || matchesName) && _state == AppState.scanning) {
            _stopScan();
            _connectToDevice(r.device);
            break;
          }
        }
      });

      await FlutterBluePlus.startScan(
        timeout: const Duration(seconds: 12),
        withServices: [audioServiceUuid],
      );

      if (_state == AppState.scanning && _autoConnect) {
        _scheduleReconnect();
      }
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _statusText = 'Ошибка поиска: $e';
        _state = AppState.idle;
      });
      if (_autoConnect) _scheduleReconnect();
    }
  }

  void _stopScan() {
    FlutterBluePlus.stopScan();
    _scanSub?.cancel();
    _scanSub = null;
  }

  Future<void> _connectToDevice(BluetoothDevice device) async {
    if (!mounted) return;
    setState(() {
      _state = AppState.connecting;
      _statusText = 'Подключение…';
      _deviceName = device.platformName.isNotEmpty
          ? device.platformName
          : device.remoteId.toString();
    });

    try {
      _device = device;
      _connSub?.cancel();
      _connSub = device.connectionState.listen((state) {
        if (state == BluetoothConnectionState.disconnected) {
          _handleDisconnect();
        }
      });

      await device.connect(timeout: const Duration(seconds: 15));
      try {
        await device.requestMtu(247);
      } catch (_) {}
      await _setupServices();
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _statusText = _autoConnect
            ? 'Сопряжение/подключение не удалось. Повтор…'
            : 'Не удалось подключиться: $e';
      });
      if (_autoConnect) {
        _handleDisconnect();
      } else {
        setState(() => _state = AppState.idle);
      }
    }
  }

  Future<void> _setupServices() async {
    final device = _device;
    if (device == null) return;

    try {
      List<BluetoothService> services = await device.discoverServices();

      for (final svc in services) {
        if (svc.uuid != audioServiceUuid) continue;
        for (final chr in svc.characteristics) {
          if (chr.uuid == audioDataUuid) {
            _audioChar = chr;
          } else if (chr.uuid == batteryUuid) {
            _batteryChar = chr;
          } else if (chr.uuid == passwordUuid) {
            _passwordChar = chr;
          }
        }
      }

      // Battery: subscribe + read once.
      if (_batteryChar != null) {
        await _batteryChar!.setNotifyValue(true);
        _batterySub?.cancel();
        _batterySub = _batteryChar!.onValueReceived.listen(_onBattery);
        try {
          final v = await _batteryChar!.read();
          _onBattery(v);
        } catch (_) {}
      }

      // Audio: subscribe to notifications.
      if (_audioChar != null) {
        await _audioChar!.setNotifyValue(true);
        _audioSub?.cancel();
        _audioSub = _audioChar!.onValueReceived.listen(_onAudioData);
      }

      // Password: check auth status.
      await _checkAuth();

      if (!mounted) return;

      if (!_isAuthed && _passwordChar != null) {
        // Need password — show dialog.
        setState(() {
          _state = AppState.needPassword;
          _statusText = 'Требуется пароль';
        });
        _showPasswordDialog();
      } else {
        _onAuthed();
      }
    } catch (e) {
      if (!mounted) return;
      setState(() => _statusText = 'Ошибка служб: $e');
      _handleDisconnect();
    }
  }

  Future<void> _checkAuth() async {
    if (_passwordChar == null) return;
    try {
      final v = await _passwordChar!.read();
      _isAuthed = v.isNotEmpty && v[0] == 1;
    } catch (_) {}
  }

  Future<bool> _writePassword(String pass) async {
    if (_passwordChar == null) return false;
    try {
      await _passwordChar!.write(pass.codeUnits, withoutResponse: false);
      await Future.delayed(const Duration(milliseconds: 100));
      await _checkAuth();
      return _isAuthed;
    } catch (e) {
      return false;
    }
  }

  void _showPasswordDialog() {
    _passCtrl.clear();
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (ctx) {
        bool wrong = false;
        bool busy = false;
        return StatefulBuilder(builder: (ctx, setDialog) {
          return AlertDialog(
            title: const Text('Введите пароль'),
            content: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                const Text(
                  'Введите пароль сопряжения, чтобы разблокировать запись и передачу аудио.',
                  style: TextStyle(color: Colors.grey, fontSize: 13),
                ),
                const SizedBox(height: 12),
                TextField(
                  controller: _passCtrl,
                  autofocus: true,
                  obscureText: true,
                  keyboardType: TextInputType.number,
                  decoration: InputDecoration(
                    hintText: 'Пароль (123456)',
                    errorText: wrong ? 'Неверный пароль' : null,
                    border: const OutlineInputBorder(),
                  ),
                  onSubmitted: (_) async {
                    if (busy) return;
                    setDialog(() { busy = true; wrong = false; });
                    final ok = await _writePassword(_passCtrl.text);
                    if (ok) {
                      if (ctx.mounted) Navigator.pop(ctx);
                      _onAuthed();
                    } else {
                      setDialog(() { busy = false; wrong = true; });
                    }
                  },
                ),
              ],
            ),
            actions: [
              TextButton(
                onPressed: () {
                  Navigator.pop(ctx);
                  _disconnect();
                },
                child: const Text('Отмена'),
              ),
              FilledButton(
                onPressed: busy
                    ? null
                    : () async {
                        if (busy) return;
                        setDialog(() { busy = true; wrong = false; });
                        final ok = await _writePassword(_passCtrl.text);
                        if (ok) {
                          if (ctx.mounted) Navigator.pop(ctx);
                          _onAuthed();
                        } else {
                          setDialog(() { busy = false; wrong = true; });
                        }
                      },
                child: busy
                    ? const SizedBox(width: 18, height: 18,
                        child: CircularProgressIndicator(strokeWidth: 2))
                    : const Text('OK'),
              ),
            ],
          );
        });
      },
    );
  }

  void _onAuthed() {
    if (!mounted) return;
    setState(() {
      _state = AppState.connected;
      _statusText = 'Подключено';
      _isAuthed = true;
      _packetsReceived = 0;
      _bytesReceived = 0;
      _streamStart = null;
      _isRecording = false;
    });
    _startStatsTimer();
  }

  void _handleDisconnect() {
    _statsTimer?.cancel();
    _audioSub?.cancel();
    _batterySub?.cancel();
    _audioSub = null;
    _batterySub = null;
    _audioChar = null;
    _batteryChar = null;
    _passwordChar = null;
    _isAuthed = false;

    if (!mounted) return;
    setState(() {
      _state = AppState.idle;
      _statusText =
          _autoConnect ? 'Отключено. Повторное подключение…' : 'Отключено';
      _isRecording = false;
      _streamStart = null;
    });

    if (_autoConnect) _scheduleReconnect();
  }

  void _scheduleReconnect() {
    _reconnectTimer?.cancel();
    _reconnectTimer = Timer(const Duration(seconds: 2), () {
      if (mounted && _state == AppState.idle && _autoConnect) {
        _startScan();
      }
    });
  }

  // ---------------- Data handlers ----------------

  void _onBattery(List<int> value) {
    if (value.length >= 2) {
      final mv = value[0] | (value[1] << 8);
      if (mounted) {
        setState(() {
          _batteryMv = mv;
          _batteryTime = DateTime.now();
        });
      }
    }
  }

  void _onAudioData(List<int> value) {
    if (!mounted) return;
    setState(() {
      _packetsReceived++;
      _bytesReceived += value.length;
      _lastPacketTime = DateTime.now();
      if (!_isRecording) {
        _isRecording = true;
        _streamStart ??= DateTime.now();
      }
    });
  }

  void _startStatsTimer() {
    _lastStatsTime = DateTime.now();
    _lastBytesSnapshot = 0;
    _statsTimer = Timer.periodic(const Duration(seconds: 1), (_) {
      if (!mounted) return;
      final now = DateTime.now();
      if (_lastPacketTime != null &&
          now.difference(_lastPacketTime!).inMilliseconds > 1200 &&
          _isRecording) {
        setState(() => _isRecording = false);
      }
      final elapsed = now.difference(_lastStatsTime!).inMilliseconds / 1000.0;
      if (elapsed > 0) {
        setState(() {
          _currentBps = (_bytesReceived - _lastBytesSnapshot) / elapsed;
          _lastBytesSnapshot = _bytesReceived;
          _lastStatsTime = now;
        });
      }
    });
  }

  Future<void> _disconnect() async {
    _autoConnect = false;
    _reconnectTimer?.cancel();
    _stopScan();
    if (_device != null) {
      try {
        await _device!.disconnect();
      } catch (_) {}
    }
    _handleDisconnect();
  }

  Future<void> _reconnect() async {
    _autoConnect = true;
    await _startScan();
  }

  // ---------------- Battery helpers ----------------

  int _batteryPercent(int mv) {
    if (mv <= 0) return 0;
    const points = <int>[
      3300, 0, 3400, 3, 3500, 8, 3600, 18, 3700, 32,
      3800, 50, 3900, 68, 4000, 82, 4100, 95, 4200, 100,
    ];
    if (mv <= points[0]) return 0;
    if (mv >= points[points.length - 2]) return 100;
    for (int i = 0; i < points.length - 2; i += 2) {
      final mv0 = points[i], p0 = points[i + 1];
      final mv1 = points[i + 2], p1 = points[i + 3];
      if (mv >= mv0 && mv <= mv1) {
        final f = (mv - mv0) / (mv1 - mv0);
        return (p0 + (p1 - p0) * f).round();
      }
    }
    return 0;
  }

  Color _batteryColor(int pct) {
    if (pct >= 50) return Colors.green;
    if (pct >= 20) return Colors.orange;
    return Colors.red;
  }

  // ---------------- Formatting ----------------

  String _formatBytes(int bytes) {
    if (bytes < 1024) return '$bytes Б';
    if (bytes < 1024 * 1024) return '${(bytes / 1024).toStringAsFixed(1)} КБ';
    return '${(bytes / (1024 * 1024)).toStringAsFixed(1)} МБ';
  }

  String _formatBps(double bps) {
    if (bps < 1024) return '${bps.toStringAsFixed(0)} Б/с';
    if (bps < 1024 * 1024) return '${(bps / 1024).toStringAsFixed(1)} КБ/с';
    return '${(bps / (1024 * 1024)).toStringAsFixed(1)} МБ/с';
  }

  String _formatVoltage(int mv) {
    final v = (mv / 1000).toStringAsFixed(2);
    return '$v В';
  }

  Duration get _elapsed {
    if (_streamStart == null) return Duration.zero;
    return DateTime.now().difference(_streamStart!);
  }

  String _formatDuration(Duration d) {
    final m = d.inMinutes.remainder(60).toString().padLeft(2, '0');
    final s = d.inSeconds.remainder(60).toString().padLeft(2, '0');
    return '${d.inHours}:$m:$s';
  }

  @override
  void dispose() {
    _autoConnect = false;
    _reconnectTimer?.cancel();
    _statsTimer?.cancel();
    _scanSub?.cancel();
    _connSub?.cancel();
    _audioSub?.cancel();
    _batterySub?.cancel();
    _passCtrl.dispose();
    _device?.disconnect();
    super.dispose();
  }

  // ---------------- UI ----------------

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Аудиорекордер'),
        actions: [
          if (_state == AppState.connected || _state == AppState.needPassword)
            IconButton(
              icon: const Icon(Icons.link_off),
              onPressed: _disconnect,
              tooltip: 'Отключиться',
            ),
          if (_state == AppState.idle || _state == AppState.scanning)
            IconButton(
              icon: Icon(
                  _autoConnect ? Icons.autorenew : Icons.bluetooth),
              onPressed: _reconnect,
              tooltip: 'Подключиться',
            ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            _statusCard(),
            const SizedBox(height: 16),
            if (_state == AppState.connected) ...[
              _batteryCard(),
              const SizedBox(height: 16),
              _audioCard(),
            ] else if (_state == AppState.needPassword) ...[
              _batteryCard(),
              const SizedBox(height: 16),
              _lockedCard(),
            ] else
              ..._hintCards(),
            const Spacer(),
          ],
        ),
      ),
    );
  }

  Widget _statusCard() {
    IconData icon;
    Color color;
    switch (_state) {
      case AppState.connected:
        icon = Icons.bluetooth_connected;
        color = Colors.green;
        break;
      case AppState.needPassword:
        icon = Icons.lock;
        color = Colors.amber;
        break;
      case AppState.connecting:
      case AppState.scanning:
        icon = Icons.bluetooth_searching;
        color = Colors.orange;
        break;
      case AppState.idle:
        icon = Icons.bluetooth_disabled;
        color = Colors.grey;
        break;
    }
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
            Icon(icon, size: 56, color: color),
            const SizedBox(height: 12),
            Text(_statusText,
                style: Theme.of(context).textTheme.titleMedium,
                textAlign: TextAlign.center),
            if (_deviceName.isNotEmpty) ...[
              const SizedBox(height: 6),
              Text(_deviceName,
                  style: TextStyle(color: Colors.grey[400], fontSize: 13)),
            ],
          ],
        ),
      ),
    );
  }

  Widget _lockedCard() {
    return Card(
      color: Colors.amber.shade900.withValues(alpha: 0.3),
      child: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
            const Icon(Icons.lock_outline, size: 40, color: Colors.amber),
            const SizedBox(height: 12),
            const Text('Устройство заблокировано',
                style: TextStyle(fontSize: 16, fontWeight: FontWeight.w600)),
            const SizedBox(height: 8),
            const Text(
              'Введите пароль, чтобы разблокировать запись и передачу аудио.\n'
              'Батарея отображается без блокировки.',
              textAlign: TextAlign.center,
              style: TextStyle(color: Colors.grey, fontSize: 13),
            ),
            const SizedBox(height: 16),
            FilledButton.icon(
              onPressed: _showPasswordDialog,
              icon: const Icon(Icons.vpn_key),
              label: const Text('Ввести пароль'),
              style: FilledButton.styleFrom(
                minimumSize: const Size(double.infinity, 48),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _batteryCard() {
    final pct = _batteryPercent(_batteryMv);
    final hasData = _batteryMv > 0;
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Row(
                  children: [
                    Icon(Icons.battery_charging_full,
                        color: hasData ? _batteryColor(pct) : Colors.grey,
                        size: 36),
                    const SizedBox(width: 12),
                    Text('Батарея',
                        style: Theme.of(context).textTheme.titleMedium),
                  ],
                ),
                Text(
                  hasData ? '$pct %' : '—',
                  style: TextStyle(
                    fontSize: 34,
                    fontWeight: FontWeight.bold,
                    color: hasData ? _batteryColor(pct) : Colors.grey,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 12),
            LinearProgressIndicator(
              value: hasData ? (pct / 100) : 0,
              minHeight: 10,
              backgroundColor: Colors.white12,
              color: hasData ? _batteryColor(pct) : Colors.grey,
            ),
            const SizedBox(height: 10),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Text(hasData ? _formatVoltage(_batteryMv) : 'нет данных',
                    style: const TextStyle(color: Colors.grey)),
                if (_batteryTime != null)
                  Text(
                      'обновлено '
                      '${_batteryTime!.hour.toString().padLeft(2, '0')}:'
                      '${_batteryTime!.minute.toString().padLeft(2, '0')}:'
                      '${_batteryTime!.second.toString().padLeft(2, '0')}',
                      style:
                          const TextStyle(color: Colors.grey, fontSize: 12)),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _audioCard() {
    final recording = _isRecording;
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(
                  recording ? Icons.fiber_manual_record : Icons.mic_none,
                  color: recording ? Colors.red : Colors.grey,
                  size: 28,
                ),
                const SizedBox(width: 10),
                Text(
                    recording
                        ? 'Идёт запись'
                        : 'Ожидание (запись выключена)',
                    style: TextStyle(
                        color: recording ? Colors.red : Colors.grey,
                        fontSize: 16,
                        fontWeight: FontWeight.w600)),
              ],
            ),
            const Divider(height: 28),
            _statRow('Длительность',
                recording ? _formatDuration(_elapsed) : '—'),
            const Divider(),
            _statRow('Пакеты', '$_packetsReceived'),
            const Divider(),
            _statRow('Принято', _formatBytes(_bytesReceived)),
            const Divider(),
            _statRow('Скорость', recording ? _formatBps(_currentBps) : '—'),
          ],
        ),
      ),
    );
  }

  List<Widget> _hintCards() {
    return [
      Card(
        child: Padding(
          padding: const EdgeInsets.all(20),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                children: const [
                  Icon(Icons.vpn_key, color: Colors.indigoAccent),
                  SizedBox(width: 10),
                  Text('Сопряжение',
                      style:
                          TextStyle(fontSize: 16, fontWeight: FontWeight.w600)),
                ],
              ),
              const SizedBox(height: 10),
              const Text(
                'При первом подключении телефон предложит ввести код.\n'
                'Введите пароль:',
                style: TextStyle(color: Colors.grey),
              ),
              const SizedBox(height: 8),
              const Text(kPairPasskey,
                  style: TextStyle(
                      fontSize: 30,
                      fontWeight: FontWeight.bold,
                      letterSpacing: 6,
                      color: Colors.indigoAccent)),
              const SizedBox(height: 10),
              const Text(
                'Чтобы запустить сопряжение заново — удерживайте кнопку на '
                'рекордере дольше 5 секунд при включении.',
                style: TextStyle(color: Colors.grey, fontSize: 12),
              ),
            ],
          ),
        ),
      ),
      const SizedBox(height: 16),
      Card(
        child: Padding(
          padding: const EdgeInsets.all(20),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                children: const [
                  Icon(Icons.touch_app, color: Colors.tealAccent),
                  SizedBox(width: 10),
                  Text('Кнопка рекордера',
                      style:
                          TextStyle(fontSize: 16, fontWeight: FontWeight.w600)),
                ],
              ),
              const SizedBox(height: 10),
              const Text(
                '• Короткое нажатие (< 2 с) — начать/остановить запись\n'
                '• Удержание (> 5 с) — режим сопряжения\n'
                '• 10 с бездействия — уход в спящий режим',
                style: TextStyle(color: Colors.grey),
              ),
            ],
          ),
        ),
      ),
    ];
  }

  Widget _statRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label, style: const TextStyle(color: Colors.grey)),
          Text(value,
              style:
                  const TextStyle(fontSize: 16, fontWeight: FontWeight.bold)),
        ],
      ),
    );
  }
}
