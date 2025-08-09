using System;
using System.Buffers;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;

// NOTE: This consolidated source provides compile-ready scaffolding that maps to the requested feature set.
// Many components are lightweight stubs designed to be extended on target hardware.

// ============================================================================
// HAL (Hardware Abstraction Layer)
// ============================================================================
namespace PrecisionControl.Hal
{
    public enum HalBackend { Real, Virtual }
    public enum PinMode { Input, Output, Alt }

    public interface IGpio
    {
        void SetPinMode(int pin, PinMode mode);
        void Write(int pin, bool value);
        bool Read(int pin);
    }

    public interface IPwm
    {
        void Configure(int channel, int frequencyHz, double duty01);
        void Stop(int channel);
    }

    public interface ICan
    {
        // Non-blocking send/receive stubs
        bool TrySend(ReadOnlySpan<byte> frame, int id, CancellationToken ct);
        bool TryReceive(Span<byte> buffer, out int id, out int len);
    }

    public interface IUsbDevice
    {
        void Write(ReadOnlySpan<byte> data);
        int Read(Span<byte> buffer);
    }

    public interface IMmio : IDisposable
    {
        Span<byte> Map(ulong physAddr, int length);
        void Unmap();
    }

    public interface IClock
    {
        long Timestamp();
        long TicksPerSecond { get; }
        double SecondsFromTicks(long ticks);
        long TicksFromSeconds(double seconds);
    }

    public interface IHal : IDisposable
    {
        IGpio Gpio { get; }
        IPwm Pwm { get; }
        ICan Can { get; }
        IUsbDevice Usb { get; }
        IMmio Mmio { get; }
        IClock Clock { get; }
        bool UnmanagedPinTogglingEnabled { get; }
    }

    public static class HalFactory
    {
        public static IHal Create(HalBackend backend, PrecisionControl.ConsoleLogger logger)
            => backend == HalBackend.Real ? new RealHal(logger) : new VirtualHal(logger);
    }

    // High-resolution clock using Stopwatch
    internal sealed class StopwatchClock : IClock
    {
        private static readonly double s_ticksToSeconds = 1.0 / Stopwatch.Frequency;
        public long Timestamp() => Stopwatch.GetTimestamp();
        public long TicksPerSecond => Stopwatch.Frequency;
        public double SecondsFromTicks(long ticks) => ticks * s_ticksToSeconds;
        public long TicksFromSeconds(double seconds) => (long)(seconds * TicksPerSecond);
    }

    // -----------------------------
    // Virtualized HAL (deterministic)
    // -----------------------------
    internal sealed class VirtualHal : IHal
    {
        private readonly PrecisionControl.ConsoleLogger _log;
        public IGpio Gpio { get; }
        public IPwm Pwm { get; }
        public ICan Can { get; }
        public IUsbDevice Usb { get; }
        public IMmio Mmio { get; }
        public IClock Clock { get; } = new StopwatchClock();
        public bool UnmanagedPinTogglingEnabled => false;

        public VirtualHal(PrecisionControl.ConsoleLogger log)
        {
            _log = log;
            Gpio = new VirtualGpio(_log);
            Pwm = new VirtualPwm(_log);
            Can = new VirtualCan(_log);
            Usb = new VirtualUsb(_log);
            Mmio = new VirtualMmio(_log);
            _log.Info("Virtual HAL initialized.");
        }

        public void Dispose()
        {
            Mmio.Dispose();
        }

        private sealed class VirtualGpio : IGpio
        {
            private readonly PrecisionControl.ConsoleLogger _log;
            private readonly object _gate = new();
            private readonly Dictionary<int, (PinMode mode, bool val)> _pins = new();

            public VirtualGpio(PrecisionControl.ConsoleLogger log) => _log = log;

            public void SetPinMode(int pin, PinMode mode)
            {
                lock (_gate)
                    _pins[pin] = (mode, _pins.TryGetValue(pin, out var t) ? t.val : false);
            }

            public void Write(int pin, bool value)
            {
                lock (_gate)
                {
                    if (!_pins.TryGetValue(pin, out var t) || t.mode != PinMode.Output)
                        _pins[pin] = (PinMode.Output, value);
                    else
                        _pins[pin] = (t.mode, value);
                }
            }

            public bool Read(int pin)
            {
                lock (_gate)
                    return _pins.TryGetValue(pin, out var t) ? t.val : false;
            }
        }

        private sealed class VirtualPwm : IPwm
        {
            private readonly PrecisionControl.ConsoleLogger _log;
            private readonly object _gate = new();
            private readonly Dictionary<int, (int freq, double duty)> _channels = new();
            public VirtualPwm(PrecisionControl.ConsoleLogger log) => _log = log;

            public void Configure(int channel, int frequencyHz, double duty01)
            {
                duty01 = Math.Clamp(duty01, 0, 1);
                lock (_gate) _channels[channel] = (frequencyHz, duty01);
            }

            public void Stop(int channel)
            {
                lock (_gate) _channels.Remove(channel);
            }
        }

        private sealed class VirtualCan : ICan
        {
            private readonly PrecisionControl.ConsoleLogger _log;
            private readonly object _gate = new();
            private readonly Queue<(int id, byte[] data)> _queue = new();
            public VirtualCan(PrecisionControl.ConsoleLogger log) => _log = log;

            public bool TrySend(ReadOnlySpan<byte> frame, int id, CancellationToken ct)
            {
                var arr = ArrayPool<byte>.Shared.Rent(frame.Length);
                frame.CopyTo(arr);
                lock (_gate) _queue.Enqueue((id, arr[..frame.Length]));
                return true;
            }

            public bool TryReceive(Span<byte> buffer, out int id, out int len)
            {
                lock (_gate)
                {
                    if (_queue.Count > 0)
                    {
                        var (fid, data) = _queue.Dequeue();
                        id = fid;
                        len = Math.Min(buffer.Length, data.Length);
                        data.AsSpan(0, len).CopyTo(buffer);
                        ArrayPool<byte>.Shared.Return(data);
                        return true;
                    }
                }
                id = 0; len = 0; return false;
            }
        }

        private sealed class VirtualUsb : IUsbDevice
        {
            private readonly PrecisionControl.ConsoleLogger _log;
            private readonly object _gate = new();
            private readonly Queue<byte> _rx = new();
            public VirtualUsb(PrecisionControl.ConsoleLogger log) => _log = log;

            public void Write(ReadOnlySpan<byte> data)
            {
                // Loopback for simulation
                lock (_gate)
                {
                    for (int i = 0; i < data.Length; i++) _rx.Enqueue(data[i]);
                }
            }

            public int Read(Span<byte> buffer)
            {
                lock (_gate)
                {
                    int n = Math.Min(buffer.Length, _rx.Count);
                    for (int i = 0; i < n; i++) buffer[i] = _rx.Dequeue();
                    return n;
                }
            }
        }

        private sealed class VirtualMmio : IMmio
        {
            private readonly PrecisionControl.ConsoleLogger _log;
            private byte[]? _mapped;
            public VirtualMmio(PrecisionControl.ConsoleLogger log) => _log = log;

            public Span<byte> Map(ulong physAddr, int length)
            {
                _mapped = new byte[length];
                return _mapped.AsSpan();
            }

            public void Unmap()
            {
                _mapped = null!;
            }

            public void Dispose() => Unmap();
        }
    }

    // -----------------------------
    // Real HAL (Linux/ARM64 stubs)
    // -----------------------------
    internal sealed class RealHal : IHal
    {
        private readonly PrecisionControl.ConsoleLogger _log;
        public IGpio Gpio { get; }
        public IPwm Pwm { get; }
        public ICan Can { get; }
        public IUsbDevice Usb { get; }
        public IMmio Mmio { get; }
        public IClock Clock { get; } = new StopwatchClock();
        public bool UnmanagedPinTogglingEnabled => true;

        public RealHal(PrecisionControl.ConsoleLogger log)
        {
            _log = log;
            Gpio = new RealGpio(_log);
            Pwm = new RealPwm(_log);
            Can = new RealCan(_log);
            Usb = new RealUsb(_log);
            Mmio = new RealMmio(_log);
            _log.Info("Real HAL initialized (Linux/ARM64 stubs). Extend for actual device drivers.");
        }

        public void Dispose() => Mmio.Dispose();

        // The following are stubs that should be replaced by platform-specific bindings (libgpiod, spidev, SocketCAN, /dev/mem)
        private sealed class RealGpio : IGpio
        {
            private readonly PrecisionControl.ConsoleLogger _log;
            public RealGpio(PrecisionControl.ConsoleLogger log) => _log = log;
            public void SetPinMode(int pin, PinMode mode) { /* TODO: libgpiod */ }
            public void Write(int pin, bool value) { /* TODO: fast toggle */ }
            public bool Read(int pin) => false;
        }
        private sealed class RealPwm : IPwm
        {
            private readonly PrecisionControl.ConsoleLogger _log;
            public RealPwm(PrecisionControl.ConsoleLogger log) => _log = log;
            public void Configure(int channel, int frequencyHz, double duty01) { /* TODO */ }
            public void Stop(int channel) { /* TODO */ }
        }
        private sealed class RealCan : ICan
        {
            private readonly PrecisionControl.ConsoleLogger _log;
            public RealCan(PrecisionControl.ConsoleLogger log) => _log = log;
            public bool TrySend(ReadOnlySpan<byte> frame, int id, CancellationToken ct) { /* TODO: SocketCAN */ return true; }
            public bool TryReceive(Span<byte> buffer, out int id, out int len) { id = 0; len = 0; return false; }
        }
        private sealed class RealUsb : IUsbDevice
        {
            private readonly PrecisionControl.ConsoleLogger _log;
            public RealUsb(PrecisionControl.ConsoleLogger log) => _log = log;
            public void Write(ReadOnlySpan<byte> data) { /* TODO: libusbdotnet */ }
            public int Read(Span<byte> buffer) { /* TODO */ return 0; }
        }
        private sealed class RealMmio : IMmio
        {
            private readonly PrecisionControl.ConsoleLogger _log;
            private byte[]? _tmp; // Placeholder until /dev/mem mapping is implemented
            public RealMmio(PrecisionControl.ConsoleLogger log) => _log = log;
            public Span<byte> Map(ulong physAddr, int length)
            {
                _tmp = new byte[length];
                return _tmp.AsSpan();
            }
            public void Unmap() { _tmp = null; }
            public void Dispose() => Unmap();
        }
    }
}

// ============================================================================
// Runtime (Event Manager, Sampling Engine, Watchdog, Recorder)
// ============================================================================
namespace PrecisionControl.Runtime
{
    using PrecisionControl.Hal;

    // Lightweight ring buffer for recording critical state
    public sealed class StateRecorder
    {
        public readonly struct Entry
        {
            public readonly long Ticks;
            public readonly int Code;
            public readonly int A;
            public readonly int B;
            public Entry(long ticks, int code, int a, int b)
            {
                Ticks = ticks; Code = code; A = a; B = b;
            }
        }

        private readonly Entry[] _buffer;
        private int _writeIndex = -1;

        public StateRecorder(int capacity)
        {
            if (capacity <= 0 || (capacity & (capacity - 1)) != 0)
                throw new ArgumentException("Capacity must be power-of-two for fast modulo.");
            _buffer = new Entry[capacity];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Record(long ticks, int code, int a = 0, int b = 0)
        {
            int idx = unchecked(Interlocked.Increment(ref _writeIndex)) & (_buffer.Length - 1);
            _buffer[idx] = new Entry(ticks, code, a, b);
        }

        public ReadOnlySpan<Entry> Snapshot()
        {
            // Non-alloc view (not thread-safe for concurrent writes; use quickly)
            return _buffer.AsSpan();
        }
    }

    // Priority lanes for non-deterministic work (telemetry/logging)
    public sealed class EventManager
    {
        public enum Priority { High = 0, Normal = 1, Low = 2 }

        private readonly PrecisionControl.ConsoleLogger _log;

        private readonly Channel<ArraySegment<byte>> _high;
        private readonly Channel<ArraySegment<byte>> _normal;
        private readonly Channel<ArraySegment<byte>> _low;

        public EventManager(PrecisionControl.ConsoleLogger log)
        {
            _log = log;
            var opts = new BoundedChannelOptions(1024)
            {
                FullMode = BoundedChannelFullMode.DropOldest,
                SingleReader = true,
                SingleWriter = false
            };
            _high = Channel.CreateBounded<ArraySegment<byte>>(opts);
            _normal = Channel.CreateBounded<ArraySegment<byte>>(opts);
            _low = Channel.CreateBounded<ArraySegment<byte>>(opts);
        }

        public bool PublishTelemetry(ReadOnlySpan<byte> payload, Priority pri = Priority.Normal)
        {
            byte[] arr = ArrayPool<byte>.Shared.Rent(payload.Length);
            payload.CopyTo(arr);
            var seg = new ArraySegment<byte>(arr, 0, payload.Length);
            var lane = pri switch
            {
                Priority.High => _high,
                Priority.Low => _low,
                _ => _normal
            };
            return lane.Writer.TryWrite(seg);
        }

        public Task StartAsync(CancellationToken ct)
        {
            Task Worker(Channel<ArraySegment<byte>> ch, string laneName) => Task.Run(async () =>
            {
                try
                {
                    while (!ct.IsCancellationRequested)
                    {
                        if (await ch.Reader.WaitToReadAsync(ct).ConfigureAwait(false))
                        {
                            while (ch.Reader.TryRead(out var seg))
                            {
                                // Simulate upload/logging
                                int sum = 0;
                                var span = seg.AsSpan();
                                for (int i = 0; i < span.Length; i++) sum += span[i];
                                ArrayPool<byte>.Shared.Return(seg.Array!);
                            }
                        }
                    }
                }
                catch (OperationCanceledException) { /* normal */ }
                catch (Exception ex)
                {
                    _log.Warn($"EventManager {laneName} pipeline error: {ex.Message}");
                }
            }, ct);

            return Task.WhenAll(Worker(_high, "high"), Worker(_normal, "normal"), Worker(_low, "low"));
        }
    }

    public interface IWatchdog
    {
        void Kick(long nowTicks);
    }

    public sealed class VirtualWatchdog : IWatchdog
    {
        private readonly PrecisionControl.ConsoleLogger _log;
        private readonly long _timeoutTicks;
        private long _lastKick;
        private int _warned;

        public VirtualWatchdog(PrecisionControl.ConsoleLogger log, TimeSpan timeout, Hal.IClock? clock = null)
        {
            _log = log;
            var c = clock ?? new Hal.StopwatchClock();
            _timeoutTicks = c.TicksFromSeconds(timeout.TotalSeconds);
            _lastKick = c.Timestamp();
        }

        public void Kick(long nowTicks)
        {
            long delta = nowTicks - Interlocked.Exchange(ref _lastKick, nowTicks);
            // Lazy warning when exceeding timeout measured between kicks
            if (delta > _timeoutTicks && Interlocked.Exchange(ref _warned, 1) == 0)
            {
                _log.Warn($"Watchdog late kick detected: deltaTicks={delta}");
            }
            else if (delta <= _timeoutTicks && _warned != 0)
            {
                Interlocked.Exchange(ref _warned, 0);
            }
        }
    }

    // Deterministic Sampling Engine with drift correction scaffolding
    public sealed class SamplingEngine
    {
        private readonly IHal _hal;
        private readonly EventManager _events;
        private readonly StateRecorder _rec;
        private readonly IWatchdog _wd;
        private readonly PrecisionControl.ConsoleLogger _log;
        private readonly int _hz;

        public SamplingEngine(IHal hal, EventManager events, StateRecorder recorder, IWatchdog watchdog, PrecisionControl.ConsoleLogger log, int sampleHz)
        {
            _hal = hal; _events = events; _rec = recorder; _wd = watchdog; _log = log; _hz = Math.Max(1, sampleHz);
        }

        public async Task StartAsync(CancellationToken ct)
        {
            await Task.Yield();

            var clock = _hal.Clock;
            long tps = clock.TicksPerSecond;
            long period = tps / _hz;
            long next = clock.Timestamp();

            // pre-allocate one buffer reused each sample to avoid GC
            byte[] tx = ArrayPool<byte>.Shared.Rent(16);

            try
            {
                while (!ct.IsCancellationRequested)
                {
                    long now = clock.Timestamp();
                    long behind = now - next;
                    if (behind >= 0)
                    {
                        // Execute sample
                        _wd.Kick(now);
                        _rec.Record(now, code: 1 /*tick*/, a: (int)Math.Min(int.MaxValue, behind), b: 0);

                        // Example: publish 8-byte timestamp telemetry
                        Unsafe.WriteUnaligned(ref tx[0], now);
                        _events.PublishTelemetry(tx.AsSpan(0, 8));

                        // Drift correction: step forward by integral periods
                        long steps = 1 + (behind / period);
                        next += steps * period;
                    }
                    else
                    {
                        // Sleep/yield a little until next
                        // Use a short delay to reduce spin; for higher precision on RT kernels, replace with nanosleep via P/Invoke.
                        int delayMs = (int)Math.Max(0, (clock.SecondsFromTicks(-behind) * 1000.0) - 0.2);
                        if (delayMs > 0)
                            await Task.Delay(delayMs, ct).ConfigureAwait(false);
                        else
                            await Task.Yield();
                    }
                }
            }
            catch (OperationCanceledException) { /* normal */ }
            catch (Exception ex)
            {
                _log.Error($"SamplingEngine error: {ex.Message}");
            }
            finally
            {
                ArrayPool<byte>.Shared.Return(tx);
            }
        }
    }
}

// ============================================================================
// Scripting (tiny DSL: WAIT, GPIO_SET, PWM_SET, LOOP)
// ============================================================================
namespace PrecisionControl.Scripting
{
    using PrecisionControl.Hal;

    public static class ScriptEngine
    {
        public readonly struct CompiledScript
        {
            public readonly Instruction[] Program;
            public CompiledScript(Instruction[] program) { Program = program; }
        }

        public enum OpCode : byte { WaitMs, GpioSet, PwmSet }

        public readonly struct Instruction
        {
            public readonly OpCode Op;
            public readonly int A;
            public readonly int B;
            public readonly double D;
            public Instruction(OpCode op, int a = 0, int b = 0, double d = 0) { Op = op; A = a; B = b; D = d; }
        }

        public static CompiledScript Compile(string script, PrecisionControl.ConsoleLogger log)
        {
            var tokens = new List<string>(256);
            var program = new List<Instruction>(256);

            using var reader = new System.IO.StringReader(script);
            string? line;
            int lineNo = 0;

            while ((line = reader.ReadLine()) != null)
            {
                lineNo++;
                var trimmed = line.Trim();
                if (trimmed.Length == 0) continue;
                if (trimmed.StartsWith("//")) continue;

                // Handle LOOP n { ... }
                if (trimmed.StartsWith("LOOP", StringComparison.OrdinalIgnoreCase))
                {
                    // Expected: LOOP count {  ...  }
                    int count = 0;
                    int braceOpen = trimmed.IndexOf('{');
                    if (braceOpen < 0)
                    {
                        // maybe next line has {
                        // parse "LOOP n"
                        var parts = trimmed.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                        if (parts.Length < 2 || !int.TryParse(parts[1], out count) || count <= 0) continue;

                        // read until encountering '{'
                        string? next;
                        do { next = reader.ReadLine(); lineNo++; } while (next != null && next.Trim().Length == 0);
                        if (next == null || !next.TrimStart().StartsWith("{"))
                            throw new FormatException($"Line {lineNo}: Expected '{{' after LOOP.");
                    }
                    else
                    {
                        // parse "LOOP n {"
                        var head = trimmed.Substring(0, braceOpen).Trim();
                        var parts = head.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                        if (parts.Length < 2 || !int.TryParse(parts[1], out count) || count <= 0) continue;
                    }

                    // Gather block lines until '}'
                    var block = new List<Instruction>(32);
                    while (true)
                    {
                        var body = reader.ReadLine();
                        lineNo++;
                        if (body == null) throw new FormatException($"Line {lineNo}: Unterminated LOOP block.");
                        var bodyTrim = body.Trim();
                        if (bodyTrim.StartsWith("}")) break;
                        if (bodyTrim.Length == 0 || bodyTrim.StartsWith("//")) continue;
                        EmitInstruction(bodyTrim, block);
                    }

                    for (int i = 0; i < count; i++) program.AddRange(block);
                    continue;
                }

                EmitInstruction(trimmed, program);
            }

            return new CompiledScript(program.ToArray());
        }

        private static void EmitInstruction(ReadOnlySpan<char> trimmed, List<Instruction> program)
        {
            // Commands:
            // WAIT ms
            // GPIO_SET pin value
            // PWM_SET channel freqHz duty01
            if (StartsWith(trimmed, "WAIT"))
            {
                var parts = trimmed.ToString().Split(' ', StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length >= 2 && int.TryParse(parts[1], out int ms) && ms >= 0)
                    program.Add(new Instruction(OpCode.WaitMs, a: ms));
                return;
            }
            if (StartsWith(trimmed, "GPIO_SET"))
            {
                var parts = trimmed.ToString().Split(' ', StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length >= 3 && int.TryParse(parts[1], out int pin) && int.TryParse(parts[2], out int val))
                    program.Add(new Instruction(OpCode.GpioSet, a: pin, b: (val != 0 ? 1 : 0)));
                return;
            }
            if (StartsWith(trimmed, "PWM_SET"))
            {
                var parts = trimmed.ToString().Split(' ', StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length >= 4 && int.TryParse(parts[1], out int ch) && int.TryParse(parts[2], out int freq) && double.TryParse(parts[3], out double duty))
                    program.Add(new Instruction(OpCode.PwmSet, a: ch, b: freq, d: duty));
                return;
            }
        }

        private static bool StartsWith(ReadOnlySpan<char> s, string value)
            => s.StartsWith(value.AsSpan(), StringComparison.OrdinalIgnoreCase);

        public static async Task RunAsync(CompiledScript compiled, IHal hal, PrecisionControl.ConsoleLogger log, CancellationToken ct)
        {
            await Task.Yield();
            _ = Task.Run(async () =>
            {
                try
                {
                    var program = compiled.Program;
                    for (int pc = 0; !ct.IsCancellationRequested && pc < program.Length; pc++)
                    {
                        var ins = program[pc];
                        switch (ins.Op)
                        {
                            case OpCode.WaitMs:
                                if (ins.A > 0) await Task.Delay(ins.A, ct).ConfigureAwait(false);
                                break;

                            case OpCode.GpioSet:
                                hal.Gpio.SetPinMode(ins.A, PinMode.Output);
                                hal.Gpio.Write(ins.A, ins.B != 0);
                                break;

                            case OpCode.PwmSet:
                                hal.Pwm.Configure(ins.A, ins.B, ins.D);
                                break;
                        }
                    }
                }
                catch (OperationCanceledException) { /* normal */ }
                catch (Exception ex)
                {
                    log.Warn($"Script runtime error: {ex.Message}");
                }
            }, ct);
        }
    }
}

// ============================================================================
// Analytics (FFT + streaming quantile stubs)
// ============================================================================
namespace PrecisionControl.Analytics
{
    // Naive FFT wrapper using System.Numerics.Complex for demonstration.
    // For embedded/RT use, replace with fixed-size, allocation-free implementation.
    public static class SimpleFft
    {
        public static void Dft(ReadOnlySpan<double> input, Span<Complex> output)
        {
            int n = input.Length;
            if (output.Length < n) throw new ArgumentException("Output too small.");
            for (int k = 0; k < n; k++)
            {
                double sumRe = 0, sumIm = 0;
                for (int t = 0; t < n; t++)
                {
                    double angle = -2.0 * Math.PI * t * k / n;
                    double v = input[t];
                    sumRe += v * Math.Cos(angle);
                    sumIm += v * Math.Sin(angle);
                }
                output[k] = new Complex(sumRe, sumIm);
            }
        }
    }

    // P2 quantile estimator (very simplified placeholder)
    public sealed class StreamingQuantile
    {
        private readonly double _phi; // desired quantile (0..1)
        private double _estimate;
        private bool _initialized;

        public StreamingQuantile(double phi)
        {
            _phi = Math.Clamp(phi, 0.0, 1.0);
        }

        public void Add(double x)
        {
            if (!_initialized) { _estimate = x; _initialized = true; return; }
            // move estimate towards x with small step based on phi
            double step = 0.01;
            _estimate += step * Math.Sign(x - _estimate) * (_phi >= 0.5 ? 1 : -1);
        }

        public double Estimate => _estimate;
    }
}

// ============================================================================
// Security (AES-GCM + HMAC stubs for CAN payloads and config vault hooks)
// ============================================================================
namespace PrecisionControl.Security
{
    public static class SecurePayload
    {
        public static int EncryptAesGcm(ReadOnlySpan<byte> key, ReadOnlySpan<byte> nonce, ReadOnlySpan<byte> plaintext, Span<byte> ciphertext, Span<byte> tag, ReadOnlySpan<byte> aad = default)
        {
            using var gcm = new AesGcm(key);
            gcm.Encrypt(nonce, plaintext, ciphertext, tag, aad);
            return plaintext.Length;
        }

        public static int DecryptAesGcm(ReadOnlySpan<byte> key, ReadOnlySpan<byte> nonce, ReadOnlySpan<byte> ciphertext, ReadOnlySpan<byte> tag, Span<byte> plaintext, ReadOnlySpan<byte> aad = default)
        {
            using var gcm = new AesGcm(key);
            gcm.Decrypt(nonce, ciphertext, tag, plaintext, aad);
            return ciphertext.Length;
        }

        public static void ComputeHmacSha256(ReadOnlySpan<byte> key, ReadOnlySpan<byte> data, Span<byte> dest)
        {
            using var h = new HMACSHA256(key.ToArray());
            var mac = h.ComputeHash(data.ToArray());
            mac.AsSpan().CopyTo(dest);
        }
    }
}

// ============================================================================
// Synchronization (PTP stubs)
// ============================================================================
namespace PrecisionControl.Sync
{
    public interface IClockSync
    {
        // Adjust local time estimation based on PTP messages (stub)
        void UpdateOffset(double seconds);
        double CurrentOffsetSeconds { get; }
    }

    public sealed class PtpSync : IClockSync
    {
        private double _offset;
        public void UpdateOffset(double seconds) => _offset = seconds;
        public double CurrentOffsetSeconds => _offset;
    }
}
