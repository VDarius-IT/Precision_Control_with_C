using System;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

using PrecisionControl.Hal;
using PrecisionControl.Runtime;
using PrecisionControl.Scripting;

namespace PrecisionControl;

public static class Program
{
    public static async Task<int> Main(string[] args)
    {
        var logger = new ConsoleLogger(minLevel: LogLevel.Info);

        try
        {
            var backend = ParseHalBackend(args, Environment.GetEnvironmentVariable("HAL_BACKEND"));
            int sampleHz = ParseIntArg(args, "--rate", defaultValue: backend == HalBackend.Real ? 1000 : 200);
            logger.Info($"Precision Control starting. HAL={backend}, SampleRate={sampleHz} Hz");

            using var cts = new CancellationTokenSource();
            Console.CancelKeyPress += (_, e) =>
            {
                e.Cancel = true;
                logger.Warn("Ctrl+C requested, shutting down...");
                cts.Cancel();
            };

            // Build subsystem graph
            IHal hal = HalFactory.Create(backend, logger);
            var recorder = new StateRecorder(capacity: 65536);
            var eventMgr = new EventManager(logger);
            var watchdog = new VirtualWatchdog(logger, timeout: TimeSpan.FromSeconds(2));
            var sampling = new SamplingEngine(hal, eventMgr, recorder, watchdog, logger, sampleHz);

            // Start non-deterministic pipelines
            var eventTask = eventMgr.StartAsync(cts.Token);

            // Load a mission script (optional: scripts/mission.pctl), otherwise use a default inline script
            string scriptText = TryLoadScriptFromDisk("scripts/mission.pctl", logger)
                ?? DefaultMissionScript();

            var compiled = ScriptEngine.Compile(scriptText, logger);
            var scriptTask = ScriptEngine.RunAsync(compiled, hal, logger, cts.Token);

            // Start deterministic sampling/control loop
            var samplingTask = sampling.StartAsync(cts.Token);

            logger.Info("Runtime active. Press Ctrl+C to stop.");
            await Task.WhenAny(Task.WhenAll(eventTask, scriptTask, samplingTask), WaitForCancellation(cts.Token));

            // Graceful stop
            cts.Cancel();
            await Task.WhenAll(Suppress(eventTask), Suppress(scriptTask), Suppress(samplingTask));

            logger.Info("Precision Control stopped.");
            return 0;
        }
        catch (Exception ex)
        {
            logger.Error($"Fatal: {ex}");
            return 1;
        }
    }

    private static HalBackend ParseHalBackend(string[] args, string? env)
    {
        string? cli = GetArgValue(args, "--hal");
        string? value = cli ?? env;
        return value?.Equals("real", StringComparison.OrdinalIgnoreCase) == true
            ? HalBackend.Real
            : HalBackend.Virtual;
    }

    private static int ParseIntArg(string[] args, string name, int defaultValue)
    {
        string? v = GetArgValue(args, name);
        return int.TryParse(v, out var n) && n > 0 ? n : defaultValue;
    }

    private static string? GetArgValue(string[] args, string name)
    {
        // Accept formats: --name=value or --name value
        for (int i = 0; i < args.Length; i++)
        {
            var a = args[i];
            if (a.StartsWith(name + "=", StringComparison.OrdinalIgnoreCase))
            {
                return a[(name.Length + 1)..];
            }
            if (a.Equals(name, StringComparison.OrdinalIgnoreCase) && i + 1 < args.Length)
            {
                return args[i + 1];
            }
        }
        return null;
    }

    private static string? TryLoadScriptFromDisk(string path, ConsoleLogger logger)
    {
        try
        {
            if (File.Exists(path))
            {
                logger.Info($"Loading mission script: {path}");
                return File.ReadAllText(path);
            }
        }
        catch (Exception ex)
        {
            logger.Warn($"Script load failed ({path}): {ex.Message}");
        }
        return null;
    }

    private static string DefaultMissionScript()
    {
        // A tiny allocation-aware DSL; compiled once, executed without extra allocations.
        // Commands:
        //   WAIT ms
        //   GPIO_SET pin value
        //   PWM_SET channel freqHz duty01
        //   LOOP count { ... }
        return """
        // Default mission: blink GPIO 17 with 5 Hz equivalent via waits
        LOOP 10 {
            GPIO_SET 17 1
            WAIT 100
            GPIO_SET 17 0
            WAIT 100
        }
        """;
    }

    private static Task WaitForCancellation(CancellationToken token)
        => Task.Run(() => token.WaitHandle.WaitOne(), token);

    private static async Task Suppress(Task t)
    {
        try { await t; } catch { /* swallow during shutdown */ }
    }
}

// Minimal logging facility
public enum LogLevel { Trace = 0, Debug = 1, Info = 2, Warn = 3, Error = 4 }

public sealed class ConsoleLogger
{
    private readonly object _gate = new();
    private readonly LogLevel _min;

    public ConsoleLogger(LogLevel minLevel = LogLevel.Info) => _min = minLevel;

    public void Trace(string msg) => Write(LogLevel.Trace, msg);
    public void Debug(string msg) => Write(LogLevel.Debug, msg);
    public void Info(string msg)  => Write(LogLevel.Info, msg);
    public void Warn(string msg)  => Write(LogLevel.Warn, msg);
    public void Error(string msg) => Write(LogLevel.Error, msg);

    private void Write(LogLevel level, string msg)
    {
        if (level < _min) return;
        var prefix = level switch
        {
            LogLevel.Trace => "[TRC]",
            LogLevel.Debug => "[DBG]",
            LogLevel.Info  => "[INF]",
            LogLevel.Warn  => "[WRN]",
            LogLevel.Error => "[ERR]",
            _ => "[LOG]"
        };
        lock (_gate)
        {
            Console.WriteLine($"{DateTime.UtcNow:O} {prefix} {msg}");
        }
    }
}
