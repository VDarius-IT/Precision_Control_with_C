# Präzisionssteuerung: Ein C#-Programm für eingebettete Systeme


![C#](https://img.shields.io/badge/C%23-239120?style=for-the-badge&logo=c-sharp&logoColor=white)
![.NET](https://img.shields.io/badge/.NET-512BD4?style=for-the-badge&logo=dotnet&logoColor=white)
![Embedded Systems](https://img.shields.io/badge/Embedded%20Systems-C70039?style=for-the-badge&logo=microchip&logoColor=white)
![Real-Time](https://img.shields.io/badge/Real--Time-0078D4?style=for-the-badge&logo=windows-terminal&logoColor=white)

> Eine hochleistungsfähige C#-Anwendung, die für die Anbindung an und die präzise Steuerung von Hardwarekomponenten in eingebetteten Systemen entwickelt wurde. Dieses Projekt legt den Schwerpunkt auf Echtzeit-Reaktionsfähigkeit, Speichereffizienz und Low-Level-Systemintegration und demonstriert erweiterte C#-Fähigkeiten jenseits der konventionellen Anwendungsentwicklung.

---

## 📌 Inhaltsverzeichnis
- [Projektübersicht](#-projektübersicht)
- [Wichtige Merkmale](#-wichtige-merkmale)
- [Architektur & Design](#️-architektur--design)
- [Hardware-Schnittstelle](#-hardware-schnittstelle)
- [Optimierungstechniken](#️-optimierungstechniken)
- [Erste Schritte](#-erste-schritte)
- [Verwendung](#-verwendung)
- [Best Practices & gewonnene Erkenntnisse](#-best-practices--gewonnene-erkenntnisse)
- [Mitwirken](#-mitwirken)

---

## 🔍 Projektübersicht

**Präzisionssteuerung** ist eine schlanke, leistungsorientierte C#-Anwendung, die für die Interaktion mit eingebetteten Hardwaresystemen unter strengen Ressourcenbeschränkungen entwickelt wurde. Dieses Projekt demonstriert die Eignung von C#, das traditionell in Unternehmensanwendungen eingesetzt wird, für Echtzeit-Steuerungsaufgaben, wenn es mit disziplinierter Optimierung entworfen wird.

Das System interagiert direkt mit GPIOs, Sensoren und Aktoren über plattformspezifische Treiber und Low-Level-APIs, was eine deterministische Ausführung für zeitkritische Operationen ermöglicht. Als Teil einer **Studie zur Systemintegration** untersucht dieses Projekt die Brücke zwischen verwaltetem .NET-Code und Bare-Metal-Hardwarefähigkeiten.

---

## ✨ Wichtige Merkmale

✅ **Echtzeit-Reaktionsfähigkeit**  
Optimierte Ausführungsschleifen und Interrupt-Simulation für zeitkritische Operationen.

✅ **Geringer Speicherbedarf**  
Aggressives Ressourcenmanagement mit dem Ziel, den RAM-Verbrauch auf Geräten mit beschränkten Ressourcen unter 50 MB zu halten.

✅ **Hardware-Abstraktionsschicht (HAL)**  
Modulares Treiberdesign, das die Wiederverwendung auf verschiedenen Plattformen ermöglicht (z. B. Raspberry Pi, Industriesteuerungen).

✅ **Direkter Peripheriezugriff**  
Integration mit GPIO, SPI, I2C und UART über P/Invoke und verwaltete Wrapper.

✅ **Plattformübergreifende Kompatibilität**  
Basiert auf .NET 8+ und ermöglicht die Bereitstellung auf Windows IoT, Linux ARM und mehr.

✅ **Diagnose & Protokollierung**  
Leichte Telemetrie und Fehlerverfolgung ohne Leistungseinbußen.

---

## 🏗️ Architektur & Design

Die Softwarearchitektur priorisiert **Determinismus** und **Trennung der Belange** (Separation of Concerns).

```text
+--------------------+ +------------------+ +-------------------+
| Anwendungs-UI | --> | Steuerkern | --> | Hardware-Treiber |
| (CLI / API-Schicht) | | (Zustand & Timing) | | (SPI, I2C, GPIO) |
+--------------------+ +------------------+ +-------------------+
|
+---------------------------------+
| Optimierte Algorithmen & Echtzeit |
+---------------------------------+
```
*   **Steuerkern:** Zentrale Logik für Timing, Steuerschleifen und Befehlssequenzierung. Kritische Schleifen laufen auf dedizierten Threads mit erhöhter Priorität.
*   **Hardware-Treiber:** Bietet eine konsistente Schnittstelle für die direkte Hardwarekommunikation (z. B. `libgpiod`, `DeviceIoControl`). Neue Geräte können durch Implementierung einer gemeinsamen Schnittstelle hinzugefügt werden.
*   **Optimierungen:** Kritische Pfade verwenden Techniken wie Object Pooling, `Span<T>` und sperrfreie Datenstrukturen, um Jitter und Pausen durch die Garbage Collection zu minimieren.

---

## 🔌 Hardware-Schnittstelle

Die Anwendung ist anpassungsfähig konzipiert und unterstützt derzeit:
- **Plattformen**: Raspberry Pi 4/5 (Linux), BeagleBone, Windows IoT Core-Geräte.
- **Protokolle**: I²C (für Sensoren wie BME280), SPI (für Hochgeschwindigkeits-ADCs), UART und GPIO.

📌 *Siehe `/docs/hardware-setup.md` für Schaltpläne und Pin-Konfigurationen.*

---

## ⚙️ Optimierungstechniken

Um die Echtzeitanforderungen zu erfüllen, wurden die folgenden Strategien angewendet und mit Werkzeugen wie `BenchmarkDotNet` validiert.

| Technik | Zweck |
|-------------------------------|--------------------------------------------------|
| **Objekt-Pooling** | Verwendet Objekte wieder, um den Druck auf die GC in engen Schleifen zu vermeiden. |
| **`Span<T>` & `Memory<T>`** | Ermöglicht die Datenmanipulation ohne Allokationen. |
| **Kein LINQ/Laufzeit-Reflection** | Reduziert Jitter und unvorhersehbaren Overhead. |
| **Puffer mit fester Größe** | Gewährleistet ein vorhersagbares Speicherlayout und Zugriffszeiten. |
| **Thread-Priorisierung & -Affinität** | Garantiert die rechtzeitige Ausführung auf dem Ziel-CPU-Kern. |
| **Ahead-of-Time (AOT) Kompilierung** | Erreicht schnelleren Start und geringeren Jitter durch Native AOT. |

📊 **Leistungsmetriken (Raspberry Pi 4, .NET 8):**
- **Durchschn. Schleifenlatenz:** ≤ 1,2 ms
- **Max. GC-Pause:** < 3 ms
- **CPU-Auslastung:** < 35 % bei 1-kHz-Regelrate
- **RAM-Nutzung:** ~42 MB im stabilen Zustand

---

## 🚀 Erste Schritte

### Voraussetzungen
- .NET 8 SDK oder neuer.
- Ein eingebettetes Zielgerät (z. B. Raspberry Pi mit Raspbian OS).
- SSH- & SCP-Zugang für die Remote-Bereitstellung.

### Installation & Bereitstellung

1.  **Repository klonen:**
    ```bash
    git clone https://github.com/your-username/precision-control.git
    cd precision-control
    ```
    <!-- Vergessen Sie nicht, 'your-username' in Ihren tatsächlichen GitHub-Benutzernamen zu ändern! -->

2.  **Abhängigkeiten wiederherstellen:**
    ```bash
    dotnet restore
    ```

3.  **Für das Zielgerät erstellen und veröffentlichen:**
    ```bash
    # Beispiel für ein 64-Bit-ARM-Linux-Ziel wie den Raspberry Pi
    dotnet publish -c Release -r linux-arm64 --self-contained true
    ```

4.  **Auf dem Gerät bereitstellen:**
    ```bash
    # Die veröffentlichten Dateien sicher auf das Zielgerät kopieren
    scp -r ./bin/Release/net8.0/linux-arm64/publish/* pi@<DEVICE_IP>:/home/pi/precision-control/
    ```

5.  **Auf dem Gerät ausführen:**
    ```bash
    # Per SSH auf dem Gerät anmelden und die Anwendung ausführen
    ssh pi@<DEVICE_IP>
    cd /home/pi/precision-control/
    sudo ./PrecisionControl
    ```
    *🔐 `sudo` kann für den direkten Hardwarezugriff erforderlich sein.*

---

## Verwendung

Das Programm wird über eine JSON-Datei konfiguriert und kann von der Kommandozeile aus gestartet werden.

**Beispiel für den Kommandozeilenaufruf:**
```bash
./PrecisionControl --config appsettings.json
```

Beispielkonfiguration in `appsettings.json`:
```json
{
  "Hardware": {
    "Bus": "SPI1",
    "BaudRate": 1000000,
    "Mode": "Mode0"
  },
  "ControlLoop": {
    "FrequencyHz": 1000,
    "MaxLatencyUs": 50
  },
  "Logging": {
    "FilePath": "telemetry.csv",
    "LogLevel": "Information"
  }
}
```

📌 Siehe `/docs/cli-guide.md` für erweiterte Verwendung und Kommandozeilenargumente.

---

## 🧠 Best Practices & gewonnene Erkenntnisse

*   C# ist für eingebettete Systeme geeignet, aber nur mit einem bewussten, leistungsorientierten Design.

*   Vermeiden Sie dynamische Allokationen in zeitkritischen Pfaden, um unvorhersehbare GC-Pausen zu verhindern.

*   Echtzeit ist in einer verwalteten Umgebung nicht garantiert; entwerfen Sie mit defensiven Prüfungen wie Watchdogs und Timeouts.

*   Alles profilieren. Nehmen Sie nicht an, wo sich Engpässe befinden. Testen Sie unter realer Last, nicht nur auf einer Entwicklungsmaschine.

*   Entwerfen Sie für eine kontrollierte Leistungsreduzierung (graceful degradation). Ihre Software sollte Hardwareausfälle bewältigen, ohne das gesamte System zum Absturz zu bringen.

Dieses Projekt war entscheidend für die Vertiefung meines Verständnisses von Systemintegration, schichtenübergreifendem Debugging und Leistungsoptimierung – Schlüsselfähigkeiten in der modernen industriellen Softwareentwicklung.

---

## 👋 Mitwirken

Beiträge sind willkommen! Dies ist ein persönliches Projekt, aber ich bin offen für Zusammenarbeit und Verbesserungen. Bitte befolgen Sie diese Schritte:

1.  Forken Sie das Repository.
2.  Erstellen Sie einen neuen Feature-Branch (`git checkout -b feature/my-awesome-feature`).
3.  Machen Sie Ihre Änderungen und fügen Sie ggf. Tests hinzu.
4.  Senden Sie einen Pull-Request mit einem aussagekräftigen Titel und Details zu Ihren Änderungen.
