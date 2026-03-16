# Asynchronous Drone Client-Server (KrattAssignment)

A UDP client-server application demonstrating high-performance, error-resilient asynchronous communication between a simulated drone and a ground control station (GCS). Communication passes through a proxy UDP bridge that simulates extreme packet drop scenarios (up to 75%+), ensuring reliable drone control over an unreliable link.

## Table of Contents

- [Asynchronous Drone Client-Server (KrattAssignment)](#asynchronous-drone-client-server-krattassignment)
  - [Table of Contents](#table-of-contents)
  - [Architecture](#architecture)
  - [Components](#components)
    - [Drone](#drone)
    - [Ground Control Station (GCS)](#ground-control-station-gcs)
    - [Proxy](#proxy)
  - [Requirements](#requirements)
  - [Building](#building)
  - [Running](#running)
  - [Testing](#testing)
  - [Project Structure](#project-structure)
  - [Communication Protocol](#communication-protocol)
    - [UDP Ports](#udp-ports)
  - [Threading Model](#threading-model)
  - [Libraries](#libraries)

## Architecture

```
┌──────────────────────────────────────────────────────┐
│               Ground Control Station                 │
│                                                      │
│  ┌──────────────────────────────────────────────┐    │
│  │  ImGui UI (Map + Telemetry + Controls)       │    │
│  │  - Click map to send waypoints               │    │
│  │  - Heartbeat monitor (3s timeout)            │    │
│  │  - Packet drop slider                        │    │
│  └──────────────────┬───────────────────────────┘    │
│                     │                                │
│  ┌──────────────────▼───────────────────────────┐    │
│  │  Proxy (UDP Bridge / Radio Simulator)        │    │
│  │  - Configurable packet drop (0–100%)         │    │
│  │  - Rate limit: 32 KB/s                       │    │
│  │  - Buffer limit: 1024 bytes                  │    │
│  │  - Simulated latency per packet              │    │
│  └──────────────────┬───────────────────────────┘    │
└─────────────────────┬────────────────────────────────┘
                      │ UDP (port 14550)
                      │
┌─────────────────────▼────────────────────────────────┐
│                     Drone                            │
│                                                      │
│  ┌────────────┐  ┌─────────────┐  ┌───────────────┐  │
│  │ Main Thread│  │ Beacon (10Hz│  │ Communicator  │  │
│  │ (Simulation│  │  Heartbeat) │  │ (Send + Recv  │  │
│  │ + Commands)│  │             │  │   threads)    │  │
│  └────────────┘  └─────────────┘  └───────────────┘  │
│                                                      │
│  ┌──────────────────────────────────────────────┐    │
│  │  Drone State: X, Y, Alt, Speed, Heading      │    │
│  │  Mode: GUIDED_DISARMED / GUIDED_ARMED /      │    │
│  │        AUTO_ARMED                            │    │
│  └──────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────┘
```

## Components

### Drone

The drone is a UDP client that simulates a quadrotor with minimal state:

- **3D State**: Local X, Y, Altitude, Speed (m/s), and Heading (degrees)
- **Modes**: `MAV_MODE_GUIDED_DISARMED` (80), `MAV_MODE_GUIDED_ARMED` (88), `MAV_MODE_AUTO_ARMED` (92). Starts as `GUIDED_ARMED`.
- **Telemetry stream at 10 Hz**: Sends `HEARTBEAT`, `LOCAL_POSITION_NED`, and `GLOBAL_POSITION_INT` messages
- **Accepts commands**:
  - `SET_POSITION_TARGET_GLOBAL_INT` — move to a waypoint (ACKed with `POSITION_TARGET_GLOBAL_INT`)
  - `MAV_CMD_OVERRIDE_GOTO` (DO_HOLD) — hold position (ACKed with `COMMAND_ACK`)
- MAVLink communication runs on separate threads and does not block the main simulation thread

### Ground Control Station (GCS)

The GCS is a UDP server with an ImGui-based debug GUI:

- **Connection status**: Connected/Disconnected indicator with connection quality percentage
- **Telemetry display**: X, Y, Altitude, Speed, Heading, MAV_MODE
- **Drone control**: Send `SET_POSITION_TARGET_GLOBAL_INT` and `MAV_CMD_OVERRIDE_GOTO` commands. All commands are ACKed.
- **2D Map visualization**: Top-down drone position display over a map of Ülemiste (click to set waypoints)
- **Proxy controls**: Packet drop percentage slider in the UI
- MAVLink communication runs on separate threads and does not block the main GUI thread

### Proxy

The proxy is a UDP bridge integrated into the GCS application that simulates realistic radio link conditions:

- **Packet drop**: Configurable 0–100% random packet loss
- **Data rate limit**: 32 KB/s (simulating a constrained radio system)
- **Buffer limit**: 1024 bytes — packets exceeding the buffer are dropped
- **Latency simulation**: Packet delay based on size (`packetDelay = packetLen × timePerByte`)
- **Discovery**: Automatic drone-proxy discovery handshake via special marker packets
- Uses a priority queue for time-ordered packet delivery

## Requirements

- **OS**: Ubuntu (tested on 22.04) or Windows
- **Compiler**: G++ with C++20 support
- **CMake**: Version 3.20 or newer
- **Python**: Required for MAVLink header generation (Python 3.x)
- **OpenGL / GLFW**: Required for the GCS GUI

On Ubuntu, install dependencies:

```bash
sudo apt update
sudo apt install build-essential cmake libglfw3-dev libgl1-mesa-dev python3
```

## Building

A convenience script is provided for clean builds:

```bash
chmod +x compile.sh
./compile.sh
```

Or build manually:

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

Executables are placed in `build/Drone/` and `build/GroundControl/`.

## Running

Start the GCS first, then the drone. Both communicate through the proxy on `localhost`.

**Terminal 1 — Ground Control Station:**

```bash
cd build/GroundControl
./GroundControl
```

**Terminal 2 — Drone:**

```bash
cd build/Drone
./Drone
```

The GCS window will display the map, telemetry data, and controls. Click on the map to send waypoint commands to the drone. Use the packet drop slider to test resilience under simulated packet loss.

## Testing

Unit tests are built with Google Test and Google Mock. They cover server and client communication patterns, including a stress test under 75% packet loss.

```bash
cd build/test
./UnitTests
```

A standalone heartbeat/parameter test is also available:

```bash
cd build/Drone
./HeartBeat
```

## Project Structure

```
KrattAssignment/
├── CMakeLists.txt              # Root build configuration
├── compile.sh                  # Clean build script
├── README.md
├── Drone/
│   ├── CMakeLists.txt          # Drone build config
│   ├── main.cpp                # Entry point, signal handling, command dispatch
│   ├── drone.h / drone.cpp     # Drone state container (position + mode)
│   ├── drone_state.h / .cpp    # Position/velocity state management
│   ├── drone_status.h          # MAV_MODE enum definitions
│   ├── communicator.h / .cpp   # UDP communication (send/recv threads, MAVLink)
│   ├── beacon.h / beacon.cpp   # 10 Hz heartbeat broadcaster
│   ├── threadsafe_queue.h      # Thread-safe queue with C++20 stop token
│   └── test_heartbeat.cpp      # Standalone heartbeat + parameter test
├── GroundControl/
│   ├── CMakeLists.txt          # GCS build config
│   ├── main.cpp                # ImGui UI, telemetry display, waypoint control
│   ├── proxy.h / proxy.cpp     # UDP proxy with packet drop/rate limiting
│   └── images/
│       └── ülemiste.jpg        # Map overlay for 2D drone visualization
├── test/
│   ├── CMakeLists.txt          # Unit test build config
│   └── main.cpp                # Google Test/Mock tests for communication
└── libs/                       # Third-party libraries (included as source)
    ├── c_library_v2/           # MAVLink C headers
    ├── glfw/                   # GLFW windowing library
    ├── googletest/             # Google Test framework
    ├── imgui/                  # Dear ImGui (immediate-mode GUI)
    ├── mavlink/                # MAVLink build support
    ├── SimpleUDP/              # UDP networking library
    └── stb/                    # stb_image for texture loading
```

## Communication Protocol

All communication uses **MAVLink v2** over UDP.

| Message                          | ID | Direction     | Purpose                              |
|----------------------------------|----|---------------|--------------------------------------|
| `HEARTBEAT`                      | 0  | Drone → GCS   | 10 Hz alive signal, mode reporting   |
| `LOCAL_POSITION_NED`             | 32 | Drone → GCS   | X, Y position                        |
| `GLOBAL_POSITION_INT`            | 33 | Drone → GCS   | Altitude, speed, heading             |
| `SET_POSITION_TARGET_GLOBAL_INT` | 86 | GCS → Drone   | Waypoint command                     |
| `COMMAND_LONG`                   | 76 | GCS → Drone   | `MAV_CMD_OVERRIDE_GOTO` (hold)       |
| `COMMAND_ACK`                    | 77 | Drone → GCS   | Command acknowledgement              |

Commands use a retry-and-ACK mechanism to guarantee delivery even under high packet loss (up to 2 second delay allowed).

### UDP Ports

| Component | Port  |
|-----------|-------|
| Drone     | 14563 |
| Proxy     | 14550 |

Discovery is automatic — the drone sends a discovery request and the proxy responds, establishing the communication link dynamically.

## Threading Model

The application uses **C++20 `std::jthread`** with stop tokens for graceful shutdown across all components.

| Component | Threads | Description                                               |
|-----------|---------|-----------------------------------------------------------|
| **Drone** | 3+      | Main (simulation + command handling), Communicator (send + receive), Beacon (heartbeat) |
| **GCS**   | 3+      | Main (ImGui render loop), Proxy recv thread, Proxy delivery thread |

Key concurrency patterns:
- **Thread-safe queue** (`ThreadSafeQueue<T>`): `std::mutex` + `std::condition_variable_any` with C++20 `std::stop_token` integration for clean shutdown
- **Priority queue**: Proxy uses `std::priority_queue` for time-ordered packet delivery
- **Atomic counters**: Proxy tracks `packets_forwarded` and `packets_dropped`
- **Mutex-protected shared state**: GCS telemetry data is safely shared between UI and proxy threads

## Libraries

All libraries are included as source in the `libs/` directory:

| Library                                                              | Purpose                        |
|----------------------------------------------------------------------|--------------------------------|
| [MAVLink (c_library_v2)](https://github.com/mavlink/c_library_v2)    | Communication protocol         |
| [SimpleUDP](https://github.com/RedFox20/SimpleUDP)                   | UDP networking                 |
| [Dear ImGui](https://github.com/ocornut/imgui)                       | GCS debug GUI                  |
| [GLFW](https://github.com/glfw/glfw)                                 | Window/OpenGL context for GUI  |
| [Google Test](https://github.com/google/googletest)                   | Unit testing framework         |
| [stb_image](https://github.com/nothings/stb)                         | Image loading for map texture  |
