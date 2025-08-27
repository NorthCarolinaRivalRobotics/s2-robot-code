# Standalone Drivetrain Controller

This is a standalone async Python program that controls the mecanum drive system without using the Tide framework. It uses zenoh directly for communication.

## Features

- **Pure async Python**: No threading issues with the Tide framework
- **Direct zenoh communication**: Publishes to `/cash/state/twist` and listens on `/cash/cmd/twist`
- **Mecanum drive control**: Reuses the existing `MecanumDrive` class
- **IMU integration**: Listens to `/cash/sensor/gyro/vel` for enhanced odometry
- **Real-time feedback**: Queries wheel velocities and publishes robot state

## Usage

### Installation

Install the required dependencies:

```bash
# Install with robot dependencies (includes zenoh)
pip install -e ".[robot]"

# Or install zenoh separately if you already have other deps
pip install zenoh>=0.11.0
```

### Running the Controller

```bash
# Run the standalone drivetrain controller
python standalone_drivetrain.py
```

### Testing

In another terminal, run the test script:

```bash
python test_standalone_drivetrain.py
```

## Communication Topics

- **Subscribes to**: 
  - `/cash/cmd/twist` - Receives twist commands (linear x/y, angular z)
  - `/cash/sensor/gyro/vel` - Receives IMU angular velocity data
  
- **Publishes to**:
  - `/cash/state/twist` - Current robot twist state based on wheel velocities

## Message Format

### Command Twist (`/cash/cmd/twist`)
```json
{
  "linear": {"x": 0.5, "y": 0.0},
  "angular": 1.0
}
```

### State Twist (`/cash/state/twist`)
```json
{
  "linear": {"x": 0.48, "y": 0.02},
  "angular": 0.95
}
```

### IMU Angular Velocity (`/cash/sensor/gyro/vel`)
```json
{
  "x": 0.01,
  "y": -0.02,
  "z": 1.05
}
```

## Key Differences from Tide Version

1. **No threading issues**: Pure async/await pattern
2. **Direct zenoh**: No Tide abstractions or runtime
3. **Simplified initialization**: Direct async initialization 
4. **JSON messaging**: Simple JSON payloads instead of Tide serialization
5. **Independent operation**: Can run standalone without Tide infrastructure

## Control Loop

The controller runs a 30Hz control loop that:
- Processes incoming twist commands immediately
- Queries wheel velocities when idle (every ~3 seconds)
- Publishes state updates after each command
- Handles IMU data for enhanced odometry

## Error Handling

- Graceful initialization of mecanum drive
- Fallback to wheel-based angular velocity if IMU unavailable
- Robust error handling for zenoh communication
- Safe shutdown on interrupt

## Development

The standalone controller reuses most of the logic from the original Tide-based version but eliminates threading conflicts by using pure async/await patterns throughout.
