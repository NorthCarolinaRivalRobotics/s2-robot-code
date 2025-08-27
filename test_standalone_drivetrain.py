#!/usr/bin/env python3
"""
Test script for the standalone drivetrain controller
"""

import asyncio
import json
import time
import zenoh
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_drivetrain():
    """Test the standalone drivetrain by sending commands and monitoring state."""
    
    # Open zenoh session
    config = zenoh.Config()
    session = zenoh.open(config)
    
    # Setup publishers and subscribers
    cmd_publisher = session.declare_publisher("cash/cmd/twist")
    
    def on_state_twist(sample):
        # Handle ZBytes properly
        if hasattr(sample.payload, 'decode'):
            payload_str = sample.payload.decode('utf-8')
        else:
            # ZBytes object - convert to bytes first
            payload_str = bytes(sample.payload).decode('utf-8')
        data = json.loads(payload_str)
        print(f"State: linear=({data['linear']['x']:.3f}, {data['linear']['y']:.3f}), angular={data['angular']:.3f}")
    
    state_subscriber = session.declare_subscriber("cash/state/twist", on_state_twist)
    
    try:
        print("Testing standalone drivetrain controller...")
        print("Make sure to run: python standalone_drivetrain.py in another terminal")
        
        await asyncio.sleep(2.0)  # Wait for startup
        
        # Test 1: Move forward
        print("\nTest 1: Moving forward...")
        cmd = {"linear": {"x": 0.5, "y": 0.0}, "angular": 0.0}
        cmd_publisher.put(json.dumps(cmd))
        await asyncio.sleep(3.0)
        
        # Test 2: Stop
        print("\nTest 2: Stopping...")
        cmd = {"linear": {"x": 0.0, "y": 0.0}, "angular": 0.0}
        cmd_publisher.put(json.dumps(cmd))
        await asyncio.sleep(2.0)
        
        # Test 3: Strafe right
        print("\nTest 3: Strafing right...")
        cmd = {"linear": {"x": 0.0, "y": 0.3}, "angular": 0.0}
        cmd_publisher.put(json.dumps(cmd))
        await asyncio.sleep(3.0)
        
        # Test 4: Rotate
        print("\nTest 4: Rotating...")
        cmd = {"linear": {"x": 0.0, "y": 0.0}, "angular": 1.0}
        cmd_publisher.put(json.dumps(cmd))
        await asyncio.sleep(3.0)
        
        # Test 5: Stop
        print("\nTest 5: Final stop...")
        cmd = {"linear": {"x": 0.0, "y": 0.0}, "angular": 0.0}
        cmd_publisher.put(json.dumps(cmd))
        await asyncio.sleep(2.0)
        
        print("\nTest completed!")
        
    except KeyboardInterrupt:
        print("Test interrupted")
    finally:
        # Final stop command
        cmd = {"linear": {"x": 0.0, "y": 0.0}, "angular": 0.0}
        cmd_publisher.put(json.dumps(cmd))
        session.close()

if __name__ == "__main__":
    asyncio.run(test_drivetrain())
