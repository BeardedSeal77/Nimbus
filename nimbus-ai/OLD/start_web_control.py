#!/usr/bin/env python3
"""
Startup script for Nimbus AI Web Control Server
Run this alongside AI.py to enable web-based control
"""

import sys
import os

# Add project paths
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

from web_control import run_web_server

if __name__ == '__main__':
    print("Starting Nimbus AI Web Control Server")
    print("=" * 50)
    print("This server provides web-based control for the AI system")
    print("Make sure AI.py is running in a separate terminal/process")
    print("")
    print("Controls available:")
    print("  - Trigger/Stop depth detection")
    print("  - Change target object")
    print("  - Adjust depth parameters")
    print("  - Reset system state")
    print("  - View real-time status")
    print("")
    
    try:
        # Run the web server
        run_web_server(host='localhost', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\n👋 Web control server stopped")
    except Exception as e:
        print(f"❌ Error starting web server: {e}")
        sys.exit(1)