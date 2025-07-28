#!/usr/bin/env python3

import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from intent_object_node import intent_object_node

def interactive_test():
    print("=" * 60)
    print("              INTERACTIVE INTENT OBJECT TEST")
    print("=" * 60)
    print("Enter voice commands to test the intent_object_node")
    print("Type 'quit', 'exit', or 'q' to stop")
    print("-" * 60)
    print()
    
    while True:
        try:
            user_input = input("Enter command: ").strip()
            
            if user_input.lower() in ['quit', 'exit', 'q']:
                print("\nExiting...")
                break
                
            if not user_input:
                print("Please enter a command\n")
                continue
                
            result = intent_object_node(user_input)
            print(f"Result: {result}")
            print()
            
        except KeyboardInterrupt:
            print("\n\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")
            print()

if __name__ == "__main__":
    interactive_test()