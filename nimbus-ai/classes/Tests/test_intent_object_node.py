#!/usr/bin/env python3

import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from intent_object_node import intent_object_node

def test_intent_object_node():
    print("=" * 80)
    print("                        INTENT OBJECT NODE TESTS")
    print("=" * 80)
    print()
    
    test_cases = [
        "Go to the chair",
        "Move to the table", 
        "Navigate to the door",
        "Fly to the window",
        "Return home",
        "Go back to origin",
        "Stop flying",
        "Cancel current task",
        "Follow that person",
        "Search for the cup",
        "Find the red bottle",
        "Land on the floor",
        "Takeoff from here",
        "Hover near the building",
        "Approach the car slowly",
        "Track the moving vehicle",
        "Scan the room for objects",
        "Look for my phone",
        "Navigate to the kitchen",
        "Go to the bedroom door"
    ]
    
    print("NORMAL TEST CASES:")
    print("-" * 50)
    for transcript in test_cases:
        try:
            result = intent_object_node(transcript)
            print(f"'{transcript:<30}' -> {result}")
        except Exception as e:
            print(f"'{transcript:<30}' -> ERROR: {e}")
    
    print()

def test_edge_cases():
    print("EDGE CASES:")
    print("-" * 50)
    
    edge_cases = [
        "",
        "   ",
        "Hello there",
        "What's the weather like?",
        "Random sentence with no drone commands",
        "Go",
        "Chair",
        "Stop stop stop",
        "Navigate fly move go to the kitchen table chair",
        "123 numbers only",
        "!@#$%^&*()",
        "UPPERCASE COMMAND GO TO CHAIR",
        "return back home origin",
        "land takeoff descent down"
    ]
    
    for case in edge_cases:
        try:
            result = intent_object_node(case)
            display_case = case if case.strip() else "<empty>" if not case else "<spaces>"
            print(f"'{display_case:<30}' -> {result}")
        except Exception as e:
            display_case = case if case.strip() else "<empty>" if not case else "<spaces>"
            print(f"'{display_case:<30}' -> ERROR: {e}")
    
    print()
    print("=" * 80)

if __name__ == "__main__":
    test_intent_object_node()
    test_edge_cases()