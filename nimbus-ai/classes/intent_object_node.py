import requests
import json
import logging
from typing import Dict, Optional

logger = logging.getLogger(__name__)

OLLAMA_URL = "http://localhost:11434/api/generate"
MODEL_NAME = "llama3.1:latest"

FINAL_INTENTS = ["go", "stop", "cancel", "home", "land"]

INTENT_MAPPINGS = {
    "go": ["go", "move", "navigate", "fly", "travel", "approach", "follow", "track", "monitor", "search", "find", "scan", "survey", "look"],
    "stop": ["stop", "halt", "pause", "hover"],
    "cancel": ["cancel", "abort", "quit", "exit"],
    "home": ["home", "return", "back", "origin"],
    "land": ["land", "takeoff", "descent", "down"]
}

COMMON_OBJECTS = [
    "chair", "table", "door", "window", "wall", "floor", "ceiling",
    "person", "human", "face", "hand",
    "car", "vehicle", "bike", "motorcycle",
    "tree", "plant", "flower", "grass",
    "building", "house", "room", "kitchen", "bedroom",
    "phone", "computer", "laptop", "screen", "monitor",
    "cup", "bottle", "glass", "plate", "bowl",
    "book", "paper", "box", "bag"
]

SYSTEM_PROMPT = f"""You are an AI assistant for a drone navigation system. Your task is to extract the intent and target object from voice commands.

INSTRUCTIONS:
1. Analyze the input text and identify the main action (intent) and target object
2. Return ONLY a valid JSON object with 'intent' and 'object' fields
3. The intent MUST be one of these 5 options ONLY: {', '.join(FINAL_INTENTS)}
4. Map variations to the correct intent (e.g., "navigate" → "go", "return" → "home", "halt" → "stop")
5. Objects should be specific nouns, not descriptions

INTENT MAPPING RULES:
- "go": Use for movement, navigation, approach, follow, search, find, scan, survey, look
- "stop": Use for halt, pause, hover (staying in place)
- "cancel": Use for abort, quit, exit (canceling current task)
- "home": Use for return, back, origin (going to starting position)
- "land": Use for landing, takeoff, descent, going down

COMMON OBJECTS (use these when appropriate, but you're not limited to them):
{', '.join(COMMON_OBJECTS)}

EXAMPLES:
Input: "Go to the chair"
Output: {{"intent": "go", "object": "chair"}}

Input: "Navigate towards the red door"
Output: {{"intent": "go", "object": "door"}}

Input: "Return home"
Output: {{"intent": "home", "object": ""}}

Input: "Stop flying"
Output: {{"intent": "stop", "object": ""}}

Input: "Follow that person"
Output: {{"intent": "go", "object": "person"}}

Input: "Halt immediately"
Output: {{"intent": "stop", "object": ""}}

Input: "Land on the ground"
Output: {{"intent": "land", "object": "ground"}}

Respond with ONLY the JSON object, no other text or explanation."""

def intent_object_node(transcript: str) -> Dict[str, str]:
    """
    Extract intent and object from transcript using Ollama LLM.
    
    Args:
        transcript (str): The voice command transcript
        
    Returns:
        Dict[str, str]: Dictionary with 'intent' and 'object' keys
    """
    if not transcript or transcript.strip() == "":
        logger.warning("Empty transcript provided")
        return {"intent": "", "object": ""}
    
    try:
        payload = {
            "model": MODEL_NAME,
            "prompt": transcript,
            "system": SYSTEM_PROMPT,
            "stream": False,
            "options": {
                "temperature": 0.1,
                "top_p": 0.9,
                "max_tokens": 100
            }
        }
        
        logger.info(f"Sending request to Ollama for transcript: '{transcript}'")
        response = requests.post(OLLAMA_URL, json=payload, timeout=30)
        response.raise_for_status()
        
        result = response.json()
        ai_response = result.get('response', '').strip()
        
        logger.debug(f"Raw AI response: {ai_response}")
        
        try:
            parsed_response = json.loads(ai_response)
            raw_intent = parsed_response.get('intent', '').lower().strip()
            obj = parsed_response.get('object', '').lower().strip()
            
            # Map to final intent
            final_intent = _map_to_final_intent(raw_intent)
            
            result_dict = {
                "intent": final_intent,
                "object": obj
            }
            
            logger.info(f"Successfully extracted intent: '{final_intent}', object: '{obj}'")
            return result_dict
            
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse AI response as JSON: {ai_response}")
            logger.error(f"JSON decode error: {e}")
            
            fallback_result = _fallback_parse(transcript)
            logger.warning(f"Using fallback parsing: {fallback_result}")
            return fallback_result
            
    except requests.exceptions.RequestException as e:
        logger.error(f"Request to Ollama failed: {e}")
        fallback_result = _fallback_parse(transcript)
        logger.warning(f"Using fallback parsing due to connection error: {fallback_result}")
        return fallback_result
        
    except Exception as e:
        logger.error(f"Unexpected error in intent_object_node: {e}")
        return {"intent": "", "object": ""}

def _map_to_final_intent(raw_intent: str) -> str:
    """
    Map any intent variation to one of the 5 final intents.
    
    Args:
        raw_intent (str): The raw intent from AI or fallback
        
    Returns:
        str: One of the 5 final intents or empty string
    """
    if not raw_intent:
        return ""
    
    raw_intent = raw_intent.lower().strip()
    
    for final_intent, variations in INTENT_MAPPINGS.items():
        if raw_intent in variations:
            return final_intent
    
    # Default fallback - if it's already a final intent
    if raw_intent in FINAL_INTENTS:
        return raw_intent
    
    return ""

def _fallback_parse(transcript: str) -> Dict[str, str]:
    """
    Simple fallback parser when Ollama is unavailable.

    Args:
        transcript (str): The voice command transcript

    Returns:
        Dict[str, str]: Dictionary with 'intent' and 'object' keys
    """
    transcript_lower = transcript.lower().strip()

    intent = ""
    obj = ""

    # Check for intent mappings in priority order (more specific first)
    priority_order = ["home", "land", "stop", "cancel", "go"]
    for final_intent in priority_order:
        variations = INTENT_MAPPINGS[final_intent]
        if any(variation in transcript_lower for variation in variations):
            intent = final_intent
            break

    # Check for objects
    for common_object in COMMON_OBJECTS:
        if common_object in transcript_lower:
            obj = common_object
            break

    return {"intent": intent, "object": obj}