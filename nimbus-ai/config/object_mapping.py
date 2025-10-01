"""
Object Mapping Configuration
Maps YOLO detection class names to Webots world object names
"""

# YOLO class name -> Webots object key in WORLD_OBJECTS
YOLO_TO_WEBOTS = {
    # Vehicles
    'car': 'car',
    'truck': 'car',
    'bus': 'car',

    # People
    'person': 'human',

    # Furniture
    'bench': 'bench',
    'chair': 'bench',
    'couch': 'bench',

    # Objects
    'bottle': 'cardboard_box',
    'box': 'cardboard_box',
    'suitcase': 'cardboard_box',
    'backpack': 'cardboard_box',

    # Buildings/Structures
    'cabinet': 'cabinet',
    'desk': 'cabinet',
    'shelf': 'cabinet',

    # Playground
    'slide': 'slide',

    # Infrastructure
    'manhole': 'manhole'
}

# Webots object type -> Generic category
WEBOTS_TYPE_TO_CATEGORY = {
    'TeslaModel3Simple': 'car',
    'Pedestrian': 'person',
    'Bench': 'bench',
    'CardboardBox': 'box',
    'Cabinet': 'cabinet',
    'Slide': 'slide',
    'SquareManhole': 'manhole'
}

def get_webots_object_name(yolo_class_name):
    """
    Convert YOLO class name to Webots object key

    Args:
        yolo_class_name: YOLO detection class (e.g., 'car', 'person')

    Returns:
        Webots object key or None if not mapped
    """
    return YOLO_TO_WEBOTS.get(yolo_class_name.lower())

def get_generic_category(webots_type):
    """
    Convert Webots object type to generic category

    Args:
        webots_type: Webots object type (e.g., 'TeslaModel3Simple')

    Returns:
        Generic category name or the input if not mapped
    """
    return WEBOTS_TYPE_TO_CATEGORY.get(webots_type, webots_type.lower())
