# Nimbus Development Guide

## Overview
This guide covers local development workflow using Docker only for external dependencies (Ollama + ROS2). Currently focused on **`nimbus-ai/`** development.

## Development Architecture
```
Local Development Environment:
├── nimbus-ai/ (Local Python)     # Your main work area
├── Docker: nimbus-ollama         # External LLM service  
└── Docker: ros2-central          # External SLAM service
```

**Benefits:**
- **Fast iteration**: No container rebuilds for code changes
- **Easy debugging**: Direct Python debugging with breakpoints  
- **Hot reloading**: Automatic code reloading
- **Full IDE support**: IntelliSense, debugging, etc.

## Quick Start

### 1. Start External Dependencies
```bash
cd Nimbus

# Start only Ollama + ROS2 containers
docker-compose -f docker-compose.dev.yml up -d

# Verify both containers are running
docker-compose -f docker-compose.dev.yml ps
```

### 2. Verify Ollama Models
Your existing models are automatically available:
```bash
# Check available models
docker exec -it nimbus-ollama ollama list

# Should show llama3:latest and other models
```

### 3. Set Up Local Environment
```bash
# Navigate to your work directory
cd nimbus-ai

# Install dependencies
pip install -r requirements.txt

# Test basic imports
python -c "from AI import get_ai_system; print('AI system imported successfully')"
```

## Development Workflow

### Daily Development
```bash
# 1. Start containers (if not running)
docker-compose -f docker-compose.dev.yml up -d

# 2. Work in nimbus-ai/ directory
cd nimbus-ai

# 3. Edit your code with any IDE/editor
# 4. Test changes immediately
python classes/Tests/interactive_intent_test.py

# 5. Run specific tests
python classes/Tests/test_intent_object_node.py
```

### Testing Your Changes
```bash
# Test intent extraction
cd nimbus-ai/classes/Tests
python interactive_intent_test.py

# Test with different inputs
python -c "from classes.intent_object_node import intent_object_node; print(intent_object_node('navigate to the kitchen'))"

# Test AI system integration
python -c "from AI import get_ai_system; ai = get_ai_system(); print(ai.health_check())"
```

### ROS2 Development
```bash
# Enter ROS2 container for development
docker exec -it ros2-central bash

# Inside container:
source /opt/ros/humble/setup.bash
ros2 pkg list | grep rtabmap

# Launch RTAB-Map (when ready)
ros2 launch rtabmap_ros rtabmap.launch.py
```

## File Structure

### Your Work Area (nimbus-ai/)
```
nimbus-ai/
├── AI.py                   # Main AI system class
├── classes/
│   ├── intent_object_node.py    # Your intent extraction
│   ├── rtabmap_node.py          # ROS2 SLAM integration
│   ├── object_detect_node.py    # Object detection
│   ├── depth_node.py            # Depth estimation
│   ├── stt_node.py              # Speech-to-text
│   ├── survey_node.py           # Environment survey
│   └── Tests/
│       ├── test_intent_object_node.py
│       ├── interactive_intent_test.py
│       └── ...
├── requirements.txt
└── Dockerfile.ai           # For production deployment
```

### External Dependencies (Containers)
```
Docker Containers:
├── nimbus-ollama (localhost:11434)     # LLM inference
└── ros2-central (localhost:11311)     # SLAM/Navigation
```

## Service Communication

### Ollama Integration
Your AI code connects directly to the containerized Ollama:

```python
# In classes/intent_object_node.py
OLLAMA_URL = "http://localhost:11434/api/generate"  # Points to container

# Usage
import requests
response = requests.post(OLLAMA_URL, json=payload)
```

### ROS2 Integration  
ROS2 communication happens through the container:

```python
# Future ROS2 integration in rtabmap_node.py
# Will communicate with ros2-central container
```

## Debugging

### Python Debugging
```bash
# Set breakpoints in your IDE
# Or use pdb
python -c "import pdb; pdb.set_trace(); from classes.intent_object_node import intent_object_node"
```

### Container Debugging
```bash
# Check Ollama logs
docker logs nimbus-ollama

# Check ROS2 logs  
docker logs ros2-central

# Enter containers for debugging
docker exec -it nimbus-ollama bash
docker exec -it ros2-central bash
```

### Network Testing
```bash
# Test Ollama connectivity
curl -X POST http://localhost:11434/api/generate \
  -H "Content-Type: application/json" \
  -d '{"model": "llama3:latest", "prompt": "Hello", "stream": false}'

# Test from Python
python -c "import requests; print(requests.get('http://localhost:11434/api/tags').json())"
```

## Common Development Tasks

### Adding New AI Nodes
1. Create new file in `nimbus-ai/classes/`
2. Follow existing pattern (see `intent_object_node.py`)
3. Add import to `AI.py` if needed
4. Create test file in `classes/Tests/`

### Modifying Intent Extraction
1. Edit `nimbus-ai/classes/intent_object_node.py`
2. Test immediately: `python classes/Tests/interactive_intent_test.py`
3. No container restart needed!

### Adding Dependencies
1. Add to `nimbus-ai/requirements.txt`
2. Install locally: `pip install -r requirements.txt`
3. Test your code
4. When ready for production, it'll be included in container build

## Performance Tips

### Fast Development Cycle
1. **No rebuilds**: Change code → test immediately
2. **Hot reloading**: Use Flask debug mode for web services
3. **Quick tests**: Run specific test files instead of full suite
4. **IDE integration**: Full debugging and IntelliSense support

### Container Management
```bash
# Start containers in background
docker-compose -f docker-compose.dev.yml up -d

# Stop containers when done
docker-compose -f docker-compose.dev.yml down

# View container status
docker-compose -f docker-compose.dev.yml ps

# Restart if needed
docker-compose -f docker-compose.dev.yml restart ollama
```

## Troubleshooting

### Common Issues

**1. Import errors**
```bash
# Make sure you're in nimbus-ai directory
cd nimbus-ai
python -c "from classes.intent_object_node import intent_object_node"
```

**2. Ollama connection failed**
```bash
# Check container is running
docker ps | grep ollama

# Test connection
curl http://localhost:11434/api/tags
```

**3. ROS2 container exits**
```bash
# Check logs
docker logs ros2-central

# Container should stay alive with tail -f /dev/null
```

**4. Port conflicts**
```bash
# Check what's using ports
netstat -tulpn | grep :11434
netstat -tulpn | grep :11311
```

### Reset Development Environment
```bash
# Stop all containers
docker-compose -f docker-compose.dev.yml down

# Start fresh
docker-compose -f docker-compose.dev.yml up -d

# Verify
docker-compose -f docker-compose.dev.yml ps
```

## Production Deployment

When your code is ready, deploy to production:

```bash
# Stop development containers
docker-compose -f docker-compose.dev.yml down

# Deploy full stack
docker-compose build
docker-compose up -d

# Your code runs in nimbus-ai container
docker logs nimbus-ai
```

## Best Practices

### Code Organization
- Keep AI logic in `nimbus-ai/classes/`
- Each node should be self-contained
- Use proper error handling and logging
- Write tests for each new feature

### Testing Strategy
- Test individual nodes in isolation
- Use interactive tests for experimentation  
- Test integration with containerized services
- Validate against expected input/output formats

### Git Workflow
- Work on feature branches
- Test locally before committing
- Include both local and container tests
- Document any new dependencies

## Next Steps

1. **Start Development**: Use the Quick Start section
2. **Explore AI.py**: Understand the central coordinator
3. **Modify Nodes**: Start with `intent_object_node.py`
4. **Add Features**: Create new nodes as needed
5. **Test Integration**: Verify with containerized services

Happy coding! 🚀