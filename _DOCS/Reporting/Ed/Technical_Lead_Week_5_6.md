📝 NLP Project – Technical Lead Report
**Name:** Erin David Cullen
**Date:** Week 5-6 (Monday 18 August - Sunday 31 August)

✅ 1. Work Completed (Since Last Report)
_What you've built, implemented, tested or reviewed and an estimate as to how many hours it took to complete_
• ROS2 implementation improvements (~6 hr)
• Standard topics publishers and subscribers (~4 hr)
• RTAB-Map and SLAM research (~4 hr)
• Project scope evaluation and architectural discussions (~3 hr)

• **Feature/Module:** ROS2 Implementation Improvements
  - Enhanced ROS2 container communication architecture for better stability
  - Improved data flow between Flask application and ROS2 nodes
  - Optimized message passing and reduced communication latency
  - Strengthened error handling and recovery mechanisms in ROS2 integration

• **Feature/Module:** Standard Topics Publishers and Subscribers
  - Implemented standardized ROS2 topic structure for video/audio data streams
  - Created reliable publisher nodes for video feed and audio capture
  - Established subscriber nodes for command processing and status updates
  - Ensured consistent message formats across all communication channels

• **Feature/Module:** RTAB-Map and SLAM Research
  - Conducted comprehensive research into RTAB-Map SLAM implementation
  - Analyzed integration requirements and computational overhead
  - Evaluated feasibility of real-time SLAM within current system constraints
  - Assessed benefits and limitations of SLAM for drone localization

• **Feature/Module:** Project Scope Evaluation
  - Discussed potential reduction in project scope regarding ROS2 elimination
  - Analyzed implications of removing SLAM functionality from system requirements
  - Evaluated trade-offs between system complexity and functional requirements
  - Assessed impact on drone localization accuracy without SLAM

• **Tools/Libs Used:**
  - ROS2 framework for improved node communication
  - RTAB-Map documentation and examples for SLAM research
  - Standard ROS2 message types for video/audio streaming
  - Docker containers for ROS2 environment management

• **Outcome/Results:**
  - Successfully improved ROS2 communication stability and performance
  - Established robust publisher/subscriber architecture using standard topics
  - Gained comprehensive understanding of RTAB-Map SLAM implementation requirements
  - Identified critical issues with drone localization if SLAM is eliminated

• **Contributions (if applicable):**
  - Enhanced ROS2 integration with improved error handling
  - Implemented standardized topic structure for better maintainability
  - Conducted thorough research into SLAM integration possibilities
  - Led architectural discussions regarding project scope and complexity trade-offs

📌 2. Current Tasks in Progress
_What you're actively working on. Include blockers if any._

| Task | Description | ETA | Blockers |
|------|-------------|-----|----------|
| YOLOv11 fine-tuning | Optimizing YOLOv11 model parameters for drone-specific detection scenarios | 4 hours | Model compatibility testing |
| Depth accuracy validation | Testing Structure From Motion accuracy across different scenarios | 6 hours | Need diverse test environments, camera stabilization for smooth motion |
| Threading optimization | Fine-tuning concurrent processing efficiency and synchronization | 4 hours | Thread synchronization complexity |
| Project scope finalization | Determining final architecture with or without ROS2/SLAM components | 2 hours | Team consensus on scope reduction |

📅 3. Upcoming Tasks
_Planned work for the next sprint or phase._

• **Task:** Voice Pipeline Completion
  - **Purpose/Goal:**
    - Complete voice activation detection and command processing pipeline
    - Implement microphone live stream with "ok drone" activation phrase detection
    - Integrate speech-to-text (Whisper) and intent processing (Ollama LLM) nodes
    - Establish command status flow ("none" → "new" → "processing" → "complete")
  - **Dependencies:**
    - Audio capture system setup
    - STT and intent processing node integration
    - Project scope finalization

• **Task:** Drone Localization Solution (if SLAM eliminated)
  - **Purpose/Goal:**
    - Develop alternative localization method without SLAM
    - Implement error correction for accumulated position drift
    - Establish reliable home position tracking system
    - Minimize localization errors during multiple movement sequences
  - **Dependencies:**
    - Final decision on SLAM elimination
    - Alternative positioning algorithm research

• **Task:** WebUI Live Video Integration
  - **Purpose/Goal:**
    - Integrate live video feed display into web control interface
    - Add AI overlay rendering to web-based video stream
    - Implement real-time video streaming via WebSocket to browser
  - **Dependencies:**
    - Stable video processing pipeline
    - WebSocket video streaming implementation

• **Task:** System Architecture Finalization
  - **Purpose/Goal:**
    - Finalize system architecture based on scope decisions
    - Update documentation to reflect architectural changes
    - Ensure all components align with final scope requirements
  - **Dependencies:**
    - Project scope finalization
    - Team agreement on final requirements

🚨 4. Issues & Risks
_Bugs, technical debt, resourcing, or anything threatening progress._

| Issue | Impact | Suggested Action | Owner |
|-------|--------|------------------|-------|
| Video processing latency | Delays between live video and AI processing overlays affect real-time performance | Optimize frame buffering and reduce processing overhead | Ed |
| Inconsistent FPS performance | Variable frame rates affect smooth video playback and user experience | Implement adaptive frame rate control and resource management | Team |
| Video quality degradation | Processing pipeline reduces video quality affecting detection accuracy | Optimize compression and maintain higher quality throughout pipeline | Team |
| Inaccurate depth detection | Structure From Motion calculations significantly off from actual distances | Recalibrate depth algorithms and improve feature matching accuracy | Ed |
| Drone localization without SLAM | Eliminating SLAM creates significant challenges for accurate drone positioning relative to home | Research alternative localization methods or maintain minimal SLAM implementation | Team |
| Error propagation in positioning | Multiple drone movements without SLAM lead to accumulated positioning errors | Implement error correction algorithms and position validation systems | Ed |

📈 5. Key Insights / Recommendations
_Lessons learned, suggestions, architecture notes, or optimizations._

• ROS2 standardization significantly improves system reliability and maintainability through consistent topic structures
• RTAB-Map SLAM integration would provide substantial localization benefits but increases system complexity considerably
• Eliminating ROS2 may reduce complexity but sacrifices standardization and potential future scalability
• Drone localization without SLAM presents significant challenges, particularly for maintaining accurate home position after multiple movements
• Error propagation in positioning becomes critical issue without SLAM - alternative correction mechanisms essential
• Project scope reduction discussions highlight need for clear priority definition between system simplicity and functional capability