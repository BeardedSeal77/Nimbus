---
output:
  word_document: default
  html_document: default
---
📝 NLP Project – Technical Lead Report
**Name:** Erin David Cullen
**Date:** Week 11-12 (Monday 29 September - Sunday 12 October)

✅ 1. Work Completed (Since Last Report)
_What you've built, implemented, tested or reviewed and an estimate as to how many hours it took to complete_

• Project video production and documentation (~4 hr)

• Webots simulation environment implementation (~8 hr)

• Python hub architecture development (~6 hr)

• **Feature/Module:** Drone Simulation Migration
  - Implemented drone simulation using Webots platform
  - Migrated project architecture away from ROS2 framework
  - Developed Python hub running with WebSocket communication
  - Integrated Webots simulation with Python hub backend
  - Connected live video feed from simulation to AI processing pipeline

• **Feature/Module:** Stereo Triangulation Depth Detection
  - Implemented stereo (nearest neighbour) triangulation system
  - Developed dual-camera pose beam projection algorithm
  - Created 2-point line calculation for beam intersection
  - Implemented minimum distance calculation for object centre determination
  - Integrated depth detection into existing AI pipeline

• **Feature/Module:** AI Pipeline Completion
  - Finalized integration of all AI processing components
  - Completed end-to-end automated detection and depth estimation
  - Validated full pipeline functionality in simulation environment

• **Tools/Libs Used:**
  - Webots simulation platform
  - Python WebSocket libraries
  - Computer vision libraries for stereo triangulation
  - Video streaming integration tools

• **Outcome/Results:**
  - Successfully migrated from ROS2 to Python-based architecture
  - Achieved functional drone simulation with live video processing
  - Completed stereo depth detection system implementation
  - Fully operational AI pipeline from video input to depth estimation
  - Project demonstration video completed

• **Contributions (if applicable):**
  - Led simulation platform migration and implementation
  - Designed and developed stereo triangulation depth system
  - Orchestrated AI pipeline integration and completion
  - Contributed to project documentation and video production

📌 2. Current Tasks in Progress
_What you're actively working on. Include blockers if any._

| Task | Description | ETA | Blockers |
|------|-------------|-----|----------|
| None | The AI pipeline is complete |  |  |

📅 3. Upcoming Tasks
_Planned work for the next sprint or phase._

• **Task:** Performance Testing and Validation
  - **Purpose/Goal:**
    - Test AI pipeline performance across diverse scenarios
    - Validate depth detection accuracy metrics
    - Monitor system performance under various conditions
  - **Dependencies:**
    - Completed AI pipeline implementation
    - Webots simulation environment
    - Test scenario definitions

• **Task:** Final Documentation and Reporting
  - **Purpose/Goal:**
    - Document complete technical implementation
    - Record system architecture and design decisions
    - Prepare final project deliverables
  - **Dependencies:**
    - Completed testing and validation
    - Performance metrics collection
    - Project video completion

🚨 4. Issues & Risks
_Bugs, technical debt, resourcing, or anything threatening progress._

| Issue | Impact | Suggested Action | Owner |
|-------|--------|------------------|-------|
| Simulation realism | Webots environment may not perfectly represent physical conditions | Document limitations in proof-of-concept scope | Team |

📈 5. Key Insights / Recommendations
_Lessons learned, suggestions, architecture notes, or optimizations._

• Webots provides more flexible simulation environment compared to Gazebo for this specific use case

• WebSocket-based architecture successfully resolved latency and bandwidth issues experienced with ROS2 framework

• Python hub design simplified integration and improved real-time communication performance

• Stereo triangulation using nearest neighbour approach provides effective depth estimation without complex sensor requirements

• Direct video stream integration enables real-time AI processing capabilities in simulation environment

• Completing AI pipeline milestone demonstrates project viability and technical implementation success

• Migration away from ROS2 reduced complexity, eliminated performance bottlenecks, and accelerated development progress
