📝 NLP Project – Technical Lead Report
**Name:** Erin David Cullen
**Date:** ____________________
**Date:** ____________________

✅ 1. Work Completed (Since Last Report)
_What you’ve built, implemented, tested or reviewed and an estimate as to how many hours it took to complete
•	Research on available drones and project limitations (~8 hr)
•	Team discussions and project planning (~2 hr)
•	Containerization architecture research (~6 hr)
•	File structure setup and GitHub organization (~2 hr)

• **Feature/Module:**
  - Conducted comprehensive research on drone hardware capabilities and limitations that will impact project scope. Investigated containerization architecture with ROS2 as the central container coordinating all system components.

• **Tools/Libs Used:**
  - Docker documentation
  - ROS2 Humble docker container
  - Drone manufacturer IDE

• **Outcome/Results:**
  - Clear understanding of hardware constraints and system architecture requirements. Identified Docker-based workflow with ROS2 container as central hub.

• **Contributions (if applicable):**
  - Established project file structure
  - Set up GitHub repository and team collaboration workflow 
 


📌 2. Current Tasks in Progress
_What you’re actively working on. Include blockers if any._

| Task | Description | ETA | Blockers |
|------|-------------|-----|----------|
| Team onboarding | Getting all team members familiar with GitHub and project structure | ~ 3 hours | WSL, computer configs |
| Architecture planning | Finalizing containerized system design | ~ 1 hour | None |

📅 3. Upcoming Tasks
_Planned work for the next sprint or phase._

• **Task:** AI Workflow Design, Build and Integration
  - **Purpose/Goal:**
    - Set up Docker containers with ROS2 as central coordination hub
  - **Dependencies:**
    - Finalized requirements from drone testing

• **Task:** Video and Audio Feed Integration
  - **Purpose/Goal:**
    - Get video and audio feeds to run through ROS2 Container
  - **Dependencies:**
    - Docker containerization setup completed

• **Task:** AI Processing Modules Development
  - **Purpose/Goal:**
    - Build and integrate AI processing modules (object detection, speech-to-text, intent processing, depth analysis, RTAB-Map SLAM)
  - **Dependencies:**
    - ROS2 container setup and video/audio feed integration 

🚨 4. Issues & Risks
_Bugs, technical debt, resourcing, or anything threatening progress._

| Issue | Impact | Suggested Action | Owner |
|-------|--------|------------------|-------|
| Drone access delay | Blocks hands-on validation of capabilities | Coordinate with equipment procurement | Team |
| System-wide integration complexity | May cause delays and compatibility issues | Design modular architecture with clear interfaces | Ed |
| Low latency video inputs | Video feed unreliability affects real-time processing | Implement buffering and error handling strategies | Team |
| ROS2 container isolation | Difficult integration with Python-based project components | Design ROS2 bridge/API for Python communication | Ed |
| Live video processing challenges | Object detection on continuous feed vs single images | Optimize processing pipeline and implement frame skipping | Team |

📈 5. Key Insights / Recommendations
_Lessons learned, suggestions, architecture notes, or optimizations._

•	Docker containerization with ROS2 as central hub appears to be the optimal architecture for system coordination
•	Early identification of drone limitations will be crucial for realistic project scoping 
•	Team GitHub workflow established successfully, enabling collaborative development

