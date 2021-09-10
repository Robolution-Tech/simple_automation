# simple_automation

Strcture: \
── simple_automation: Metapackage folder \
│   ├── robo_base: Execute commands to control the vehicle. Sub: /cmd_vel \
│   ├── robo_bringup: Startup scripts/roslaunch files to bring up all components \
│   ├── robo_localization: SLAM/GPS etc. Provides localization, also updates maps \
│   ├── robo_detection: Detects humans/target objects etc \
│   ├── robo_planning: Path planning node \
│   ├── robo_decision: Final decision tree to output final commands \


TODO:
- [ ] Set up the simple testing environment (bringup)
- [ ] Base node
- [ ] Localization node
- [ ] planning node
- [ ] decision node
- [ ] detection ndoe
- [ ] Field test!!
