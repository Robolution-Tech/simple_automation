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
- [X] Set up the simple testing environment (bringup)
- [X] Base node
- [X] Localization node
- [X] planning node
- [X] decision node
- [X] detection ndoe
- [ ] Field test!!




[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/t_TQEVIVbMo/0.jpg)](https://youtu.be/t_TQEVIVbMo)
