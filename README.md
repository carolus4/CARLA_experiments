# CARLA_experiments

This repo walks through exploration & experimentation in CARLA. 

My goal is to understand users and workflows for SIM in Autonomous Driving.

My approach is based on 
- [CARLA 0.9.16 docs](https://carla.readthedocs.io/en/0.9.16/)
- Insights and advice from users and industry experts (thank you!)
- ChatGPT 5.2 and Claude Opus 4.6 for planning and coding

## 01. Run carla on Runpod
Stood up a full CARLA simulation stack on RunPod using an RTX 4090, separated the engine (/opt/carla) from our experiments (/workspace/CARLA_experiments), and got the binary running headless in a containerized cloud environment.

→ [Walkthrough](notes/01.%20runpod%20bootstrapping.md)


## 02. Render Capture, 
Tested rgb and depth renders out-of-the-box.
Tested editing the camera setup on the test vehicle.

→ [Renders and notes](notes/02.%20sensor%20tests.md)
