# CARLA_experiments

This repo documents hands-on experiments with CARLA to better understand how simulation environments are configured and controlled. I'm particularly interested in areas where visual realism breaks down, creating the sim-to-real gap for training and validation.

My approach is based on 
- [CARLA 0.9.16 docs](https://carla.readthedocs.io/en/0.9.16/)
- Insights and advice from Carla users and industry experts (thank you!)
- ChatGPT 5.2 and Claude Opus 4.6 for planning and coding

## 01. CARLA on Runpod
Stood up CARLA headless on RunPod (RTX 4090), separated engine from experiment workspace, validated reproducibility. Added setup and installation scripts for easy teardown and rebuild of a pod.

→ [Bootstrapping notes](notes/01.%20runpod%20bootstrapping.md)


## 02. Sensors & Renders
Explored RGB + depth renders, multi-camera setups, sync vs async modes, and capture pipelines.

→ [Renders and notes](notes/02.%20sensor%20tests.md)

## 03. World variations

Systematically varied environment (sun position, cloudiness, precipitation, wetness) and actors (oncoming traffic, pedestrian on crosswalk) to review realism.

![Precipitation](data/illustrations/precipitation.gif)

![Walker](data/illustrations/walker.gif)

→ [Variations and notes](notes/03.%20worldbuilding.md)

## Route-following (BasicAgent)

Drew a map with aerial view + directional lanes overlaid. Copied the BasicAgent and rendered a 45s drive.

→ [Closed-loop drive notes](notes/04.%20closed%20loop%20drive.md)
