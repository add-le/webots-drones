
# Webots Drones

[![GPLv3 License](https://img.shields.io/badge/License-GPL%20v3-yellow.svg)](https://github.com/add-le/webots-drones/blob/master/LICENCE)
[![Language Java](https://img.shields.io/badge/Language-Java-blue.svg)](https://www.java.com/)
[![Engine Webots](https://img.shields.io/badge/Engine-Webots-red.svg)](https://cyberbotics.com/)


Drone swarm evolving in an unknown environment.


## Features

- Three levels of autonomies
- Evolution in an unknown environment
- Behavior sharing in the swarm
- Use of sensors


## Installation

1. Download Webots

`https://cyberbotics.com/`

2. Download the project
```bash
gh repo clone add-le/webots-drones
git checkout master
```

3. Run the appropriate autonomy java file (2 or 3) in Webots software.
    
## Usage

- Autonomy 2 : Will find the exist on a maze using small randomness and its sensors, use a machine state to create better ways and possibilities
- Autonomy 3 : Can interact with other drone in the swarm to find an objective in an unknown environment, approach like SLAM algorithm.

Use increase time of Webots software to find the answer in less time.
## Authors

- [@add-le](https://github.com/add-le)
- [@wih](#)

## License

[GPL v3](https://github.com/add-le/webots-drones/blob/master/LICENCE)

