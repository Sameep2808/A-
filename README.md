# A*
## Badges
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
## Authors
- [Sameep Pote](https://github.com/Sameep2808) - Graduate student at University of Maryland pursuing M.Eng. Robotics.Loves to watch animes.

URL: https://github.com/Sameep2808/dijkstra.git

## License
```
MIT License

Copyright (c) 2022 Sameep Pote

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```


## Output

### Inputs

Clearance, goal position, starting position 
agent_input.yaml

### Visualization
![g1](https://github.com/Sameep2808/A-/blob/main/Videos/g1.gif)
### Path
![p1](https://github.com/Sameep2808/A-/blob/main/Videos/p1)

## Dependencies and Technologies used

- Programming language : Python
- Operating System : Ubuntu 20.04
- Packages : OpenCV, numpy, matplotlib, yaml

## Run

1. Download the repository in src folder of catkin workspace
```
git clone https://github.com/Sameep2808/A-
```

2. Enter robot clearance and radius information and **_ALL_** agent(s) starting and goal positions into agent_input.yaml:

**Robot information:**
```
robot:
  clearance: 5
  radius: 2
  movement_length: 2
```

**Example of agent information formatting:**
```
- agent_name: agent1
  start: [0,0]
  goal: [240,240]
  start_angle: 30
  goal_angle: 30
```

3. Run the python file calling agent_input.yaml file
```
python3 Sameep_Pote.py agent_input.yaml
```
4. Wait For the Output

