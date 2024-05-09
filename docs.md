


## Zadanie 3

The `mode.sh` command is used to set the mode of a system. The command takes a single argument, `X`, which represents the desired mode.

- OFF: 0
- MANUAL: 1
- AUTO: 2

```bash
bash mode.sh 0
```

```bash
bash mode.sh 1
```

```bash
bash mode.sh 2
```

### Startup



```bash
ros2 run control_package main
```

```bash
ros2 run control_package auto
```

```bash
ros2 run control_package manual
```



```bash
ros2 launch control_package robot_launch.py
```