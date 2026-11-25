# Episode summary: results/d2/ablation_no_obstacle

```
date: 2026-11-25T17:51:18+00:00
software: d1-submission-4-g63c8bf9
ros_distro: jazzy  ROS_DOMAIN_ID: 37
host: Linux alf-Ubuntu-24-04-PC-Q35-ICH9-2009 7.0.0-42-generic #42~24.04.1-Ubuntu SMP PREEMPT_DYNAMIC Mon Oct 19 14:43:01 UTC 2 x86_64 x86_64 x86_64 GNU/Linux
cpu:  AMD Ryzen 5 7600X3D 6-Core Processor
cpus: 4  mem: 7.8Gi
visualization: off (rviz:=false visualize:=false)
scenarios: 5  seeds: 1 2 3  extra: use_obstacle_check:=false  label: ablation: no LiDAR corridor check
```

| run | result | violations | sim time [s] | min clearance [m] | max speed [m/s] | stopped [s] | rejected cmds | entry t [s] | GO latched | mean/max \|cte\| [m] | scenario measurements | transitions |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|---:|
| s5_seed1 | FAIL | collision | 16.45 | -0.059 | 3.500 | 0.55 | 6 | 7.10 | t=6.40 x=21.65 | 0.058 / 0.231 | detect +- s, brake +- s, stop x=-, obst clr -0.06 m | 5 |
| s5_seed2 | FAIL | collision | 16.50 | -0.059 | 3.500 | 0.60 | 10 | 7.15 | t=6.45 x=21.65 | 0.059 / 0.231 | detect +- s, brake +- s, stop x=-, obst clr -0.06 m | 7 |
| s5_seed3 | FAIL | collision | 16.45 | -0.059 | 3.500 | 0.55 | 1 | 7.10 | t=6.40 x=21.65 | 0.060 / 0.232 | detect +- s, brake +- s, stop x=-, obst clr -0.06 m | 5 |

**0/3 runs pass.** min clearance is the evaluator's disc-to-disc minimum over all objects; rejected_commands includes startup ticks before the first valid command.
