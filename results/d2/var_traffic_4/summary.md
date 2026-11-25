# Episode summary: results/d2/var_traffic_4

```
date: 2026-11-25T17:39:34+00:00
software: d1-submission-4-g63c8bf9
ros_distro: jazzy  ROS_DOMAIN_ID: 37
host: Linux alf-Ubuntu-24-04-PC-Q35-ICH9-2009 7.0.0-42-generic #42~24.04.1-Ubuntu SMP PREEMPT_DYNAMIC Mon Oct 19 14:43:01 UTC 2 x86_64 x86_64 x86_64 GNU/Linux
cpu:  AMD Ryzen 5 7600X3D 6-Core Processor
cpus: 4  mem: 7.8Gi
visualization: off (rviz:=false visualize:=false)
scenarios: 3  seeds: 1 2 3  extra: traffic_speed:=4.0  label: traffic variation
```

| run | result | violations | sim time [s] | min clearance [m] | max speed [m/s] | stopped [s] | rejected cmds | entry t [s] | GO latched | mean/max \|cte\| [m] | scenario measurements | transitions |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|---:|
| s3_seed1 | PASS | none | 33.00 | 1.168 | 3.500 | 5.50 | 9 | 14.80 | t=14.50 x=23.69 | 0.061 / 0.295 | yield 12.9 s, gap safe t=13.7, car clr 1.69 m | 9 |
| s3_seed2 | PASS | none | 33.00 | 1.169 | 3.500 | 5.50 | 12 | 14.80 | t=14.50 x=23.69 | 0.061 / 0.294 | yield 12.9 s, gap safe t=13.7, car clr 1.69 m | 8 |
| s3_seed3 | PASS | none | 33.00 | 1.168 | 3.500 | 5.50 | 4 | 14.80 | t=14.50 x=23.69 | 0.062 / 0.294 | yield 13.1 s, gap safe t=13.7, car clr 1.69 m | 8 |

**3/3 runs pass.** min clearance is the evaluator's disc-to-disc minimum over all objects; rejected_commands includes startup ticks before the first valid command.
