# Episode summary: results/d2/var_traffic_2

```
date: 2026-11-25T17:37:00+00:00
software: d1-submission-4-g63c8bf9
ros_distro: jazzy  ROS_DOMAIN_ID: 37
host: Linux alf-Ubuntu-24-04-PC-Q35-ICH9-2009 7.0.0-42-generic #42~24.04.1-Ubuntu SMP PREEMPT_DYNAMIC Mon Oct 19 14:43:01 UTC 2 x86_64 x86_64 x86_64 GNU/Linux
cpu:  AMD Ryzen 5 7600X3D 6-Core Processor
cpus: 4  mem: 7.8Gi
visualization: off (rviz:=false visualize:=false)
scenarios: 3  seeds: 1 2 3  extra: traffic_speed:=2.0  label: traffic variation
```

| run | result | violations | sim time [s] | min clearance [m] | max speed [m/s] | stopped [s] | rejected cmds | entry t [s] | GO latched | mean/max \|cte\| [m] | scenario measurements | transitions |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|---:|
| s3_seed1 | PASS | none | 46.65 | 1.168 | 3.500 | 19.15 | 11 | 28.45 | t=28.15 x=23.69 | 0.061 / 0.295 | yield 26.3 s, gap safe t=27.3, car clr 1.68 m | 8 |
| s3_seed2 | PASS | none | 46.65 | 1.168 | 3.500 | 19.15 | 9 | 28.45 | t=28.15 x=23.69 | 0.062 / 0.293 | yield 26.3 s, gap safe t=27.3, car clr 1.68 m | 8 |
| s3_seed3 | PASS | none | 46.60 | 1.168 | 3.500 | 19.10 | 2 | 28.40 | t=28.10 x=23.69 | 0.062 / 0.294 | yield 26.7 s, gap safe t=27.2, car clr 1.68 m | 8 |

**3/3 runs pass.** min clearance is the evaluator's disc-to-disc minimum over all objects; rejected_commands includes startup ticks before the first valid command.
