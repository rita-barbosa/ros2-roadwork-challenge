# Episode summary: results/d2/default

```
date: 2026-11-25T17:10:20+00:00
software: d1-submission-4-g63c8bf9
ros_distro: jazzy  ROS_DOMAIN_ID: 37
host: Linux alf-Ubuntu-24-04-PC-Q35-ICH9-2009 7.0.0-42-generic #42~24.04.1-Ubuntu SMP PREEMPT_DYNAMIC Mon Oct 19 14:43:01 UTC 2 x86_64 x86_64 x86_64 GNU/Linux
cpu:  AMD Ryzen 5 7600X3D 6-Core Processor
cpus: 4  mem: 7.8Gi
visualization: off (rviz:=false visualize:=false)
scenarios: 1 2 3 4 5  seeds: 1 2 3  extra: none  label: default parameters
```

| run | result | violations | sim time [s] | min clearance [m] | max speed [m/s] | stopped [s] | rejected cmds | entry t [s] | GO latched | mean/max \|cte\| [m] | scenario measurements | transitions |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|---:|
| s1_seed1 | PASS | none | 25.05 | 1.166 | 3.500 | 0.60 | 1 | 7.15 | t=6.45 x=21.65 | 0.068 / 0.293 | unexplained stops 0 | 7 |
| s1_seed2 | PASS | none | 25.00 | 1.167 | 3.500 | 0.55 | 1 | 7.10 | t=6.40 x=21.65 | 0.068 / 0.293 | unexplained stops 0 | 7 |
| s1_seed3 | PASS | none | 25.05 | 1.167 | 3.500 | 0.60 | 4 | 7.15 | t=6.45 x=21.65 | 0.068 / 0.290 | unexplained stops 0 | 7 |
| s2_seed1 | PASS | none | 29.55 | 1.168 | 3.500 | 2.05 | 1 | 11.35 | t=11.05 x=23.69 | 0.086 / 0.294 | GO +0.20 s, resume +0.20 s | 8 |
| s2_seed2 | PASS | none | 29.55 | 1.168 | 3.500 | 2.05 | 3 | 11.35 | t=11.05 x=23.69 | 0.086 / 0.295 | GO +0.20 s, resume +0.20 s | 8 |
| s2_seed3 | PASS | none | 29.60 | 1.168 | 3.500 | 2.10 | 2 | 11.40 | t=11.10 x=23.69 | 0.085 / 0.294 | GO +0.25 s, resume +0.25 s | 8 |
| s3_seed1 | PASS | none | 37.55 | 1.168 | 3.500 | 10.05 | 6 | 19.35 | t=19.05 x=23.69 | 0.060 / 0.294 | yield 17.7 s, gap safe t=18.2, car clr 1.69 m | 8 |
| s3_seed2 | PASS | none | 37.55 | 1.168 | 3.500 | 10.05 | 1 | 19.35 | t=19.05 x=23.69 | 0.061 / 0.295 | yield 17.6 s, gap safe t=18.2, car clr 1.69 m | 8 |
| s3_seed3 | PASS | none | 37.55 | 1.168 | 3.500 | 10.05 | 2 | 19.35 | t=19.05 x=23.69 | 0.063 / 0.295 | yield 16.5 s, gap safe t=18.2, car clr 1.69 m | 10 |
| s4_seed1 | PASS | none | 27.15 | 1.167 | 3.500 | 1.05 | 11 | 9.15 | t=8.75 x=23.33 | 0.082 / 0.294 | beacon stale t=2.70, GO +0.20 s, resume +0.20 s | 9 |
| s4_seed2 | PASS | none | 27.15 | 1.167 | 3.500 | 1.05 | 11 | 9.15 | t=8.75 x=23.33 | 0.083 / 0.294 | beacon stale t=2.70, GO +0.20 s, resume +0.20 s | 9 |
| s4_seed3 | PASS | none | 27.25 | 1.167 | 3.500 | 0.60 | 6 | 9.15 | t=8.85 x=23.59 | 0.080 / 0.294 | beacon stale t=2.70, GO +0.20 s, resume +0.20 s | 9 |
| s5_seed1 | PASS | none | 32.45 | 1.167 | 3.500 | 6.00 | 2 | 7.60 | t=6.90 x=21.65 | 0.072 / 0.296 | detect +0.00 s, brake +1.50 s, stop x=53.55, obst clr 1.97 m | 8 |
| s5_seed2 | PASS | none | 32.40 | 1.167 | 3.500 | 5.95 | 9 | 7.60 | t=6.90 x=21.65 | 0.072 / 0.295 | detect +0.00 s, brake +1.50 s, stop x=53.55, obst clr 1.97 m | 8 |
| s5_seed3 | PASS | none | 31.95 | 1.167 | 3.500 | 5.50 | 3 | 7.10 | t=6.40 x=21.65 | 0.072 / 0.295 | detect +0.05 s, brake +1.50 s, stop x=53.55, obst clr 1.97 m | 8 |

**15/15 runs pass.** min clearance is the evaluator's disc-to-disc minimum over all objects; rejected_commands includes startup ticks before the first valid command.
