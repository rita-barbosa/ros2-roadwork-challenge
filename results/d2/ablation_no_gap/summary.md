# Episode summary: results/d2/ablation_no_gap

```
date: 2026-11-25T17:50:23+00:00
software: d1-submission-4-g63c8bf9
ros_distro: jazzy  ROS_DOMAIN_ID: 37
host: Linux alf-Ubuntu-24-04-PC-Q35-ICH9-2009 7.0.0-42-generic #42~24.04.1-Ubuntu SMP PREEMPT_DYNAMIC Mon Oct 19 14:43:01 UTC 2 x86_64 x86_64 x86_64 GNU/Linux
cpu:  AMD Ryzen 5 7600X3D 6-Core Processor
cpus: 4  mem: 7.8Gi
visualization: off (rviz:=false visualize:=false)
scenarios: 3  seeds: 1 2 3  extra: use_traffic_gap:=false  label: ablation: no radar gap check
```

| run | result | violations | sim time [s] | min clearance [m] | max speed [m/s] | stopped [s] | rejected cmds | entry t [s] | GO latched | mean/max \|cte\| [m] | scenario measurements | transitions |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|---:|
| s3_seed1 | FAIL | collision | 13.05 | -0.164 | 3.500 | 1.05 | 11 | 7.60 | t=6.90 x=21.65 | 0.051 / 0.232 | yield 0.0 s, gap safe t=-, car clr -0.16 m | 5 |
| s3_seed2 | FAIL | collision | 12.75 | -0.108 | 3.500 | 0.55 | 9 | 7.10 | t=6.40 x=21.65 | 0.052 / 0.232 | yield 0.0 s, gap safe t=-, car clr -0.11 m | 6 |
| s3_seed3 | FAIL | collision | 12.75 | -0.045 | 3.500 | 0.60 | 9 | 7.15 | t=6.45 x=21.65 | 0.052 / 0.232 | yield 0.0 s, gap safe t=-, car clr -0.04 m | 6 |

**0/3 runs pass.** min clearance is the evaluator's disc-to-disc minimum over all objects; rejected_commands includes startup ticks before the first valid command.
