# MATLAB code

All analysis, results and figures in this repository come from the files below.

| File | What it does |
|---|---|
| `original_submission/tepee_simulation.m`, `solveTeepee.m` | The Group 4 code from the ME 4504 report, as submitted |
| `run_original_submission.m` | Runs the submitted script (without its animation loop) and returns its results |
| `tepee_kinematics.m` | The same vector-loop and Newton method as a function, for any r₂, r₃, θ̇₂ and θ̈₂ |
| `validate_tepee.m` | Checks `tepee_kinematics` against the closed-form solution and against finite differences |
| `tepee_analysis.m` | Runs everything: course results, full-revolution analysis, figures 01–03, the animation, `outputs/*.csv` and `results/summary.txt` |
| `write_csv.m` | Small CSV helper |

Run from this folder:

```
validate_tepee
tepee_analysis
```

**Differences between `tepee_kinematics.m` and the submitted script**

- Accelerations come from the matrix solve `J \ ba`, which also covers r₂ ≠ r₃. For the course case (r₂ = r₃, θ̈₂ = 0) the submitted script gives the same results.
- The θ̈₂ term in the second acceleration equation follows the derivative of the y loop equation.
- Newton's method stops after 50 iterations, since J is singular at θ₂ = 90°.
- An optional starting guess selects the assembly mode.

The code was run with GNU Octave 8.4 and uses only functions that also exist in MATLAB.
