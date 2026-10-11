# Tepee Mechanism: Kinematic Analysis, MATLAB Code and CAD

Course project for **ME 4504, Mechanics of Machines Lab** (Department of Mechanical and Production Engineering, Islamic University of Technology)
This repository holds the group's MATLAB code, an extended MATLAB analysis built on it, and CAD file of the linkage.

*Credit.* The report and the original MATLAB code are the combined group's work. The extended MATLAB analysis, the CAD and this README were added afterwards by Zahin Tasnim Ridita.



<p align="center"><img src="figures/tepee\_motion.gif" width="70%"></p>

## The mechanism

A crank **r₂** turns about the fixed pivot O. A coupler **r₃** of the same length joins the crank tip P to a slider S that moves along the line through O.
With four links and four one-degree-of-freedom joints, Kutzbach's equation gives M = 3(4 − 1) − 2·4 = **1**, so one input angle θ₂ fixes the motion.

<p align="center"><img src="figures/01\_mechanism.png" width="90%"></p>

Because r₂ = r₃, triangle OPS is isosceles and the slider position is exactly harmonic: **r₁ = 2 r₂ cos θ₂**, with θ₃ = π − θ₂.
The report notes that the mechanism is "sometimes also called Scotch Yoke". Its slider output is the same simple harmonic motion as a Scotch yoke's, but it is a linkage, not a pin-in-slot yoke (see the comparison below).

## Kinematic analysis

Vector loop, split into x and y components, with unknowns r₁ and θ₃:

r₁ − r₂ cos θ₂ + r₃ cos θ₃ = 0  and  −r₂ sin θ₂ + r₃ sin θ₃ = 0

Position comes from Newton's method (warm-started from the previous angle). Differentiating the loop gives J·\[ṙ₁; θ̇₃] = \[−r₂ sin θ₂ θ̇₂; r₂ cos θ₂ θ̇₂] and the corresponding acceleration equation, with J = \[1, −r₃ sin θ₃; 0, r₃ cos θ₃].
For r₂ = r₃: ṙ₁ = −2 r₂ sin θ₂ θ̇₂, r̈₁ = −2 r₂ cos θ₂ θ̇₂², θ̇₃ = −θ̇₂.
This is a **kinematic** analysis (position, velocity, acceleration). Forces and torques are not analysed.

**Course inputs:** r₂ = r₃ = 10, θ₂ from 80° to 0° in 30 frames, θ̇₂ = 1 rad/s, θ̈₂ = 0. (The report mixes length units; they are treated as plain "length units" here.)

|θ₂|r₁|ṙ₁|r̈₁|
|-|-|-|-|
|80°|3.47|-19.70|-3.47|
|0°|20.00|-0.00|-20.00|

<p align="center"><img src="figures/02\_kinematics\_course\_range.png" width="60%"></p>

*Output of the submitted MATLAB code (run in GNU Octave 8.4), with the closed-form curves dashed.*

**Verification.** The course code (Newton tolerance 1e-3) agrees with the closed form to within 9e-04 in r₁, 4e-03 in ṙ₁ and 9e-04 in r̈₁.
The revised function (`matlab/tepee\_kinematics.m`) with tolerance 1e-10 agrees to within 1e-10, 2e-10 and 5e-10.
For unequal links (r₃ = 1.2 r₂, θ̈₂ ≠ 0) it agrees with finite differences of the position, so the same code covers mechanisms beyond the course case. See [`matlab/README.md`](matlab/README.md).

## Beyond the course range

Running the crank through a full revolution (`matlab/tepee\_analysis.m`) gives the slider position r₁ = 2 r₂ cos θ₂ over 360°, a stroke of 4 r₂.

<p align="center"><img src="figures/03\_full\_revolution.png" width="60%"></p>

* At θ₂ = 90° and 270° the slider reaches the crank pivot and det J = r₃ cos θ₃ = 0. There the tepee configuration meets a second assembly mode with r₁ = 0 (link 3 folded onto link 2, θ₃ = θ₂), found by starting Newton's method from a different guess.
* The course range (80° to 0°) stays clear of this position. To pass through it, the slider and coupler have to run in a different plane from the crank, which is how the CAD below is laid out.

## CAD

<p align="center"><img src="figures/04\_cad\_parts\_and\_assembly.png" width="90%"></p>
<p align="center"><img src="figures/05\_cad\_assembly.png" width="65%"></p>

Scale: r₂ = r₃ = 25 mm, i.e. 2.5 mm per report unit, so the slider stroke is 100 mm. Each dimension is a parameter at the top of [`cad/tepee/build\_tepee.py`](cad/tepee/build_tepee.py).

|Part|Volume|Notes|
|-|-|-|
|Base|191.8 cm³|240 × 100 × 8 mm plate with Ø8 pivot post and 4 mounting holes|
|Guide bridge|77.4 cm³|slotted plate on two end walls, carries the slider above the linkage|
|Crank (link 2)|3.2 cm³|14 × 6 mm link with a boss and Ø6 pin at P|
|Coupler (link 3)|2.6 cm³|same outline, two Ø6.3 holes 25 mm apart|
|Slider|6.0 cm³|30 × 16 × 12 mm block with a Ø6 pin|

**Motion check.** The assembly was placed at 24 crank angles (15° apart) around a full revolution, with the coupler and slider following the kinematics above.

|Check|Result|
|-|-|
|Interference between any pair of the 5 parts|0 mm³ at every angle|
|Pin-to-hole gap (crank pin, slider pin)|0.15 mm|
|Slider block to bridge slot|0.3 mm|
|Coupler to crank pivot post and bridge|2.0 mm|
|Contact by design|bridge on base; coupler resting on the crank boss|

Dimensions for rebuilding the parts in SolidWorks: [`docs/tepee\_solidworks\_guide.md`](docs/tepee_solidworks_guide.md).

## Comparison with a Scotch yoke

[`cad/scotch\_yoke\_reference/`](cad/scotch_yoke_reference/) holds CAD of a pin-in-slot Scotch yoke with crank radius 20 mm, so its yoke moves as x = 20 cos θ.
That is the same motion as the tepee with r₂ = r₃ = 10 (maximum difference between the two curves: 0).

||Tepee linkage|Scotch yoke|
|-|-|-|
|Crank length for a given stroke|stroke / 4|stroke / 2|
|Output motion|exactly harmonic|exactly harmonic|
|Joints|revolute joints plus a sliding slider|sliding pin in a slot, two guide rods|
|Packaging|crank, coupler and slider in separate layers|crank and yoke plate in layers|

A point on the line of link 3 extended past P by r₃ moves on the straight line x = 0 (maximum |x| in the check: 0), the basis of the Scott Russell straight-line linkage.

## Reproducing the results

Analysis and figures 01–03 (MATLAB or GNU Octave, from `matlab/`):

```
validate\_tepee
tepee\_analysis
```

CAD and figures 04–05 (Python, from the repo root):

```
pip install -r requirements.txt
python cad/tepee/build\_tepee.py
python cad/tepee/check\_motion.py
python cad/tepee/render\_cad.py
```

## Repository contents

```
├── matlab/                 course code (original\_submission/), kinematics function, validation, full analysis, outputs/
├── cad/
│   ├── tepee/              parametric build script, motion check, renders, STEP and STL of every part and the assembly
│   └── scotch\_yoke\_reference/   comparison mechanism
├── figures/  results/      images, summary.txt and CAD check results used above
```

## Sources cited in the group report

1. YouTube video (link as given in the report): https://www.youtube.com/watch?v=L-r15ES3vFI\&list=PLTMo3nrl2KWbeCx8-OGfrMHcFOUSjyk4D\&index=8
2. *ME 3011 Final Report (Revised)*, hosted at jonathanmbowman.com (link as given in the report): https://www.jonathanmbowman.com/pdfs/ME%203011%20Final%20Report%20Revised.pdf

