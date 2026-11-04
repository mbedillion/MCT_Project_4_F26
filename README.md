# MCT-F26 Project 4

MATLAB R2025a modernization of the Project 4 EKF-SLAM assignment for Webots
R2025a. It uses the same tractor, trajectory, vehicle parameters, and MATLAB
workflow as Projects 1–3 while retaining Project 4's estimator equations and
grading thresholds. Python is not required.

## Requirements

- Webots R2025a
- MATLAB R2025a
- Windows: MATLAB Support for MinGW-w64 C/C++ Compiler
- macOS: Xcode Command Line Tools with a MATLAB-supported Clang compiler
- Ubuntu Linux: a GCC release supported by the installed MATLAB version

The controller selects `driver.dll` on Windows, `libdriver.dylib` on macOS,
and `libdriver.so` on Linux. macOS and Ubuntu support follows the same code path
as Project 1 but remains runtime-unverified. Do not use the Linux Snap package,
whose sandbox does not support MATLAB controllers.

## Run from Webots

Open `worlds/automotive_new.wbt`, then press **Run**. Webots launches
`controllers/main_matlab/main_matlab.m`; do not run it directly because the
Webots controller connection must already exist.

If MATLAB is not discovered, set Webots **Tools > Preferences > General >
MATLAB command** to the MATLAB R2025a executable. Reset the simulation after
editing controller files. If the MATLAB bridge is stale, close Webots and
MATLAB, then remove only the generated `protofile_matlab_*` and
`libController_thunk_*` files from your temporary directory. Do this only after
Webots and MATLAB have exited so no thunk DLL remains locked.

The repository includes Webots' Windows controller thunk for installations
that generate the API prototype without emitting the matching DLL.
The launcher restores that thunk to the temporary bridge directory when
needed; it does not modify the Webots or MATLAB installations.

## Assignment work

Read the Fall 2026 directions, `MCT_Project_4_2026.pdf`. Edit only:

- `controllers/main_matlab/EKF_SLAM.m` — nonlinear motion and measurement
  models, both Jacobians, and the predict/correct sequence.
- `controllers/main_matlab/your_controller.m` — lateral and longitudinal
  control, reusing or refining Project 3 work as permitted by the assignment.

Keep angular innovations in `[-pi, pi]` and use `n` rather than hard-coding
the 49 landmarks. Full-credit criteria are a completed loop within 200 s,
maximum deviation no greater than 10 m, and average deviation no greater than
5 m. The average absolute steering-angle change must be no greater than
0.025 rad/timestep. Completion prints the grade, saves `controller_results.png`,
and keeps the result window open until it is closed.

Before running Webots, test the estimator from MATLAB:

```matlab
cd controllers/main_matlab
test_ekf
```

The test numerically checks both Jacobians and verifies that one complete
predict/correct step produces a finite, symmetric, positive-semidefinite
covariance. It contains no instructor solution values.

The vehicle constants match Projects 1–3: `m = 1000 kg`, `lr = 0.82 m`,
`lf = 1.18 m`, and `Iz = 3004.5 kg m^2`.

Instructor solutions, proof logs, generated plots, MATLAB build products, and
Webots UI state are intentionally excluded from Git.
