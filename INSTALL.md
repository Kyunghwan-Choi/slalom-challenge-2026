# Install and check the environment (Windows x64)

The supplied [Conda package lock](pychrono-win-64-explicit.txt) specifies the tested Windows x64 environment, including Python 3.12, PyChrono 10.0.0, and NumPy. You need an internet connection to download its packages. These steps use the **Miniconda Prompt** so that `conda` is available without changing PowerShell's execution policy.

1. Install [Miniconda for Windows x86-64](https://docs.conda.io/projects/conda/en/latest/user-guide/install/windows.html). Open **Miniconda Prompt** from the Windows Start menu and check the installation:

   ```bat
   conda --version
   ```

2. Download or clone this repository and open its folder in File Explorer. Copy the folder path from the address bar. In Miniconda Prompt, change to that folder. For a path containing spaces, keep the quotation marks:

   ```bat
   cd /d "C:\path\to\slalom-challenge"
   ```

   `README.md` and `pychrono-win-64-explicit.txt` should be in the current folder. Entering a folder path by itself does not change directories.

3. Create the course environment from the lock file. Run this once:

   ```bat
   conda create -n slalom2026 --file pychrono-win-64-explicit.txt
   ```

4. Verify the interpreter, PyChrono, and the provided interfaces:

   ```bat
   conda run --no-capture-output -n slalom2026 python -c "import sys, pychrono, pychrono.vehicle, numpy; print(sys.version.split()[0]); print('PyChrono import OK')"
   conda run --no-capture-output -n slalom2026 python test_contract.py
   conda run --no-capture-output -n slalom2026 python test_easy_mode.py
   ```

   The import check should print Python `3.12.x` and `PyChrono import OK`; both test commands should finish with `OK`.

5. Run the supplied low-speed example:

   ```bat
   conda run --no-capture-output -n slalom2026 python run_local.py --mode full --controller controller.py --output runs\example
   ```

   Open `runs\example\result.json` to see the result. The runner is headless by default: it writes data files without opening a driving window. Add `--visual` to display the car, cones, and finish line. See [Running and interfaces](RUNNING.md) for both modes and output fields.

You can use the same `conda run --no-capture-output -n slalom2026 python ...` prefix in VS Code's PowerShell terminal **if** `conda` is available there. Otherwise, use Miniconda Prompt or select its terminal profile in VS Code. If you prefer a Python interpreter in VS Code, select the `python.exe` inside the `slalom2026` Conda environment; opening an existing terminal does not automatically change its interpreter.

The [PyChrono installation guide](https://api.chrono.projectchrono.org/pychrono_installation.html) explains the underlying package. The provided lock is the environment specification for this challenge. The included validation plots can be viewed without extra packages; regenerating them with `validate_model.py` additionally requires Matplotlib.
