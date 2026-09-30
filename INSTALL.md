# Install and check the environment (Windows x64)

The [Conda lock](pychrono-win-64-explicit.txt) specifies the tested Windows x64 environment: Python 3.12, PyChrono 10.0.0, and NumPy. Package download requires internet. Use **Miniconda Prompt** so `conda` works without changing PowerShell's execution policy.

1. Install [Miniconda for Windows x86-64](https://docs.conda.io/projects/conda/en/latest/user-guide/install/windows.html). Open **Miniconda Prompt** from Start and check:

   ```bat
   conda --version
   ```

2. Download or clone this repository. Copy its folder path from File Explorer's address bar, then change directory in Miniconda Prompt (keep quotes around paths with spaces):

   ```bat
   cd /d "C:\path\to\slalom-challenge"
   ```

   The current folder should contain `README.md` and `pychrono-win-64-explicit.txt`. Entering a path alone does not change directories.

3. Create the environment once:

   ```bat
   conda create -n slalom2026 --file pychrono-win-64-explicit.txt
   ```

4. Check Python, PyChrono, and both interfaces:

   ```bat
   conda run --no-capture-output -n slalom2026 python -c "import sys, pychrono, pychrono.vehicle, numpy; print(sys.version.split()[0]); print('PyChrono import OK')"
   conda run --no-capture-output -n slalom2026 python test_contract.py
   conda run --no-capture-output -n slalom2026 python test_easy_mode.py
   ```

   Expect Python `3.12.x`, `PyChrono import OK`, and `OK` from each test.

5. Run the slow Full-mode example:

   ```bat
   conda run --no-capture-output -n slalom2026 python run_local.py --mode full --controller controller.py --output runs\example
   ```

   Check `runs\example\result.json`. Runs are headless by default; add `--visual` to see the car, cones, and finish line. [RUNNING.md](RUNNING.md) covers both modes and outputs.

The same `conda run --no-capture-output -n slalom2026 python ...` prefix works in VS Code's PowerShell terminal if `conda` is available there. Otherwise, use Miniconda Prompt or its VS Code terminal profile. To use a VS Code interpreter, select `python.exe` inside `slalom2026`; an already open terminal does not switch automatically.

The [PyChrono installation guide](https://api.chrono.projectchrono.org/pychrono_installation.html) explains the package; this challenge uses the supplied lock. Included validation plots need no extra package to view, but `validate_model.py` needs Matplotlib to regenerate them. Evaluation uses only the supplied Conda environment; extra packages may be used for offline training under the [submission rules](SUBMISSION_AND_AI.md).
