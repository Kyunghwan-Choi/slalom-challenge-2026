# Optional code checks

Ruff formats Python code and checks common errors and import order. It is optional and runs in a separate development environment; use the [course environment](INSTALL.md) for simulation.

Run these commands from the repository root in PowerShell or Miniconda Prompt with Python available. No environment activation is needed:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
```

Check the code without modifying it:

```powershell
.\.venv\Scripts\python.exe -m ruff check .
.\.venv\Scripts\python.exe -m ruff format --check .
```

To apply automatic lint fixes and formatting:

```powershell
.\.venv\Scripts\python.exe -m ruff check --fix .
.\.venv\Scripts\python.exe -m ruff format .
```

[pyproject.toml](pyproject.toml) sets Python 3.12 syntax, a 100-character formatting target, double quotes, and basic error/import checks. Markdown is excluded. [requirements-dev.txt](requirements-dev.txt) pins the Ruff version. Review fixes and run the interface checks in [Installation](INSTALL.md) after editing control or simulation code.
