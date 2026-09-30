# Submission and AI Use

Submit **one individual ZIP** by **1 November 2026, 23:59 KST** to [fairytale@kaist.ac.kr](mailto:fairytale@kaist.ac.kr). In the email, give your name, student ID, and mode (**easy** or **full**). If submitting both, identify the base-challenge result to evaluate. Results and a **five-minute individual discussion** of your report and code follow in class on **3 November 2026**.

## What to submit

Include the PDF report and files needed to reproduce and run your method:

~~~text
submission/
  report.pdf
  easy_plan.json           # easy mode, if selected
  controller.py            # full mode, if selected
  ...                      # only the helper code and learned artifacts you use
~~~

- **Easy:** Include the code and settings that produced `easy_plan.json`.
- **Full:** Include training code and settings used to produce `controller.py`.
- **Both:** Include every learned weight, table, or other file loaded by the entry point. Evaluation must run **without retraining**. Submitted files must match your reported method and results.
- **Packages:** Extra packages may be used for training; list pinned versions in `requirements.txt`. **Evaluation-time imports must work in the supplied Conda environment**. Scoring will not install packages from `requirements.txt`.
- **Omit:** Conda environments, PyChrono installations, and large raw training datasets. Give the data source or generation procedure instead.

Easy runs from `easy_plan.json`; Full runs from `Controller.reset/act` in `controller.py`. See [Running](RUNNING.md) and the [interface](INTERFACE.md) for contracts and commands.

## Report

Use the [IEEE-style two-column template](report_template.tex). **Two pages are recommended**; length is not graded. Include these four sections:

1. **Problem Statement:** State, actions, constraints, objective, and relation of your training objective to the official minimum-time task.
2. **Proposed Approach:** Course concepts implemented (for example, DP, VI/PI, value or policy approximation, rollout, multistep lookahead); model or data, training, and online action selection. **Include at least one representative figure** of your method or controller.
3. **Own Validation:** Your PyChrono outcome and time, meaningful baseline comparison, supporting evidence, limitations, and failures.
4. **Conclusion:** Result and brief AI-use disclosure.

Choose any suitable course-based method; PI and a learned value function are optional. Explain what learning or optimization changed in the actual controller. A generic RL library run without course-based formulation, controller design, and validation receives **zero credit**. Algorithm name and size do not determine the score. Absolute-time thresholds will be announced separately; a failed drive receives the minimum performance score.

## AI use

AI may help write and debug code. **Lead the design, verify outputs, and explain your method, results, and code** in the November 3 discussion. Individual questions also check that you directed the AI use and understand its output.

In the report's Conclusion, name each **AI tool and model** and its main uses (such as drafting, debugging, or editing). State if you used none. No AI log or transcript is required. AI use alone does not change your score; understanding and consistency among code, report, and measured results do.

Cite external code and data. Report only results you obtained. The submitted controller must run locally from submitted files, without an online AI service at evaluation time.
