# Submission and AI Use

Submit **one individual ZIP** by **November 1, 2026, 23:59 KST** to [fairytale@kaist.ac.kr](mailto:fairytale@kaist.ac.kr). In the email, give your name, student ID, and mode (**easy** or **full**). If submitting both, identify which result to evaluate. Results and a **five-minute individual Q&A** about your report and code follow in class on **November 3, 2026**.

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

Easy runs from `easy_plan.json`; Full runs from `Controller.reset/act` in `controller.py`. Submit your solution files, not replacements for the [fixed evaluation components](RUNNING.md#what-you-may-change). See the [interface](INTERFACE.md) for contracts and commands.

## Report

Use the [IEEE-style two-column template](report_template.tex). **Two pages are recommended**; length is not graded. Include these four sections:

1. **Problem Statement:** State, actions, constraints, objective, and relation of your training objective to the official minimum-time task.
2. **Proposed Approach:** Course concepts implemented (for example, DP, VI/PI, value or policy approximation, rollout, multistep lookahead); model or data, training, and online action selection. **Include at least one representative figure** of your method or controller.
3. **Own Validation:** Your PyChrono outcome and time, meaningful baseline comparison, supporting evidence, limitations, and failures.
4. **Conclusion:** Result and brief AI-use disclosure.

Choose a course-based method and explain how your computed plan, policy, or learned parameters determine the actions used in PyChrono. See [Assessment](ASSIGNMENT.md#assessment) for scoring rules.

## AI use

AI may help write and debug code, but **you must lead the design and check its output**. In the **individual Q&A** on **November 3**, explain your method, results, and code to show that you understand the work and directed any AI use.

In the report's Conclusion, name each **AI tool and model** and its main uses (such as drafting, debugging, or editing). State if you used none. No AI log or transcript is required. AI use alone does not change your score; understanding and consistency among code, report, and measured results do.

Cite external code and data. Report only results you obtained. The submitted controller must run locally from submitted files, without an online AI service at evaluation time.
