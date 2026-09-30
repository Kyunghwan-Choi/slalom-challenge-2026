# Submission and AI Use

Submit individually by **1 November 2026, 23:59 KST**. Email one ZIP file to [fairytale@kaist.ac.kr](mailto:fairytale@kaist.ac.kr). State your name, student ID, and selected mode (**easy** or **full**) in the email. If you submit both modes, identify which result you want evaluated for the base challenge. The results will be announced in class on **3 November 2026**. Each student will then have a **five-minute individual discussion** of the report and code.

## What to submit

Your ZIP needs a PDF report and the code needed to reproduce and run your method:

~~~text
submission/
  report.pdf
  easy_plan.json           # easy mode, if selected
  controller.py            # full mode, if selected
  ...                      # only the helper code and learned artifacts you use
~~~

For the easy mode, include the code and settings that produced your plan. For the full mode, include any training code and settings used to produce the submitted controller. Include learned weights, tables, or other files that the entry point loads, so the evaluator can run it **without retraining**. You may use additional packages while training; if you do, list their pinned versions in a short `requirements.txt`. The submitted controller's **evaluation-time imports must work in the supplied environment**: additional packages from `requirements.txt` will not be installed for scoring. Do not include a Conda environment, PyChrono installation, or large raw training datasets; provide the data-generation procedure or source instead. Your submitted files must correspond to the method and results in the report.

The entry point follows the mode you select: `easy_plan.json` for easy mode and `Controller.reset/act` in `controller.py` for full mode. The exact JSON and Python contracts and local commands are in [Running the challenge](RUNNING.md) and [Controller interface](INTERFACE.md).

## Report

Use the supplied [IEEE-style two-column template](report_template.tex). **Two pages are recommended**, but page count itself is not graded. Keep these four sections:

1. **Problem Statement:** Define your state, actions, constraints, objective, and how your training objective relates to the official minimum-time task.
2. **Proposed Approach:** Explain the course concepts you actually implemented, such as DP, VI/PI, value or policy approximation, rollout, or multistep lookahead. Give the model or data source, training procedure, and online action selection. **Include at least one representative figure** of your method or controller.
3. **Own Validation:** Report your PyChrono completion or failure and time, compare with a meaningful baseline, and show enough evidence to support your claims. State limitations or failure cases.
4. **Conclusion:** Summarize the result and include a brief AI-use disclosure as described below.

You may choose any suitable course-based method; PI and a learned value function are not mandatory. The report should make clear what your learning or optimization step changed in the actual controller. A generic RL library run without a course-based problem formulation, controller design, and validation receives **zero credit**. The report's length and the name or size of an algorithm are not scoring criteria. The absolute-time scoring schedule will be announced separately. A failed drive receives the minimum performance score.

## AI use

AI tools **may be used to help write and debug code**. You must lead the design, verify the outputs, and be able to explain the submitted method, results, and code during the five-minute discussion.

The individual questions on November 3 will also check whether you directed the AI use yourself and understand the work it produced.

In the Conclusion of your report, list the **tool and model names** you used and their main uses (for example, code drafting, debugging, or editing). If you did not use AI, state that. A separate AI log or conversation transcript is not required. AI use alone does not change your score; your understanding and the consistency of your code, report, and measured results do.

Cite any external code or data you used. Report only results you actually obtained. Your controller must run from the submitted files in the common local environment; it must not require an online AI service during evaluation.
