# Kinematics of the myCobot 280 — A Worked Book

A textbook-style teaching document that explains the **mathematics and the code**
of the myCobot 280 projects, in the **notation of the reference notes** (modified
Denavit–Hartenberg, the velocity-propagation method), with **Key point** and
**Meaning** callouts throughout.

## Files

| File | What it is |
|------|-----------|
| `book.md` | The book source (the "log file" — Markdown, editable) |
| `../docs/mycobot_thesis_book.pdf` | The built teaching PDF (book-quality equations) |
| `render_equations.py` | Renders the equations as images in reference notation |
| `eq/` | The rendered equation images |
| `build_book.sh` | Rebuilds everything: equations → Word → PDF |

## Contents

- **Preface** — notation key (`ⁱ⁻¹ᵢT`, `ⁱωᵢ`, `θ̇`, `Ẑ`) and how to read it
- **Ch. 1** — the robot and its frames (the four DH parameters and their meaning)
- **Ch. 2** — forward kinematics (the modified-DH transform matrix, the product)
- **Ch. 3** — the Jacobian by velocity propagation (angular & linear, geometric form)
- **Ch. 4** — inverse kinematics (damped least squares) + solver comparison
- **Ch. 5** — manipulability and the task-constrained (drawable) workspace
- **Ch. 6** — reading the code (each module mapped to its chapter)
- **Appendix** — identified DH table + run commands

Every equation in the book is implemented in `mycobot_thesis/robot.py` and
`mycobot_thesis/ik.py`; each chapter names the file and function to open alongside it.

## Rebuild

```bash
cd ~/ros2_ws/src/mycobot_thesis
bash book/build_book.sh          # -> docs/mycobot_thesis_book.pdf
```

Requires `pandoc`, `libreoffice`, and Python `matplotlib` (all installed). The math
is rendered as images (matplotlib) because the reference left-super/subscript
notation needs it; no LaTeX engine is required.
