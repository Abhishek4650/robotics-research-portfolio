#!/usr/bin/env bash
# Build the research-paper PDF: markdown (LaTeX math) -> Word (OMML) -> PDF.
set -e
cd "$(dirname "$0")/.."
pandoc paper/paper.md -o docs/mycobot_thesis_paper.docx
libreoffice --headless --convert-to pdf --outdir docs docs/mycobot_thesis_paper.docx >/dev/null 2>&1
echo "wrote docs/mycobot_thesis_paper.pdf"
