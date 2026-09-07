#!/usr/bin/env bash
# Build the teaching book PDF: markdown + rendered equation images -> Word -> PDF.
set -e
cd "$(dirname "$0")/.."
python3 book/render_equations.py
pandoc book/book.md -o docs/mycobot_thesis_book.docx
libreoffice --headless --convert-to pdf --outdir docs docs/mycobot_thesis_book.docx >/dev/null 2>&1
echo "wrote docs/mycobot_thesis_book.pdf"
