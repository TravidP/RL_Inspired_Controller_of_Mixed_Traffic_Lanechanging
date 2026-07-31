#!/usr/bin/env bash
set -euo pipefail

deck_dir="$(cd "$(dirname "$0")" && pwd)"
cd "$deck_dir"

latexmk -xelatex -interaction=nonstopmode -halt-on-error \
  -jobname=IFAC2026_PARC_Methodology_Audience \
  IFAC2026_PARC_Methodology.tex

xelatex -interaction=nonstopmode -halt-on-error \
  -jobname=IFAC2026_PARC_Methodology_Notes \
  '\def\EnableNotes{1}\input{IFAC2026_PARC_Methodology.tex}'
xelatex -interaction=nonstopmode -halt-on-error \
  -jobname=IFAC2026_PARC_Methodology_Notes \
  '\def\EnableNotes{1}\input{IFAC2026_PARC_Methodology.tex}'
