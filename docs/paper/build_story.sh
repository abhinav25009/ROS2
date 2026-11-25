#!/bin/bash
# story.md -> ../AHPPA_complete_story_report.pdf (single column, via HTML + headless Chrome)
set -e
cd "$(dirname "$0")"
pandoc story.md -s --css paper.css --embed-resources --metadata title-prefix= -o .story.html
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless=new --disable-gpu \
  --no-pdf-header-footer --print-to-pdf="$PWD/../AHPPA_complete_story_report.pdf" "file://$PWD/.story.html" 2>/dev/null
echo built ../AHPPA_complete_story_report.pdf
