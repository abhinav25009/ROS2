#!/bin/bash
# build.sh: paper.md -> paper.tex (LaTeX source for Overleaf, biblatex) and
# paper.pdf (via HTML + headless Chrome; no TeX install needed)
set -e
cd "$(dirname "$0")"
pandoc paper.md -s --biblatex --bibliography=refs.bib -o paper.tex
sed 's/figs\/\([a-z0-9_]*\)\.pdf/figs\/\1.png/g' paper.md > .paper_html.md
pandoc .paper_html.md -s --citeproc --bibliography=refs.bib --math-method=mathml --css paper.css \
  --metadata link-citations=true --embed-resources -o .paper.html
# wrap the body (after the title block) in a two-column div
python3 - <<'PY'
s = open('.paper.html').read()
i = s.index('</header>') + len('</header>')
j = s.rindex('</body>')
open('.paper.html', 'w').write(s[:i] + '<div class="cols">' + s[i:j] + '</div>' + s[j:])
PY
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless=new --disable-gpu \
  --no-pdf-header-footer --print-to-pdf="$PWD/paper.pdf" "file://$PWD/.paper.html" 2>/dev/null
rm -f .paper_html.md
echo built paper.tex paper.pdf

# report.md -> ../AHPPA_v3_final_report.pdf (single column)
sed 's/figs\/\([a-z0-9_]*\)\.pdf/figs\/\1.png/g' report.md > .report_html.md
pandoc .report_html.md -s --css paper.css --embed-resources --metadata title-prefix= -o .report.html
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless=new --disable-gpu \
  --no-pdf-header-footer --print-to-pdf="$PWD/../AHPPA_v3_final_report.pdf" "file://$PWD/.report.html" 2>/dev/null
rm -f .report_html.md
echo built ../AHPPA_v3_final_report.pdf
