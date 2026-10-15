#!/usr/bin/env python3
"""Build the full v2 draft: convert [key] citations to \\cite{key} (only known BibTeX keys), compile."""
import re, os, subprocess, shutil
R = os.path.dirname(os.path.abspath(__file__)); F = os.path.join(R, 'full')
os.makedirs(F, exist_ok=True)
keys = set()
for b in ('refs_v2.bib', 'refs_v1sel.bib'):
    keys |= set(re.findall(r'@\w+\{\s*([^,\s]+)\s*,', open(os.path.join(R, b)).read()))
    shutil.copy(os.path.join(R, b), F)
def conv(s):
    def rep(m):
        ks = [k.strip() for k in m.group(1).split(',')]
        return '\\cite{' + ','.join(ks) + '}' if all(k in keys for k in ks) else m.group(0)
    return re.sub(r'\[([^\[\]$\\]+?)\]', rep, s)
for f in ['front_back.tex', 'sec1_intro.tex', 'sec2_background.tex', 'sec3_problem.tex', 'sec4_model.tex', 'sec5_platform.tex',
          'sec6_method.tex', 'tables_v2.tex', 'table1_v2.tex'] + [os.path.join('figs', x) for x in os.listdir(os.path.join(R, 'figs')) if x.startswith('fig_') and x.endswith('.tex')]:
    p = os.path.join(R, f)
    if os.path.exists(p):
        open(os.path.join(F, os.path.basename(f)), 'w').write(conv(open(p).read()))
fb = open(os.path.join(F, 'front_back.tex')).read()
open(os.path.join(F, 'abstract.tex'), 'w').write(fb[fb.index('\\begin{abstract}'):fb.index('\\end{abstract}') + 14])
open(os.path.join(F, 'back.tex'), 'w').write(fb[fb.index('\\section{Conclusion}'):])
if os.path.exists(os.path.join(R, 'sec7_results.tex')):
    for x in ('sec7_results.tex', 'sec8_discussion.tex'):
        if os.path.exists(os.path.join(R, x)):
            open(os.path.join(F, x), 'w').write(conv(open(os.path.join(R, x)).read()))
unknown = set()
for f in os.listdir(F):
    if f.endswith('.tex') and f != 'full.tex':
        for m in re.finditer(r'\[([A-Za-z][A-Za-z0-9:_\-]*(?:,\s*[A-Za-z][A-Za-z0-9:_\-]*)*)\]', open(os.path.join(F, f)).read()):
            if any(c.isupper() or c.isdigit() for c in m.group(1)) and m.group(1) not in ('t', 'b'):
                unknown.add(m.group(1))
print('bracket groups left (check):', sorted(unknown))
for cmd in (['pdflatex', '-interaction=nonstopmode', 'full.tex'], ['bibtex', 'full'], ['pdflatex', '-interaction=nonstopmode', 'full.tex'], ['pdflatex', '-interaction=nonstopmode', 'full.tex']):
    subprocess.run(cmd, cwd=F, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
log = open(os.path.join(F, 'full.log'), errors='ignore').read()
print('undefined citations:', sorted(set(re.findall(r"Citation `([^']+)' .*undefined", log))))
print('undefined refs:', sorted(set(re.findall(r"Reference `([^']+)' .*undefined", log))))
print(subprocess.run(['pdfinfo', 'full.pdf'], cwd=F, capture_output=True, text=True).stdout.split('Pages:')[1].split('\n')[0].strip(), 'pages')
