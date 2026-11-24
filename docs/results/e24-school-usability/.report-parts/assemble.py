import os
R="/home/claude/work/repo/docs/results/e24-school-usability/"
H="/home/claude/work/e24/report/"
s=open(H+"body.md").read()
rd=lambda f: open(f).read().strip()
s=s.replace("<<ACCEPT>>", rd(R+"acceptance/table.md")+"\n\n"+rd(H+"accept_tail.md"))
s=s.replace("<<DOE>>", rd(H+"doe.md"))
s=s.replace("<<SUITES>>", rd(H+"suites.md"))
s=s.replace("<<CONCLUSION>>", rd(H+"conclusion.md"))
s=s.replace("<<REVIEW>>", rd(H+"review.md"))
s=s.replace("<<SWEEP1>>", rd(H+"sweep1.md"))
s=s.replace("<<SWEEP2>>", rd(H+"sweep2.md"))
assert "<<" not in s
open("/home/claude/work/repo/docs/build-reports/e24-nov24-school-and-usability.md","w").write(s)
d=R+".report-draft.md"
if os.path.exists(d): os.remove(d)
print(len(s.splitlines()),"lines")
