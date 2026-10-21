"""The project's accuracy report (Maxfield IRL Tracking): one self-contained HTML page.

    python tools/recognition/report.py <project folder>        writes <project>/report.html

Also served live by the studio at /report. Everything in it is read from the project
folder (project.json, models/curve.json, models/<vN>/eval_*.csv), so it always matches
what the studio shows.
"""
import csv
import html as _html
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import project as projmod  # noqa: E402
import timesaved  # noqa: E402

BRAND = "Maxfield IRL Tracking"
TAGLINE = "Identify. Recognize. Label."


def _pct(v):
    return "–" if v is None else "%d%%" % round(100 * v)


def _num(v, d=1):
    try:
        x = float(v)
        return "–" if x != x else ("%." + str(d) + "f") % x
    except (TypeError, ValueError):
        return "–"


def _curve_svg(points):
    """The learning curve: one place per model version, oldest to newest (no model first), each
    labelled with the confirmed frames it trained on. Two models trained on the same frames sit
    side by side, not on top of each other."""
    import trainer
    pts = sorted([p for p in points if p.get("f1") is not None], key=lambda p: trainer.version_order(p["model"]))
    if not pts:
        return "<p>No model has been scored yet.</p>"
    W, H, L, R, T, B = 680, 296, 52, 20, 20, 62
    n = len(pts)

    def X(i):
        return L + (W - L - R) * (i + 0.5) / n

    def Y(v):
        return T + (H - T - B) * (1 - v)
    s = ['<svg viewBox="0 0 %d %d" width="100%%" role="img" aria-label="Learning curve">' % (W, H)]
    for v in (0, .25, .5, .75, 1):
        s.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#ddd"/>' % (L, W - R, Y(v), Y(v)))
        s.append('<text x="%d" y="%.1f" font-size="11" fill="#666" text-anchor="end">%d%%</text>' % (L - 6, Y(v) + 4, v * 100))
    for key, col in (("recall", "#2e9d6d"), ("precision", "#2b7bbb"), ("f1", "#111")):
        ps = [(i, p) for i, p in enumerate(pts) if p.get(key) is not None]
        s.append('<polyline fill="none" stroke="%s" stroke-width="2" points="%s"/>'
                 % (col, " ".join("%.1f,%.1f" % (X(i), Y(p[key])) for i, p in ps)))
        for i, p in ps:
            s.append('<circle cx="%.1f" cy="%.1f" r="4" fill="%s"><title>%s: %s</title></circle>'
                     % (X(i), Y(p[key]), col, _html.escape(p["label"]), _pct(p[key])))
    for i, p in enumerate(pts):
        s.append('<text x="%.1f" y="%.1f" font-size="11" fill="#444" text-anchor="middle">%s</text>'
                 % (X(i), H - B + 16, _html.escape("no model" if p["model"] == "none" else p["model"])))
        s.append('<text x="%.1f" y="%.1f" font-size="10.5" fill="#777" text-anchor="middle">%d frames</text>'
                 % (X(i), H - B + 30, p["n_train_frames"]))
    s.append('<text x="%.1f" y="%d" font-size="12" fill="#666" text-anchor="middle">model versions, oldest to newest '
             '(under each: the confirmed frames it trained on)</text>' % ((L + W - R) / 2, H - 8))
    s.append("</svg>")
    s.append('<p class="key"><span style="color:#111">&#9632; F1 (overall)</span> <span style="color:#2e9d6d">&#9632; '
             'found (recall)</span> <span style="color:#2b7bbb">&#9632; right when it says so (precision)</span></p>')
    s.append('<p class="muted">Each point is one model version scored on the same held-back frames; higher is better, and '
             'a line that climbs from left to right means the labelling is paying off.</p>')
    return "".join(s)


def html_report(proj):
    curve_p = os.path.join(proj.folder, "models", "curve.json")
    curve = json.load(open(curve_p)) if os.path.exists(curve_p) else dict(points=[], test={})
    import trainer
    pts = sorted(curve.get("points", []), key=lambda p: trainer.version_order(p.get("model")))
    best = None
    for p in pts:
        if p["model"] != "none" and p.get("f1") is not None and (best is None or p["f1"] > best["f1"]):
            best = p
    base = next((p for p in pts if p["model"] == "none"), None)
    e = _html.escape
    out = ["<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>",
           "<title>%s – %s report</title>" % (e(proj.name), BRAND),
           "<style>body{font:15px/1.5 system-ui,-apple-system,'Segoe UI',Roboto,sans-serif;max-width:860px;margin:32px auto;"
           "padding:0 16px;color:#1b1d21;background:#fff}h1{font-size:24px;margin-bottom:4px}h2{font-size:18px;margin-top:28px}"
           "table{border-collapse:collapse;width:100%;font-size:14px}th,td{text-align:left;padding:5px 8px;border-bottom:1px solid #e3e3e3}"
           "th{color:#555;font-weight:600}.muted{color:#666;font-size:13px}.key span{margin-right:14px;font-size:13px}"
           ".lead{font-size:16px}.brand{font-size:13px;color:#555;letter-spacing:.02em;margin:0}"
           ".brand b{color:#1b1d21;font-size:15px;letter-spacing:0}.brand span{display:block;font-size:11px;color:#777}"
           ".caution{background:#fff4e0;border:1px solid #e8c48a;border-radius:6px;padding:8px 10px}"
           "</style></head><body>"]
    out.append("<p class='brand'><b>%s</b><span>%s</span></p>" % (BRAND, TAGLINE))
    out.append("<h1>%s: accuracy report</h1>" % e(proj.name))
    out.append("<p class='muted'>Updated %s · %s</p>" % (e(curve.get("updated", "–")), e(proj.folder)))
    if best and base:
        out.append("<p class='lead'>The best model, <b>%s</b>, trained on %d confirmed frames, finds <b>%s</b> of the objects "
                   "and is right <b>%s</b> of the time it reports one, on %d frames it never trained on. With no model "
                   "(motion and tracking only, where labelling starts) the same frames score %s found, %s right.</p>"
                   % (e(best["model"]), best["n_train_frames"], _pct(best["recall"]), _pct(best["precision"]),
                      curve.get("test", {}).get("frames", 0), _pct(base["recall"]), _pct(base["precision"])))
    elif best:
        out.append("<p class='lead'>The best model, <b>%s</b>, finds %s and is right %s of the time on held-back frames.</p>"
                   % (e(best["model"]), _pct(best["recall"]), _pct(best["precision"])))
    else:
        out.append("<p class='lead'>No model trained yet.</p>")

    out.append("<h2>Learning curve</h2>")
    out.append(_curve_svg(pts))
    note = curve.get("test", {}).get("note", "")
    out.append("<p class='muted'>Every model, and the starting point with no model, scored on the same held-back frames%s.</p>"
               % ((" (%s)" % e(note)) if note else ""))
    import trainer
    caution = trainer.score_caution(curve)
    if caution:
        out.append("<p class='caution'><b>Caution:</b> %s</p>" % e(caution))
    out.append("<table><tr><th>Guesses from</th><th>Trained on (frames)</th><th>Found</th><th>Right</th><th>F1</th>"
               "<th>Centre off (px)</th><th>Heading off, median (deg)</th><th>ms per frame</th></tr>")
    for p in pts:
        out.append("<tr><td>%s</td><td>%d</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>"
                   % (e(p["label"]), p["n_train_frames"], _pct(p["recall"]), _pct(p["precision"]), _pct(p["f1"]),
                      _num(p["centre_err_px"]), _num(p["heading_err_deg"]), _num(p["ms_per_frame"])))
    out.append("</table>")

    if best:
        ev = os.path.join(proj.folder, "models", best["model"], "eval_%s.csv" % best["model"])
        newest = sorted([m for m in proj.models() if m.get("status") == "ready"], key=lambda m: m["created"])
        if newest:
            ev = os.path.join(proj.model_dir(newest[-1]["id"]), "eval_%s.csv" % best["model"])
        if os.path.exists(ev):
            out.append("<h2>By object type (model %s)</h2><table><tr><th>Type</th><th>Objects</th><th>Labels</th>"
                       "<th>Found</th><th>Right</th><th>Centre off (px)</th><th>Heading off (deg)</th></tr>" % e(best["model"]))
            with open(ev, newline="") as f:
                for r in csv.DictReader(f):
                    out.append("<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>"
                               % (e(r["cls"]), _num(r["n_objects"], 0), _num(r["n_labels"], 0),
                                  _pct(_flt(r["recall"])), _pct(_flt(r["precision"])), _num(r["centre_err_px_mean"]),
                                  _num(r["heading_err_deg_median"])))
            out.append("</table><p class='muted'>Objects: different physical objects behind the labels (one object seen in "
                       "100 frames counts once). A type with few objects has an unreliable score.</p>")

    out.append(situations_html(proj, best))
    out.append(training_extras_html(proj))

    out.append(time_saved_html(proj))

    out.append("<h2>Data</h2><table><tr><th>Recording</th><th>Camera</th><th>Frames confirmed</th></tr>")
    for r in proj.recordings():
        out.append("<tr><td>%s</td><td>%s</td><td>%d</td></tr>" % (e(r["name"]), e(r.get("camera") or "–"),
                                                                   _done(proj.labels_dir(r["id"]))))
    out.append("</table>")
    out.append("<h2>Object types</h2><table><tr><th>Type</th><th>Has a front</th><th>Size (px)</th>"
               "<th>Height of the marked point above the floor (cm, for the second camera)</th></tr>")
    for c in proj.classes:
        out.append("<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>" % (
            e(c["name"]), "yes" if c.get("has_front") else "no", _num(c.get("size_px"), 0) if c.get("size_px") else "measured",
            "it varies" if c.get("height_varies") else _num(c.get("height_cm"))))
    out.append("</table>")
    out.append("<h2>How the scores are kept honest</h2><ul>"
               "<li>Labelled frames are cut into time blocks (%s s; shorter blocks only when the recordings are too "
               "short for 30 s blocks to give every part some frames). Each block goes wholly to training, to picking the "
               "best training round, or to scoring; frames within 1 s of a block edge are left out.</li>"
               "<li>A block's part depends only on its time, so adding labels never moves a scoring block into training, "
               "and the two cameras' views of one moment always land in the same part.</li>"
               "<li>Only frames a person checked are scored; frames accepted without looking are not. Frames holding "
               "an object marked unsure are left out of training and scoring until it is resolved.</li>"
               "<li>Guesses a person rejected are taught as “not an object here”, and copy-paste adds objects only to "
               "frames used for learning: never to the frames used to pick the best round or to score.</li>"
               "<li>“No model” is what the labelling page shows before any training: motion guesses followed by the "
               "tracker, with no confirmations.</li>"
               "<li>Found = share of labelled objects the model reported within 8 px (about a quarter of an object). "
               "Right = share of the model's reports that matched a label. F1 combines the two: 2 × found × right / (found + right).</li>"
               "</ul>" % _num(_block_s(proj), 0))
    out.append("</body></html>")
    return "".join(out)


SITUATION_WORDS = (
    "Alone: no other labelled object within 1.5 object sizes. Close together: another object within 1.5 sizes. "
    "Near a person: a person label (a hand, an arm) within 1.5 sizes. At the edge: within 1 size of the picture's "
    "border or of the work area's border, or outside it. Standing still: the same object did not move over the "
    "last 2 seconds of labelled frames (counted only where labelled frames cover those 2 seconds).")


def _situation_rows(proj):
    p = os.path.join(proj.folder, "models", "situations.json")
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


# each situation as words that fit "The model does worst with ..."
SITUATION_PHRASES = dict(alone="objects on their own", close="objects close together", person="objects near a person",
                         edge="objects at the edge of the work area or picture", still="objects standing still")


def situations_html(proj, best):
    """Accuracy by situation for the best model (and, beside it, the start with no model), on the
    held-back frames. Groups that rest on few labels are marked."""
    data = _situation_rows(proj)
    if not data or not best or best["model"] not in data.get("models", {}):
        return ""
    import situations
    e = _html.escape
    rows = data["models"][best["model"]]
    base = {r["situation"]: r for r in data["models"].get("none", [])}
    out = ["<h2>By situation (model %s)</h2>" % e(best["model"])]
    shown = [r for r in rows if r["situation"] != "all"]
    ok = [r for r in shown if r["n_labels"] and not r["few"] and r["found"] is not None]
    if ok:
        w = min(ok, key=lambda r: r["found"])
        out.append("<p class='lead'>The model does worst with <b>%s</b>: it finds %s of %d such labels.</p>"
                   % (e(SITUATION_PHRASES.get(w["situation"], w["name"].lower())), _pct(w["found"]), w["n_labels"]))
    else:
        out.append("<p class='lead'>No situation has enough labels yet (%d labels and %d different objects needed) to "
                   "say where the model is weakest.</p>" % (situations.MIN_LABELS, situations.MIN_OBJECTS))
    out.append("<table><tr><th>Situation</th><th>Labels</th><th>Different objects</th><th>Found</th>"
               "<th>Found with no model</th><th>Centre off (px)</th><th>Heading off, median (deg)</th><th></th></tr>")
    for r in shown + [x for x in rows if x["situation"] == "all"]:
        b = base.get(r["situation"])
        note = ("Too few to trust: under %d labels or %d different objects" % (situations.MIN_LABELS, situations.MIN_OBJECTS)
                if r["few"] and r["n_labels"] else ("None in the held-back frames" if not r["n_labels"] else ""))
        out.append("<tr><td>%s</td><td>%d</td><td>%d</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td class='muted'>%s</td></tr>"
                   % (e(r["name"]), r["n_labels"], r["n_objects"], _pct(r["found"]) if r["n_labels"] else "–",
                      _pct(b["found"]) if b and b["n_labels"] else "–", _num(r["centre_err_px"]), _num(r["heading_err_deg"]),
                      e(note)))
    out.append("</table><p class='muted'>Found = share of labelled objects in that situation that the model reported within "
               "8 px. One object can be in several groups (for example close together and at the edge), so the groups "
               "do not add up to all labels. %s</p>" % e(SITUATION_WORDS))
    return "".join(out)


def training_extras_html(proj):
    """What the newest ready model's training used: rejected guesses as 'not an object' examples, and the
    copy-paste comparison (both scores, and which is better)."""
    e = _html.escape
    ready = sorted([m for m in proj.models() if m.get("status") == "ready"], key=lambda m: m["created"])
    if not ready:
        return ""
    m = ready[-1]
    out = []
    hn = m.get("hard_negatives")
    if hn and hn.get("spots"):
        out.append("<li>Model %s learned from <b>%d guesses you rejected</b>, in %d frames, as “not an object here”%s.</li>"
                   % (e(m["id"]), hn["spots"], hn["frames"],
                      (" (%d of those frames were used only for that)" % hn["frames_only"]) if hn.get("frames_only") else ""))
    cp = m.get("copy_paste")
    if cp and cp.get("compared"):
        a, b = cp["without"], cp["with"]
        out.append("<li><b>Copy-paste (objects you labelled pasted onto other learning frames):</b> %s</li>" % e(cp["text"]))
        rows = ("<table><tr><th>Trained</th><th>Found</th><th>Right</th><th>F1</th><th>Centre off (px)</th></tr>"
                "<tr><td>Without copy-paste%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>"
                "<tr><td>With copy-paste%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr></table>"
                % (" (kept)" if not cp["used"] else "", _pct(a.get("recall")), _pct(a.get("precision")), _pct(a.get("f1")),
                   _num(a.get("centre_err_px")), " (kept)" if cp["used"] else "", _pct(b.get("recall")),
                   _pct(b.get("precision")), _pct(b.get("f1")), _num(b.get("centre_err_px"))))
        if cp.get("decided_on") == "validation":
            t = cp.get("test") or {}
            how = ("Both scored on the same frames used for picking the best training round (that is where the choice "
                   "is made); only the kept one was then scored on the held-back frames: found %s, right %s, F1 %s. "
                   "Each was trained for up to %s minutes." % (_pct(t.get("recall")), _pct(t.get("precision")),
                                                              _pct(t.get("f1")), _num(cp.get("minutes_each"), 0)))
        else:       # models compared before this was changed: both were scored on the held-back frames
            how = ("Both scored on the same %s held-back labels; each was trained for up to %s minutes."
                   % (_num(b.get("n_labels"), 0), _num(cp.get("minutes_each"), 0)))
        out.append("<li style='list-style:none'>%s<span class='muted'>%s</span></li>" % (rows, how))
    elif cp:
        out.append("<li>%s</li>" % e(cp["text"]))
    if not out:
        return ""
    return "<h2>What training used</h2><ul>%s</ul>" % "".join(out)


def _dur(s):
    """Seconds in plain words."""
    if s is None:
        return "–"
    s = float(s)
    if s < 90:
        return "%d s" % round(s)
    if s < 5400:
        return "%d min" % round(s / 60.0)
    return "%.1f h" % (s / 3600.0)


def _block_s(proj):
    """The block length the last training used (models/curve.json), else the project's setting."""
    try:
        with open(os.path.join(proj.folder, "models", "curve.json"), encoding="utf-8") as f:
            v = json.load(f).get("test", {}).get("block_s")
        if v:
            return v
    except (OSError, ValueError):
        pass
    return proj.data.get("settings", {}).get("block_s", 30)


def _model_name(m, guesses=""):
    name = {"none": "No model (motion + tracking)", "unknown": "Not known"}.get(m, "Model " + m)
    if guesses and "+camera" in guesses:
        name += " + the other camera"
    return name


def time_saved_html(proj, summary=None):
    """The time-saving section: seconds per frame by guess source (measured frames only; the
    estimated ones in their own rows), and frames accepted without looking as a count."""
    e = _html.escape
    sm = summary or timesaved.summary(proj)
    t, rows = sm["totals"], sm["by_model"]
    out = ["<h2>Time saved</h2>"]
    if not rows:
        out.append("<p>No frames finished yet.</p>")
        return "".join(out)
    lead = []
    meas = [r for r in rows if r.get("measured", True)]
    base = next((r for r in meas if r["model"] == "none" and r["seconds_per_frame"] is not None), None)
    best = [r for r in meas if r["model"] not in ("none", "unknown") and r["seconds_per_frame"] is not None and r["frames"]]
    if base and best:
        last = best[-1]
        lead.append("Measured: with no model a frame took %s; with model %s it took %s (%d%% accepted unchanged)."
                    % (_dur(base["seconds_per_frame"]), e(last["model"]), _dur(last["seconds_per_frame"]),
                       round(100 * (last["accepted_unchanged"] or 0))))
    n = t["frames_by_person"]
    lead.append("%d frame%s checked by a person (%s in all)." % (n, " was" if n == 1 else "s were", _dur(t["seconds_by_person"])))
    if t["auto"] or t["same"]:
        lead.append("%d more %s accepted without a person looking, and %d for having the same picture as a frame a "
                    "person accepted. These are not counted as time saved." % (t["auto"], "was" if t["auto"] == 1 else "were",
                                                                                t["same"]))
        if t.get("estimate_if_checked_s"):
            lead.append("Estimate only: checking them at the measured no-model pace (%s a frame) would have taken about "
                        "%s; they were the easier frames, so the true figure is likely lower."
                        % (_dur(t["baseline_seconds_per_frame"]), _dur(t["estimate_if_checked_s"])))
    out.append("<p class='lead'>%s</p>" % " ".join(lead))
    out.append("<p class='muted'>The page shows the hardest frames first (the least sure guesses), so later frames "
               "are usually easier. Seconds per frame before and after a model are only a fair comparison on similar "
               "frames: a drop is partly the model and partly easier frames.</p>")
    mx = max([r["seconds_per_frame"] or 0 for r in meas] + [1])
    # bars only when there is a real comparison (three or more measured guess sources with a time)
    bars = sum(1 for r in meas if r["seconds_per_frame"] is not None) >= 3
    out.append("<table><tr><th>Guesses from</th><th>Timing</th><th>Frames checked by a person</th><th>Seconds per frame</th>"
               "<th>Accepted unchanged</th><th>Changes per frame</th><th>Accepted without looking</th>"
               "<th>Same picture</th></tr>")
    for r in sorted(rows, key=lambda r: not r.get("measured", True)):
        measured = r.get("measured", True)
        bar = ""
        if bars and measured and r["seconds_per_frame"] is not None:
            bar = ("<span style='display:inline-block;height:9px;background:#2b7bbb;margin-left:8px;vertical-align:0;"
                   "width:%dpx'></span>" % max(2, round(90 * r["seconds_per_frame"] / mx)))
        out.append("<tr><td>%s</td><td>%s</td><td>%d</td><td>%s%s</td><td>%s</td><td>%s</td><td>%d</td><td>%d</td></tr>" % (
            e(_model_name(r["model"], r.get("guesses") or "")), "measured" if measured else "estimated",
            r["frames"], _num(r["seconds_per_frame"]), bar, _pct(r["accepted_unchanged"]),
            _num(r["edits_per_frame"]), r["auto"], r["same"]))
    out.append("</table>")
    out.append("<p class='muted'>%sMeasured seconds count the time the frame was on screen, with "
               "pauses over %d s counted as %d s and at most %d s per frame. Rows marked estimated are frames finished "
               "before this record existed, estimated from their save times (the gap to the previous saved frame; a gap "
               "over %d s is a break and gets the usual gap instead); they are never mixed into the measured rows, and "
               "which model was in use then is not known after the first model was trained.</p>"
               % ("Each bar is the average seconds a person spent on one frame with those guesses (measured rows only): "
                  "shorter means the guesses needed less fixing. " if bars else "", timesaved.timing.IDLE_S,
                  timesaved.timing.IDLE_S, timesaved.timing.CAP_S, timesaved.timing.CAP_S))
    extra = []
    if t.get("setup_seconds") is not None:
        extra.append("project setup %s" % _dur(t["setup_seconds"]))
    if t.get("install_seconds") is not None:
        extra.append("first install %s" % _dur(t["install_seconds"]))
    if extra:
        out.append("<p class='muted'>Getting started: %s.</p>" % ", ".join(extra))
    out.append("<p class='muted'>The numbers behind this table: the studio's Time saved panel (CSV), or "
               "<code>python tools/recognition/timesaved.py &lt;project folder&gt;</code>.</p>")
    return "".join(out)


def _flt(v):
    try:
        x = float(v)
        return None if x != x else x
    except (TypeError, ValueError):
        return None


def _done(label_dir):
    p = os.path.join(label_dir, "review.json")
    if not os.path.exists(p):
        return 0
    try:
        fr = json.load(open(p)).get("frames", {})
    except ValueError:
        return 0
    return sum(1 for v in fr.values() if v.get("state") in ("reviewed", "auto"))




def html(proj):          # noqa: F811  (the studio calls report.html(project))
    return html_report(proj)


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    proj = projmod.Project(sys.argv[1])
    out = os.path.join(proj.folder, "report.html")
    with open(out, "w", encoding="utf-8") as f:
        f.write(html_report(proj))
    print("Wrote " + out)


if __name__ == "__main__":
    main()
