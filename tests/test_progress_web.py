"""emu/tools/progress_web.py: the status of a runner manifest and the page that shows it."""
import json
import os
import sys
import threading
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "emu", "tools"))
import progress_web as P  # noqa: E402


def _manifest():
    return {"name": "q", "runs": [
        {"id": "a", "status": "done", "t_begin": 0, "t_done": 200, "seconds": 200, "slot": 0,
         "lag": {"p99_us": 150, "max_us": 400, "steps_over_1ms": 0.0}, "lagging": False, "attempts": 1},
        {"id": "b", "status": "done", "t_begin": 0, "t_done": 300, "seconds": 300, "slot": 1,
         "lag": {"p99_us": 450, "max_us": 1500, "steps_over_1ms": 0.02}, "lagging": True, "attempts": 1},
        {"id": "c", "status": "running", "t_begin": 900, "slot": 0, "M": 4, "scenario": "S1"},
        {"id": "d", "status": "pending"}, {"id": "e", "status": "pending"},
        {"id": "f", "status": "failed", "t_done": 100, "seconds": 50, "attempts": 1}]}


def test_summarize_counts_eta_and_lag():
    s = P.summarize(_manifest(), now=1000)
    assert s["total"] == 6 and s["counts"]["pending"] == 2 and s["counts"]["running"] == 1
    assert s["complete"] == 3 and abs(s["fraction"] - 0.5) < 1e-9
    assert s["running"][0]["elapsed_s"] == 100
    assert [r["id"] for r in s["recent"]] == ["b", "a", "f"]
    assert s["lag"] == {"runs": 2, "lagging": 1, "p99_median_us": 450, "p99_max_us": 450}
    # mean duration (200 + 300 + 50) / 3 times 3 runs left, over the 1 slot in use
    assert abs(s["eta_s"] - round((550 / 3) * 3, 0)) < 1e-9


def test_page_and_api(tmp_path):
    d = tmp_path / "q"
    d.mkdir()
    (d / "manifest.json").write_text(json.dumps(_manifest()))
    log = tmp_path / "x.log"
    log.write_text("line 1\nline 2\n")
    cfg = {"manifests": [str(d)], "logs": [("x", str(log))], "disk": str(tmp_path)}
    srv = P.ThreadingHTTPServer(("127.0.0.1", 0), P.make_handler(cfg))
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        base = "http://127.0.0.1:%d" % srv.server_address[1]
        page = urllib.request.urlopen(base + "/").read().decode()
        assert "api/status" in page and "<title>Fleet-DT campaign</title>" in page
        st = json.loads(urllib.request.urlopen(base + "/api/status").read())
        assert st["experiments"][0]["counts"]["done"] == 2
        assert st["logs"][0]["lines"] == ["line 1", "line 2"]
        assert "load" in st["host"] and "disk_free_gb" in st["host"]
    finally:
        srv.shutdown()
