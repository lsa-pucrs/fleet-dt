#!/usr/bin/env python3
"""Read-only web page with the progress of the campaign of Section VI on the emulation host.

  python3 emu/tools/progress_web.py --manifest DIR [--manifest DIR2 ...] [--log NAME=PATH ...]
                                    [--bind 127.0.0.1] [--port 8765] [--disk PATH]

The page (/) reloads /api/status every 10 s. /api/status reads, at each request, the manifest.json of every
--manifest directory (emu/experiments/runner.py writes it atomically): the runs per status, the runs in
progress with their slot and elapsed time, the last finished runs with their duration and the lag of the
emulator (p99, maximum and share of the steps above 1 ms, from runner.lag_verdict), and an estimate of the
remaining time (mean duration of the finished runs times the pending runs, divided by the slots in use). It
also reports the load of the host (load average, available memory, free disk of --disk, the number of QEMU
processes) and the last lines of each --log file. The page writes nothing and starts nothing.
"""
import argparse
import json
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

STATUSES = ("pending", "running", "done", "failed", "invalid", "aborted", "skipped")


def read_manifest(d):
    p = os.path.join(d, "manifest.json")
    with open(p) as f:
        return json.load(f), os.path.getmtime(p)


def summarize(m, now=None):
    """Status of one manifest: counts, runs in progress, last finished runs, lag summary, ETA."""
    now = time.time() if now is None else now
    runs = m.get("runs", [])
    counts = {s: 0 for s in STATUSES}
    for r in runs:
        counts[r.get("status", "pending")] = counts.get(r.get("status", "pending"), 0) + 1
    running = [{"id": r["id"], "slot": r.get("slot"), "M": r.get("M"), "scenario": r.get("scenario"),
                "elapsed_s": round(now - r["t_begin"], 0) if r.get("t_begin") else None}
               for r in runs if r.get("status") == "running"]
    finished = [r for r in runs if r.get("t_done")]
    finished.sort(key=lambda r: r["t_done"], reverse=True)
    recent = [{"id": r["id"], "status": r.get("status"), "slot": r.get("slot"), "seconds": r.get("seconds"),
               "t_done": r.get("t_done"), "lagging": r.get("lagging"),
               "p99_us": (r.get("lag") or {}).get("p99_us"), "max_us": (r.get("lag") or {}).get("max_us"),
               "steps_over_1ms": (r.get("lag") or {}).get("steps_over_1ms"),
               "attempts": r.get("attempts")} for r in finished[:20]]
    durs = [r["seconds"] for r in finished if isinstance(r.get("seconds"), (int, float)) and r["seconds"] > 0]
    p99 = sorted(x["p99_us"] for x in (
        {"p99_us": (r.get("lag") or {}).get("p99_us")} for r in finished) if isinstance(x["p99_us"], (int, float)))
    lag = {"runs": len(p99), "lagging": sum(1 for r in finished if r.get("lagging")),
           "p99_median_us": p99[len(p99) // 2] if p99 else None, "p99_max_us": p99[-1] if p99 else None}
    slots = max(1, len(running))
    left = counts.get("pending", 0) + counts.get("running", 0)
    eta = (sum(durs) / len(durs)) * left / slots if durs else None
    total = len(runs)
    complete = counts.get("done", 0) + counts.get("failed", 0) + counts.get("invalid", 0) + \
        counts.get("aborted", 0) + counts.get("skipped", 0)
    return {"name": m.get("name", "?"), "total": total, "counts": counts, "complete": complete,
            "fraction": complete / total if total else 0.0, "running": running, "recent": recent, "lag": lag,
            "mean_run_s": round(sum(durs) / len(durs), 1) if durs else None,
            "eta_s": round(eta, 0) if eta is not None else None}


def host_status(disk):
    out = {}
    try:
        out["load"] = [round(x, 2) for x in os.getloadavg()]
    except OSError:
        out["load"] = None
    try:
        with open("/proc/meminfo") as f:
            mi = {l.split(":")[0]: int(l.split()[1]) for l in f if ":" in l}
        out["mem_available_gb"] = round(mi.get("MemAvailable", 0) / 1048576, 1)
        out["mem_total_gb"] = round(mi.get("MemTotal", 0) / 1048576, 1)
    except (OSError, ValueError):
        pass
    try:
        st = os.statvfs(disk)
        out["disk_free_gb"] = round(st.f_bavail * st.f_frsize / 1e9, 1)
    except OSError:
        out["disk_free_gb"] = None
    n = 0
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                with open("/proc/%s/comm" % p) as f:
                    n += f.read().startswith("qemu-system")
            except OSError:
                pass
    out["qemu_processes"] = n
    out["cpus"] = os.cpu_count()
    return out


def tail(path, n=12):
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 8192))
            lines = f.read().decode("utf-8", "replace").splitlines()[-n:]
        return {"lines": lines, "mtime": os.path.getmtime(path)}
    except OSError as e:
        return {"lines": ["(%s)" % e.strerror], "mtime": None}


def status(cfg):
    now = time.time()
    exps = []
    for d in cfg["manifests"]:
        try:
            m, mt = read_manifest(d)
            s = summarize(m, now)
            s["dir"] = d
            s["updated"] = mt
        except (OSError, ValueError) as e:
            s = {"name": os.path.basename(d.rstrip("/")), "dir": d, "error": str(e)}
        exps.append(s)
    logs = [{"name": k, **tail(v)} for k, v in cfg["logs"]]
    return {"now": now, "host": host_status(cfg["disk"]), "experiments": exps, "logs": logs}


PAGE = """<!doctype html><html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Fleet-DT campaign</title>
<style>
:root{--bg:#fbfbfa;--fg:#1d1d1b;--mut:#6b6b66;--line:#e2e1dc;--bar:#2b6cb0;--ok:#2f855a;--bad:#c53030;--warn:#b7791f;--card:#fff}
@media (prefers-color-scheme:dark){:root{--bg:#161615;--fg:#ecebe6;--mut:#a2a19b;--line:#33332f;--bar:#63a4e8;--ok:#68d391;--bad:#fc8181;--warn:#f6c453;--card:#1f1f1d}}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,sans-serif}
main{max-width:1100px;margin:0 auto;padding:16px}
h1{font-size:20px;margin:0 0 4px} h2{font-size:16px;margin:24px 0 8px}
.mut{color:var(--mut)} .card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:12px;margin:8px 0}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:8px}
.kv b{display:block;font-size:20px;font-variant-numeric:tabular-nums}
.bar{height:10px;background:var(--line);border-radius:5px;overflow:hidden;margin:8px 0}
.bar i{display:block;height:100%;background:var(--bar)}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
th,td{text-align:left;padding:4px 6px;border-bottom:1px solid var(--line);white-space:nowrap}
.scroll{overflow-x:auto} .ok{color:var(--ok)} .bad{color:var(--bad)} .warn{color:var(--warn)}
pre{margin:0;font-size:12px;overflow-x:auto;white-space:pre-wrap}
</style></head><body><main>
<h1>Fleet-DT: progresso da campanha</h1><div class="mut" id="stamp">carregando</div>
<div id="host" class="card grid"></div><div id="exps"></div><h2>Logs</h2><div id="logs"></div>
</main><script>
const fmtS=s=>{if(s==null)return"—";s=Math.round(s);const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h?`${h} h ${m} min`:`${m} min ${s%60} s`};
const esc=t=>String(t??"—").replace(/[&<>]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;"}[c]));
const kv=(k,v)=>`<div class="kv"><span class="mut">${k}</span><b>${v}</b></div>`;
async function load(){
 try{const r=await fetch("api/status",{cache:"no-store"});const d=await r.json();
 document.getElementById("stamp").textContent="Atualizado em "+new Date(d.now*1000).toLocaleString("pt-BR");
 const h=d.host;document.getElementById("host").innerHTML=kv("Carga (1/5/15 min)",(h.load||[]).join(" / ")+` <span class="mut">de ${h.cpus} CPUs</span>`)+kv("Memória livre",`${h.mem_available_gb} de ${h.mem_total_gb} GB`)+kv("Disco livre",`${h.disk_free_gb} GB`)+kv("VMs (QEMU)",h.qemu_processes);
 document.getElementById("exps").innerHTML=d.experiments.map(e=>{if(e.error)return`<h2>${esc(e.name)}</h2><div class="card bad">${esc(e.error)}</div>`;
  const c=e.counts,pct=(100*e.fraction).toFixed(1);
  const run=e.running.length?`<div class="scroll"><table><tr><th>Em execução</th><th>Slot</th><th>M</th><th>Cenário</th><th>Tempo</th></tr>${e.running.map(r=>`<tr><td>${esc(r.id)}</td><td>${esc(r.slot)}</td><td>${esc(r.M)}</td><td>${esc(r.scenario)}</td><td>${fmtS(r.elapsed_s)}</td></tr>`).join("")}</table></div>`:"";
  const rec=e.recent.length?`<div class="scroll"><table><tr><th>Concluído</th><th>Status</th><th>Duração</th><th>Atraso p99</th><th>Atraso máx.</th><th>Passos &gt; 1 ms</th><th>Tentativas</th></tr>${e.recent.map(r=>`<tr><td>${esc(r.id)}</td><td class="${r.status==="done"&&!r.lagging?"ok":(r.status==="pending"?"warn":"bad")}">${esc(r.status)}${r.lagging?" (atraso)":""}</td><td>${fmtS(r.seconds)}</td><td>${r.p99_us==null?"—":Math.round(r.p99_us)+" µs"}</td><td>${r.max_us==null?"—":Math.round(r.max_us)+" µs"}</td><td>${r.steps_over_1ms==null?"—":(100*r.steps_over_1ms).toFixed(2)+"%"}</td><td>${esc(r.attempts)}</td></tr>`).join("")}</table></div>`:`<div class="mut">Nenhum run concluído ainda.</div>`;
  return`<h2>${esc(e.name)}</h2><div class="card"><div class="grid">${kv("Concluídos",`${e.complete} de ${e.total} (${pct}%)`)+kv("Pendentes",c.pending)+kv("Em execução",c.running)+kv("Falhas",(c.failed||0)+(c.aborted||0)+(c.invalid||0))+kv("Com atraso",`${e.lag.lagging} de ${e.lag.runs}`)+kv("Atraso p99 (mediana)",e.lag.p99_median_us==null?"—":Math.round(e.lag.p99_median_us)+" µs")+kv("Duração média",fmtS(e.mean_run_s))+kv("Tempo restante (estim.)",fmtS(e.eta_s))}</div><div class="bar"><i style="width:${pct}%"></i></div>${run}${rec}</div>`}).join("");
 document.getElementById("logs").innerHTML=d.logs.map(l=>`<div class="card"><b>${esc(l.name)}</b> <span class="mut">${l.mtime?new Date(l.mtime*1000).toLocaleString("pt-BR"):""}</span><pre>${esc(l.lines.join("\\n"))}</pre></div>`).join("");
 }catch(err){document.getElementById("stamp").textContent="Falha ao ler o status: "+err}}
load();setInterval(load,10000);
</script></body></html>"""


def make_handler(cfg):
    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.split("?")[0] in ("/", "/index.html"):
                body, ctype = PAGE.encode(), "text/html; charset=utf-8"
            elif self.path.split("?")[0] == "/api/status":
                body, ctype = json.dumps(status(cfg)).encode(), "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass
    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--manifest", action="append", default=[], help="runner directory with manifest.json")
    ap.add_argument("--log", action="append", default=[], help="NAME=PATH of a log whose last lines the page shows")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--disk", default=os.path.expanduser("~"))
    a = ap.parse_args()
    logs = []
    for x in a.log:
        k, _, v = x.partition("=")
        logs.append((k, os.path.expanduser(v)))
    cfg = {"manifests": [os.path.expanduser(d) for d in a.manifest], "logs": logs, "disk": a.disk}
    srv = ThreadingHTTPServer((a.bind, a.port), make_handler(cfg))
    print("progress page on http://%s:%d/" % (a.bind, a.port), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
