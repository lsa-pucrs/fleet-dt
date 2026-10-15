#!/bin/bash
# Inside the VM, after provision.sh: brings up the simulated network, runs a scenario and checks
# the result. Arguments after OUT go to scenario.py (default: 90 s, one burst of 30 s at duty 0.7).
#   tests/hwsim/run.sh OUT [scenario options]
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
W=${FDT_HWSIM_WORK:-$HOME/fdt-hwsim}
OUT=$(realpath -m "$1"); shift
[ $# -gt 0 ] || set -- --seconds 90 --plan "30 30 0.7"
sudo "$ROOT/tests/hwsim/topology.sh" up "$W/hwsim/mac80211_hwsim.ko" "$W/wmediumd/wmediumd/wmediumd"
trap 'sudo "$ROOT/tests/hwsim/topology.sh" down' EXIT
sudo rm -rf "$OUT"
sudo ip netns exec gs python3 "$ROOT/tests/hwsim/scenario.py" --out "$OUT" "$@" > /dev/null
sudo chown -R "$(id -u):$(id -g)" "$OUT"
python3 - "$OUT/summary.json" <<'PY'
import json, sys
s = json.load(open(sys.argv[1]))
c, b = s["clean"], s["burst"]
checks = {
    "Algorithm 1 reproduced from every G(t)": s["checked"] > 0 and s["alg1_ok"] == s["checked"],
    "r <= d on every edge": s["policing_ok"] == s["checked"],
    "M(t) full in 95% of the clean steps": c["m_full"] >= 0.95 * c["steps"],
    "y(t) drops during the bursts": b["steps"] == 0 or b["y_mean"] < 0.6 * c["y_mean"],
    "video admitted in 90% of the clean steps": c["video_admitted"] >= 0.9 * c["steps"],
    "video withheld in 90% of the burst steps": b["steps"] == 0 or b["video_admitted"] <= 0.1 * b["steps"],
    "delta >= 0.95": all(x["metrics"].get("delta", 1) >= 0.95 for x in (c, b) if x["steps"]),
    "mu <= 0.05": all(x["metrics"].get("mu", 0) <= 0.05 for x in (c, b) if x["steps"]),
}
for k, v in checks.items():
    print("%-45s %s" % (k, "ok" if v else "FAILED"))
print(json.dumps({k: {m: s[k][m] for m in ("steps", "m_full", "y_mean", "C_hat_mean", "video_admitted", "metrics")}
                  for k in ("clean", "burst")}, indent=1))
sys.exit(0 if all(checks.values()) else 1)
PY
