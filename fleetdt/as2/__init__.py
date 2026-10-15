"""Fleet-DT v2, ground-station side: application space (AS), operation-control stub and analysis.

Modules (paper Sections III to V, v2; interfaces in SPEC_v2.md):
  dtp        DTP v2 loader and validation (SPEC Section 2), step clock (SPEC Section 3)
  airtime    eq. (tsucc), eq. (cost), cost of held bytes
  bianchi    eq. (bianchi), eq. (eff)
  monitor    fleet/monitor batches -> b(t), e(t), kappa(t), n(t), R_i(t), R_0i(t)
  predictor  eq. (pred), scores and Z_t(alpha)
  aci        eq. (aci) and F_R
  admission  order pi_t, Algorithm 1, B-bar, tables, slotted variant
  failure    accepted messages of boat/+/tx/# and f(t) (Def. aci)
  core       one step of the AS without MQTT (phase 2 of step t)
  service    the AS process (MQTT, phase timers, DTA log)
  opctl      operation-control stub (fleet/order, goals)
  analysis   fidelity metrics (Def. fidelity), Theorem 1 check
  whatif     what-if evaluation (Def. whatif)
"""
