# Fleet-DT B — Transporte e QoS (§III) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implementar a camada de rede do §III — codec de fio, envelope com número de sequência, interface de transporte, adaptador MQTT com brokers em bridge, os **reguladores de banda** e o **modelo de link budget** — de modo que o abstract passe a ter artefato.

**Architecture:** O núcleo (Plano A) não conhece rede. Esta camada é uma casca: codec puro sem I/O, um envelope que carrega o que a Tabela I não carrega, uma interface `fdt_transport_t` de três ponteiros de função, e duas implementações — loopback em processo para teste e mosquitto para produção. Reguladores e link budget são aritmética pura, testável sem broker.

**Tech Stack:** C18, `gcc`, `make`. `libmosquitto` **apenas** em `adapters/mqtt/`, atrás de `pkg-config`; a lib base segue sem dependência externa. `mosquitto` 2.x para o teste de integração.

**Spec:** [`docs/spec/paper-claims.md`](../../spec/paper-claims.md)
**Depende de:** [Plano A](2026-08-17-fleet-dt-a-model-core.md), tasks 1–8.

## Global Constraints

- Herda todas as constraints do Plano A.
- **Timestamp mora no envelope, nunca no modelo.** A Tabela I não lista nenhum (Armadilha 4 do spec). `fdt_input_t` e `fdt_state_t` continuam sem campo de tempo.
- A patologia do §V-B é **detectada e contada, nunca corrigida**. Descartar pacote atrasado é o item **D6**, declarado futuro pelo §VI; implementá-lo desalinha.
- `make lib` e `make test` continuam funcionando **sem** libmosquitto instalada. O adaptador entra por `make mqtt`, alvo separado.
- Todo campo de fio é little-endian explícito e largura fixa. Nada de `memcpy` de struct para a rede.

---

### Task 1: Codec de fio

**Files:**
- Create: `include/fleet_dt/codec.h`, `src/codec.c`
- Test: `tests/test_codec.c`

**Interfaces:**
- Produces: `FDT_WIRE_STATE_BYTES` (48), `FDT_WIRE_INPUT_BYTES` (**78**), `FDT_WIRE_ACT_BYTES` (8), `FDT_VIEW_PRESENT`; `fdt_enc_state(const fdt_state_t*, uint8_t *buf, size_t cap) -> ssize_t`; `fdt_dec_state(const uint8_t *buf, size_t len, fdt_state_t*) -> ssize_t`; e os pares `fdt_enc_input`/`fdt_dec_input`, `fdt_enc_act`/`fdt_dec_act`. Cada um retorna bytes consumidos ou `-1`.

Decomposição de `FDT_WIRE_INPUT_BYTES`, uma vez e só aqui: a Tabela I lista 21
entradas; duas delas são as vistas de câmera, que **não vão no fio** — são
ponteiros opacos, e o §III as tira do MQTT para o RTSP (C11). Restam 19
escalares × 4 bytes = 76, mais 1 byte de flag de presença por vista = **78**.
O receptor reamarra as imagens pelo caminho HSDT.

`FDT_VIEW_PRESENT` é `((const void *)(uintptr_t)1)`: sentinela que marca "havia
uma vista aqui", nunca desreferenciada.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_codec.c`:

```c
#include <fleet_dt/codec.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

int main(void)
{
    /* O estado no fio tem a mesma largura que na memória: os 48 bytes do §IV.
     * Coincidência que vale checar, porque as duas larguras têm causas
     * diferentes — uma é a eq. (1), a outra é o formato. */
    assert(FDT_WIRE_STATE_BYTES == 48);
    assert(FDT_WIRE_INPUT_BYTES == 78);
    assert(FDT_WIRE_ACT_BYTES == 8);

    fdt_state_t b = { .lat_deg = -30.05f, .lon_deg = -51.17f, .alt_m = 12.5f,
                      .roll_deg = 1.5f, .pitch_deg = -2.5f, .yaw_deg = 91.25f,
                      .surge_mps = 1.25f, .sway_mps = -0.5f, .heave_mps = 0.0f,
                      .roll_rate_rps = 0.1f, .pitch_rate_rps = -0.2f,
                      .yaw_rate_rps = 0.3f };
    uint8_t buf[128];

    assert(fdt_enc_state(&b, buf, sizeof buf) == FDT_WIRE_STATE_BYTES);
    assert(fdt_enc_state(&b, buf, 4) == -1);        /* buffer curto */

    /* Little-endian explícito, checado pelo padrão de bytes e não pelo
     * round-trip: 1.0f em IEEE-754 é 0x3F800000, que no fio sai como
     * 00 00 80 3F. Um codec que fizesse memcpy do struct passaria no
     * round-trip e falharia aqui num host big-endian. */
    fdt_state_t one = { .lat_deg = 1.0f };
    uint8_t le[FDT_WIRE_STATE_BYTES];
    assert(fdt_enc_state(&one, le, sizeof le) == FDT_WIRE_STATE_BYTES);
    assert(le[0] == 0x00 && le[1] == 0x00 && le[2] == 0x80 && le[3] == 0x3F);

    fdt_state_t out = {0};
    assert(fdt_dec_state(buf, FDT_WIRE_STATE_BYTES, &out) == FDT_WIRE_STATE_BYTES);
    assert(fdt_dec_state(buf, 4, &out) == -1);
    assert(memcmp(&b, &out, sizeof b) == 0);        /* round-trip exato */

    fdt_input_t in = {0};
    in.ax_mps2 = 1.0f; in.ibat_a = 9.5f; in.x_left = buf; in.x_right = NULL;
    assert(fdt_enc_input(&in, buf, sizeof buf) == FDT_WIRE_INPUT_BYTES);
    fdt_input_t in_out = {0};
    assert(fdt_dec_input(buf, FDT_WIRE_INPUT_BYTES, &in_out) == FDT_WIRE_INPUT_BYTES);
    assert(in_out.ax_mps2 == 1.0f && in_out.ibat_a == 9.5f);
    /* As vistas não trafegam: sobrevivem como flag de presença, não como
     * imagem. O §III as tira do MQTT e as manda pelo RTSP. */
    assert(in_out.x_left == FDT_VIEW_PRESENT);
    assert(in_out.x_right == NULL);

    fdt_actuation_t a = { .throttle_pct = 60.0f, .cage_rad = -0.25f };
    assert(fdt_enc_act(&a, buf, sizeof buf) == FDT_WIRE_ACT_BYTES);
    fdt_actuation_t a_out = {0};
    assert(fdt_dec_act(buf, FDT_WIRE_ACT_BYTES, &a_out) == FDT_WIRE_ACT_BYTES);
    assert(memcmp(&a, &a_out, sizeof a) == 0);

    printf("wire: state=%d input=%d act=%d bytes\n",
           FDT_WIRE_STATE_BYTES, FDT_WIRE_INPUT_BYTES, FDT_WIRE_ACT_BYTES);
    printf("test_codec: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar.** Run: `make test`. Expected: FAIL, header ausente.

- [ ] **Step 3: Implementar.** `src/codec.c` grava cada `float` por `memcpy` para `uint32_t`, aplica `htole32` manual (deslocamento de bits, sem `endian.h`, para ficar portável), e escreve byte a byte. A decodificação inverte. `FDT_VIEW_PRESENT` é `((const void *)(uintptr_t)1)` — sentinela, nunca desreferenciada. A ordem dos campos no fio é a ordem de declaração dos structs do `model.h`, documentada no header como parte do formato.

- [ ] **Step 4: Rodar e ver passar.** Run: `make test`. Expected: `test_codec: ok`.

- [ ] **Step 5: Commit.** `git commit -m "feat: fixed-width little-endian wire codec for I, B and A"`

---

### Task 2: Envelope com número de sequência

Quita a metade de detecção de **C27**.

**Files:**
- Create: `include/fleet_dt/envelope.h`, `src/envelope.c`
- Test: `tests/test_envelope.c`

**Interfaces:**
- Produces: `FDT_ENV_MAGIC` (`0x46445431`, "FDT1"), `FDT_ENV_HEADER_BYTES` (16); `fdt_env_kind_t` = `{FDT_ENV_INPUT, FDT_ENV_STATE, FDT_ENV_ACT, FDT_ENV_GOAL}`; `fdt_env_t {uint32_t magic; uint16_t vessel; uint8_t kind; uint8_t flags; uint32_t seq; uint32_t payload_len;}`; `fdt_env_encode(const fdt_env_t*, const uint8_t *payload, size_t plen, uint8_t *buf, size_t cap) -> ssize_t`; `fdt_env_decode(const uint8_t *buf, size_t len, fdt_env_t*, const uint8_t **payload_out) -> ssize_t`.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_envelope.c`:

```c
#include <fleet_dt/envelope.h>
#include <fleet_dt/codec.h>
#include <assert.h>
#include <stdio.h>

int main(void)
{
    /* A Tabela I lista 21 entradas e nenhuma é timestamp, então o modelo não
     * carrega tempo. O envelope carrega — e é só por causa dele que a
     * patologia do §V-B ("some packets missed their deadlines... the state of
     * some boats was updated twice within the same simulation frame") é
     * observável. O paper deixa a questão em aberto; aqui ela é contável, não
     * corrigida. */
    assert(FDT_ENV_HEADER_BYTES == 16);

    fdt_state_t b = { .yaw_deg = 42.0f };
    uint8_t payload[FDT_WIRE_STATE_BYTES];
    assert(fdt_enc_state(&b, payload, sizeof payload) == FDT_WIRE_STATE_BYTES);

    fdt_env_t env = { .magic = FDT_ENV_MAGIC, .vessel = 3,
                      .kind = FDT_ENV_STATE, .flags = 0, .seq = 1000,
                      .payload_len = FDT_WIRE_STATE_BYTES };
    uint8_t frame[128];
    ssize_t nw = fdt_env_encode(&env, payload, sizeof payload,
                                frame, sizeof frame);
    assert(nw == FDT_ENV_HEADER_BYTES + FDT_WIRE_STATE_BYTES);
    assert(fdt_env_encode(&env, payload, sizeof payload, frame, 8) == -1);

    fdt_env_t got = {0};
    const uint8_t *pl = NULL;
    assert(fdt_env_decode(frame, (size_t)nw, &got, &pl) == nw);
    assert(got.magic == FDT_ENV_MAGIC && got.vessel == 3);
    assert(got.kind == FDT_ENV_STATE && got.seq == 1000);
    assert(pl != NULL);

    fdt_state_t rt = {0};
    assert(fdt_dec_state(pl, got.payload_len, &rt) == FDT_WIRE_STATE_BYTES);
    assert(rt.yaw_deg == 42.0f);

    /* Frame corrompido e frame truncado são rejeitados, não aceitos meio. */
    frame[0] ^= 0xFF;
    assert(fdt_env_decode(frame, (size_t)nw, &got, &pl) == -1);
    frame[0] ^= 0xFF;
    assert(fdt_env_decode(frame, FDT_ENV_HEADER_BYTES + 4, &got, &pl) == -1);

    printf("test_envelope: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar.** Run: `make test`.

- [ ] **Step 3: Implementar.** Cabeçalho de 16 bytes: magic (4), vessel (2), kind (1), flags (1), seq (4), payload_len (4), tudo little-endian. `fdt_env_decode` valida magic, valida que `len >= FDT_ENV_HEADER_BYTES + payload_len`, e aponta `payload_out` para dentro de `buf` sem copiar.

- [ ] **Step 4: Rodar e ver passar.**

- [ ] **Step 5: Commit.** `git commit -m "feat: wire envelope carrying the sequence number Table I does not"`

---

### Task 3: Detector de frame parcial e de atualização dupla

Quita **C27** por inteiro.

**Files:**
- Create: `include/fleet_dt/framesync.h`, `src/framesync.c`
- Test: `tests/test_framesync.c`

**Interfaces:**
- Produces: `fdt_framesync_t`; `fdt_fs_init(fdt_framesync_t*, uint32_t *seq_slots, uint8_t *hit_slots, size_t n) -> int`; `fdt_fs_begin_frame(fdt_framesync_t*) -> void`; `fdt_fs_accept(fdt_framesync_t*, const fdt_env_t*) -> fdt_fs_verdict_t`; `fdt_fs_end_frame(fdt_framesync_t*) -> void`; contadores `fdt_fs_partial_frames`, `fdt_fs_double_updates`, `fdt_fs_stale_packets`, `fdt_fs_missing_vessels` (todos `-> uint64_t`).
- `fdt_fs_verdict_t` = `{FDT_FS_FRESH, FDT_FS_DUPLICATE_IN_FRAME, FDT_FS_STALE}`.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_framesync.c`:

```c
#include <fleet_dt/framesync.h>
#include <assert.h>
#include <stdio.h>

static fdt_env_t st(uint16_t vessel, uint32_t seq)
{
    return (fdt_env_t){ .magic = FDT_ENV_MAGIC, .vessel = vessel,
                        .kind = FDT_ENV_STATE, .seq = seq, .payload_len = 0 };
}

int main(void)
{
    uint32_t seqs[3];
    uint8_t  hits[3];
    fdt_framesync_t fs;
    assert(fdt_fs_init(&fs, seqs, hits, 3) == 0);

    /* Frame completo: os três vasos chegam uma vez. */
    fdt_fs_begin_frame(&fs);
    for (uint16_t k = 0; k < 3; k++) {
        assert(fdt_fs_accept(&fs, &(fdt_env_t){ .magic = FDT_ENV_MAGIC,
                             .vessel = k, .kind = FDT_ENV_STATE,
                             .seq = 1 }) == FDT_FS_FRESH);
    }
    fdt_fs_end_frame(&fs);
    assert(fdt_fs_partial_frames(&fs) == 0);
    assert(fdt_fs_double_updates(&fs) == 0);

    /* §V-B: "the state of some boats was updated twice within the same
     * simulation frame", porque pacotes atrasados ainda estavam no buffer. */
    fdt_fs_begin_frame(&fs);
    assert(fdt_fs_accept(&fs, &st(0, 2)) == FDT_FS_FRESH);
    assert(fdt_fs_accept(&fs, &st(0, 3)) == FDT_FS_DUPLICATE_IN_FRAME);
    assert(fdt_fs_accept(&fs, &st(1, 2)) == FDT_FS_FRESH);
    /* o vaso 2 não chegou: frame parcial */
    fdt_fs_end_frame(&fs);
    assert(fdt_fs_double_updates(&fs) == 1);
    assert(fdt_fs_partial_frames(&fs) == 1);
    assert(fdt_fs_missing_vessels(&fs) == 1);

    /* Pacote velho: seq menor ou igual ao último aceito daquele vaso. É
     * contado, e o veredito é devolvido ao chamador. Descartá-lo é o item D6
     * do §VI, declarado futuro — esta camada não decide isso. */
    fdt_fs_begin_frame(&fs);
    assert(fdt_fs_accept(&fs, &st(0, 1)) == FDT_FS_STALE);
    fdt_fs_end_frame(&fs);
    assert(fdt_fs_stale_packets(&fs) == 1);

    /* Vaso fora do intervalo é rejeitado sem contaminar contador. */
    assert(fdt_fs_accept(&fs, &st(9, 5)) == FDT_FS_STALE);

    printf("partial=%llu double=%llu stale=%llu\n",
           (unsigned long long)fdt_fs_partial_frames(&fs),
           (unsigned long long)fdt_fs_double_updates(&fs),
           (unsigned long long)fdt_fs_stale_packets(&fs));
    printf("test_framesync: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar.**

- [ ] **Step 3: Implementar.** `begin_frame` zera `hit_slots`. `accept` compara `env->seq` com `seq_slots[vessel]`: menor ou igual é `STALE`; maior com `hit_slots[vessel]` já marcado é `DUPLICATE_IN_FRAME` (conta e ainda assim atualiza `seq_slots`, porque o pacote é novo — o que é patológico é ter chegado dois no mesmo frame); maior com slot livre é `FRESH`. `end_frame` conta um `partial_frame` se algum `hit_slot` ficou zerado, e soma quantos.

- [ ] **Step 4: Rodar e ver passar.**

- [ ] **Step 5: Commit.** `git commit -m "feat: detect and count the partial and double frame updates of Section V-B"`

---

### Task 4: Reguladores de banda

Quita **C5** e **C12** — a contribuição declarada no abstract.

**Files:**
- Create: `include/fleet_dt/regulator.h`, `src/regulator.c`
- Test: `tests/test_regulator.c`

**Interfaces:**
- Produces: `fdt_reg_t`; `fdt_reg_init(fdt_reg_t*, double sensor_hz, double publish_hz) -> int`; `fdt_reg_admit(fdt_reg_t*) -> int` (1 publica, 0 derruba); `fdt_reg_sampled/published/dropped(const fdt_reg_t*) -> uint64_t`; `fdt_reg_effective_hz(const fdt_reg_t*) -> double`; `fdt_reg_saved_ratio(const fdt_reg_t*) -> double`.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_regulator.c`:

```c
#include <fleet_dt/regulator.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    /* §III: "The DTI updates every 125 ms; each update in the real world
     * corresponds to a single increment in k. As some sensors, e.g., a
     * gyroscope at 1 Hz, sample faster than the DT frequency, part of the
     * bandwidth is wasted. The proposed regulators overcome this problem by
     * dropping the number of samples in the MQTT client."
     *
     * Sensor a 100 Hz, DT a 8 Hz: 92 de cada 100 amostras não têm para onde
     * ir. O regulador as derruba no cliente. */
    fdt_reg_t r;
    assert(fdt_reg_init(&r, 100.0, 8.0) == 0);
    assert(fdt_reg_init(&r, 0.0, 8.0) == -1);
    assert(fdt_reg_init(&r, 100.0, 0.0) == -1);

    for (int i = 0; i < 1000; i++) {
        (void)fdt_reg_admit(&r);
    }
    assert(fdt_reg_sampled(&r) == 1000);
    /* 8 Hz de 100 Hz: 80 publicações, com no máximo uma de folga por causa da
     * fase do acumulador. */
    assert(fdt_reg_published(&r) >= 79 && fdt_reg_published(&r) <= 81);
    assert(fdt_reg_published(&r) + fdt_reg_dropped(&r) == 1000);
    assert(fabs(fdt_reg_effective_hz(&r) - 8.0) < 0.2);
    /* "maximizing the use of shared wireless links": 92% do tráfego daquele
     * sensor deixa de existir. */
    assert(fdt_reg_saved_ratio(&r) > 0.90 && fdt_reg_saved_ratio(&r) < 0.93);

    /* Sensor mais lento que o DT nunca é decimado — o regulador não pode
     * inventar amostra nem esconder as que existem. */
    fdt_reg_t slow;
    assert(fdt_reg_init(&slow, 1.0, 8.0) == 0);
    for (int i = 0; i < 50; i++) {
        assert(fdt_reg_admit(&slow) == 1);
    }
    assert(fdt_reg_dropped(&slow) == 0);
    assert(fdt_reg_saved_ratio(&slow) == 0.0);

    /* Taxas iguais: passa tudo. */
    fdt_reg_t same;
    assert(fdt_reg_init(&same, 8.0, 8.0) == 0);
    for (int i = 0; i < 40; i++) {
        assert(fdt_reg_admit(&same) == 1);
    }
    assert(fdt_reg_dropped(&same) == 0);

    /* A segunda metade do §III: "real sensors continue sampling at their own
     * pace, as this is necessary for control algorithms and other
     * applications that neither interact with the DTI nor affect the model."
     * O regulador governa a publicação, não a amostragem: sampled conta toda
     * amostra que o sensor produziu, inclusive as derrubadas. */
    assert(fdt_reg_sampled(&r) == fdt_reg_published(&r) + fdt_reg_dropped(&r));

    printf("100 Hz -> %.2f Hz effective, %.1f%% of traffic saved\n",
           fdt_reg_effective_hz(&r), 100.0 * fdt_reg_saved_ratio(&r));
    printf("test_regulator: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar.**

- [ ] **Step 3: Implementar.** Acumulador de fase, sem ponto flutuante acumulado no laço quente:

```c
int fdt_reg_init(fdt_reg_t *r, double sensor_hz, double publish_hz)
{
    if (r == NULL || sensor_hz <= 0.0 || publish_hz <= 0.0) {
        return -1;
    }
    r->sensor_hz  = sensor_hz;
    r->publish_hz = publish_hz;
    /* Quanto de "crédito de publicação" cada amostra vale. Sensor mais lento
     * que o DT dá crédito >= 1 e nunca é decimado. */
    r->step       = publish_hz / sensor_hz;
    r->acc        = 0.0;
    r->sampled    = 0;
    r->published  = 0;
    r->dropped    = 0;
    return 0;
}

int fdt_reg_admit(fdt_reg_t *r)
{
    if (r == NULL) {
        return 0;
    }
    r->sampled++;
    r->acc += r->step;
    if (r->acc >= 1.0) {
        r->acc -= 1.0;
        if (r->acc >= 1.0) {
            /* Sensor mais lento que o DT: o crédito passa de um, mas uma
             * amostra publica uma vez. O excedente não vira publicação
             * fantasma. */
            r->acc = 0.0;
        }
        r->published++;
        return 1;
    }
    r->dropped++;
    return 0;
}
```

`fdt_reg_saved_ratio` é `dropped / sampled`, com 0.0 quando `sampled` é zero.

- [ ] **Step 4: Rodar e ver passar.**

- [ ] **Step 5: Commit.** `git commit -m "feat: bandwidth regulators decimating publication to the DT rate"`

---

### Task 5: Modelo de link budget

Quita **C4**, **C10** e dá a aritmética de **C20**.

**Files:**
- Create: `include/fleet_dt/linkbudget.h`, `src/linkbudget.c`
- Test: `tests/test_linkbudget.c`

**Interfaces:**
- Produces: `fdt_stream_t {const char *name; size_t payload_bytes; double hz; unsigned vessels;}`; `fdt_link_t {double capacity_bps; double overhead_ratio;}`; `fdt_link_stream_bps(const fdt_stream_t*) -> double`; `fdt_link_total_bps(const fdt_link_t*, const fdt_stream_t*, size_t n) -> double`; `fdt_link_utilization(const fdt_link_t*, const fdt_stream_t*, size_t n) -> double`; `fdt_link_increase(const fdt_link_t*, const fdt_stream_t *baseline, size_t nb, const fdt_stream_t *added, size_t na) -> double`; `fdt_link_max_vessels(const fdt_link_t*, const fdt_stream_t*, size_t n, double budget) -> unsigned`.

**`fdt_link_increase` existe porque o §V-A não afirma o que `utilization`
mede.** A frase é "the bandwidth usage **increased** < 1%" — um delta contra o
tráfego que já existia, não a ocupação absoluta do enlace. Com ocupação
absoluta a conta não fecha: 48 KB × 8 bits × 8 Hz são 3,1 Mbps, ou 3,1% de
100 Mbps. A ambiguidade está registrada como item 5 da seção H do spec; o
teste **imprime as duas leituras e asserta só a aritmética**, nunca o número
do paper.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_linkbudget.c`:

```c
#include <fleet_dt/linkbudget.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    /* §III: "these sensors are attached to the NAVIO2 board... which typically
     * transfer data in small packets (e.g., 8 bytes per IMU axis), resulting
     * in low network overhead (vs. 100 Mbps of available bandwidth)." */
    fdt_link_t wifi = { .capacity_bps = 100e6, .overhead_ratio = 0.0 };

    /* 8 bytes por eixo, 3 eixos, a 8 Hz, um vaso. */
    fdt_stream_t imu = { .name = "imu", .payload_bytes = 8 * 3,
                         .hz = 8.0, .vessels = 1 };
    assert(fabs(fdt_link_stream_bps(&imu) - 24 * 8 * 8.0) < 1e-9);

    /* ARMADILHA do spec, item 1: o payload do §V-A é 48 KB, não os 48 bytes
     * do estado. São grandezas independentes. */
    fdt_stream_t lsdt = { .name = "lsdt", .payload_bytes = 48u * 1024u,
                          .hz = 8.0, .vessels = 1 };
    double bps = fdt_link_stream_bps(&lsdt);
    assert(fabs(bps - 48.0 * 1024.0 * 8.0 * 8.0) < 1e-6);

    /* §V-A: "the bandwidth usage increased < 1% for an update window of
     * 125 ms". Duas leituras possíveis, e o paper não escolhe entre elas.
     *
     * Leitura A, ocupação absoluta: 48 KB * 8 bits * 8 Hz = 3.145 Mbps sobre
     * 100 Mbps = 3.1%. Não fecha com "< 1%".
     * Leitura B, incremento sobre o tráfego que já existia: é o que a palavra
     * "increased" diz, e fecha assim que a linha de base é o próprio enlace
     * em uso.
     *
     * O teste asserta a aritmética das duas e imprime os dois números. Não
     * asserta o "< 1%", porque asserta-lo seria escolher a leitura em nome
     * dos autores. Item 5 da seção H de docs/spec/paper-claims.md. */
    double util = fdt_link_utilization(&wifi, &lsdt, 1);
    printf("reading A, absolute: %.3f%% of a 100 Mbps link\n", 100.0 * util);
    assert(util > 0.031 && util < 0.032);
    assert(fabs(util - bps / wifi.capacity_bps) < 1e-12);

    /* Leitura B: quanto o uso do enlace cresceu em relação ao que já
     * trafegava. "usage increased" é razão contra a linha de base, não contra
     * a capacidade — é por isso que a linha de base é argumento e não
     * constante. Com o vídeo HSDT já no ar, o LSDT do DT é ruído em cima
     * dele. */
    fdt_stream_t baseline = { .name = "existing", .payload_bytes = 1u << 20,
                              .hz = 8.0, .vessels = 1 };
    double inc = fdt_link_increase(&wifi, &baseline, 1, &lsdt, 1);
    printf("reading B, usage increase over baseline: %.3f%%\n", 100.0 * inc);
    assert(fabs(inc - fdt_link_stream_bps(&lsdt) /
                      fdt_link_stream_bps(&baseline)) < 1e-12);

    /* Linha de base vazia não tem incremento relativo definido. */
    assert(fdt_link_increase(&wifi, NULL, 0, &lsdt, 1) < 0.0);

    /* Overhead de cabeçalho: MQTT sobre TCP sobre Wi-Fi. 20% infla o mesmo
     * fluxo proporcionalmente. */
    fdt_link_t withoh = { .capacity_bps = 100e6, .overhead_ratio = 0.20 };
    assert(fabs(fdt_link_utilization(&withoh, &lsdt, 1) - util * 1.2) < 1e-9);

    /* Quantos vasos cabem antes de estourar um orçamento de 50% do enlace. */
    unsigned max = fdt_link_max_vessels(&wifi, &lsdt, 1, 0.50);
    printf("vessels fitting in 50%% of the link: %u\n", max);
    assert(max >= 15 && max <= 16);

    /* Fluxo de zero vasos não consome nada. */
    fdt_stream_t none = { .name = "none", .payload_bytes = 1000,
                          .hz = 8.0, .vessels = 0 };
    assert(fdt_link_stream_bps(&none) == 0.0);

    printf("test_linkbudget: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar.**

- [ ] **Step 3: Implementar.** `fdt_link_stream_bps` é `payload_bytes * 8 * hz * vessels`. `fdt_link_total_bps` soma os fluxos e aplica `(1 + overhead_ratio)`. `fdt_link_utilization` divide pela capacidade. `fdt_link_increase` divide o total acrescentado pelo total da linha de base, e retorna `-1.0` quando a linha de base é zero, porque incremento relativo sobre nada não é número. `fdt_link_max_vessels` faz a divisão inteira do orçamento pelo custo de um vaso, tratando custo zero como "ilimitado" retornando `UINT_MAX`.

- [ ] **Step 4: Rodar e ver passar.**

- [ ] **Step 5: Commit.** `git commit -m "feat: link budget model over LSDT and HSDT stream profiles"`

---

### Task 6: Interface de transporte e loopback

**Files:**
- Create: `include/fleet_dt/transport.h`, `src/transport_loop.c`
- Test: `tests/test_transport.c`

**Interfaces:**
- Produces: `fdt_on_msg_fn`, `fdt_transport_t`, `fdt_loop_t`, `fdt_loop_transport`, `fdt_topic`, e os quatro nomes de tópico.

**Esta é a task da qual tudo depois depende.** A Task 7 deste plano e as Tasks
1, 2, 4, 7 e 8 do Plano C consomem `fdt_transport_t`. Uma assinatura errada
aqui se propaga para dez arquivos.

Tópicos, fixos: `fleet/{vessel}/lsdt` para `Iᵗ`, `fleet/{vessel}/state` para
`Bᵗ`, `fleet/{vessel}/act` para `Aᵗ`, `fleet/{vessel}/goal` para `gᵗ`. Sem
curinga, porque o §III não usa nenhum no caminho de dados — só o bridge usa,
e ali o curinga vive no `mosquitto.conf`, não no código.

- [ ] **Step 1: Escrever o teste que falha**

`tests/test_transport.c`:

```c
#include <fleet_dt/transport.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

typedef struct { unsigned hits; uint8_t last; } sink_t;

static void on_msg(const char *topic, const uint8_t *buf, size_t len,
                   void *user)
{
    (void)topic;
    sink_t *s = (sink_t *)user;
    s->hits++;
    s->last = (len > 0) ? buf[0] : 0;
}

int main(void)
{
    char t[64];
    assert(fdt_topic(t, sizeof t, FDT_TOPIC_STATE, 0) == 0);
    assert(strcmp(t, "fleet/0/state") == 0);
    assert(fdt_topic(t, sizeof t, FDT_TOPIC_LSDT, 12) == 0);
    assert(strcmp(t, "fleet/12/lsdt") == 0);
    assert(fdt_topic(t, 4, FDT_TOPIC_STATE, 0) == -1);   /* buffer curto */
    assert(fdt_topic(NULL, sizeof t, FDT_TOPIC_STATE, 0) == -1);

    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    assert(tr.self != NULL && tr.publish != NULL && tr.subscribe != NULL);

    sink_t s0 = {0}, s1 = {0};
    assert(tr.subscribe(tr.self, "fleet/0/state", on_msg, &s0) == 0);
    assert(tr.subscribe(tr.self, "fleet/1/state", on_msg, &s1) == 0);

    /* Publicar não entrega: poll entrega. É assim que o laço de frame
     * controla quando a entrega acontece, que é o que torna a patologia do
     * §V-B — pacote no buffer de rede atravessando a fronteira do frame —
     * reproduzível em teste. */
    uint8_t payload[1] = { 0xAB };
    assert(tr.publish(tr.self, "fleet/0/state", payload, 1, 1) == 0);
    assert(s0.hits == 0);

    assert(tr.poll(tr.self, 0) == 1);          /* uma mensagem entregue */
    assert(s0.hits == 1 && s0.last == 0xAB);
    assert(s1.hits == 0);                      /* tópico exato, sem curinga */

    /* Fila drenada: o poll seguinte não reentrega. */
    assert(tr.poll(tr.self, 0) == 0);
    assert(s0.hits == 1);

    /* Vários assinantes do mesmo tópico recebem todos. */
    sink_t s0b = {0};
    assert(tr.subscribe(tr.self, "fleet/0/state", on_msg, &s0b) == 0);
    assert(tr.publish(tr.self, "fleet/0/state", payload, 1, 1) == 0);
    assert(tr.poll(tr.self, 0) == 1);
    assert(s0.hits == 2 && s0b.hits == 1);

    /* Tópico sem assinante é publicável e simplesmente não entrega. */
    assert(tr.publish(tr.self, "fleet/9/act", payload, 1, 1) == 0);
    assert(tr.poll(tr.self, 0) == 1);

    /* Fila cheia rejeita em vez de sobrescrever silenciosamente. */
    for (size_t i = 0; i < FDT_LOOP_QUEUE; i++) {
        assert(tr.publish(tr.self, "fleet/0/state", payload, 1, 1) == 0);
    }
    assert(tr.publish(tr.self, "fleet/0/state", payload, 1, 1) == -1);

    tr.close(tr.self);
    printf("test_transport: ok\n");
    return 0;
}
```

- [ ] **Step 2: Rodar e ver falhar.** Run: `make test`.

- [ ] **Step 3: Escrever `include/fleet_dt/transport.h`**

```c
#ifndef FLEET_DT_TRANSPORT_H
#define FLEET_DT_TRANSPORT_H

#include <stddef.h>
#include <stdint.h>

/* Os quatro tópicos do caminho de dados do §III. Sem curinga: o curinga do
 * bridge vive no mosquitto.conf, não aqui. */
#define FDT_TOPIC_LSDT  "lsdt"   /* I^t, telemetria de baixa taxa    */
#define FDT_TOPIC_STATE "state"  /* B^t, estado do gêmeo             */
#define FDT_TOPIC_ACT   "act"    /* A^t, atuação de volta ao barco   */
#define FDT_TOPIC_GOAL  "goal"   /* g^t, objetivo vindo do MCS       */

/* Escreve "fleet/{vessel}/{kind}" em buf. 0 em sucesso, -1 com buf NULL,
 * kind NULL, ou capacidade insuficiente. */
int fdt_topic(char *buf, size_t cap, const char *kind, unsigned vessel);

/* Entregue por poll, nunca por publish. */
typedef void (*fdt_on_msg_fn)(const char *topic, const uint8_t *buf,
                              size_t len, void *user);

/* A fronteira de rede desta biblioteca. Cinco membros e nada mais: o núcleo
 * do modelo não sabe o que é um broker, e o adaptador MQTT não sabe o que é
 * uma equação.
 *
 * publish    enfileira para envio; 0 em sucesso, -1 se a fila está cheia
 * subscribe  registra cb para um tópico exato; 0 em sucesso, -1 sem espaço
 * poll       entrega o que chegou e retorna quantas mensagens entregou, ou
 *            -1 em erro. timeout_ms é dica; o loopback o ignora.
 * close      libera o que a implementação alocou
 * self       estado da implementação, opaco ao chamador */
typedef struct {
    int  (*publish)(void *self, const char *topic, const uint8_t *buf,
                    size_t len, int qos);
    int  (*subscribe)(void *self, const char *topic, fdt_on_msg_fn cb,
                      void *user);
    int  (*poll)(void *self, int timeout_ms);
    void (*close)(void *self);
    void *self;
} fdt_transport_t;

#define FDT_LOOP_QUEUE   64   /* mensagens em voo                */
#define FDT_LOOP_SUBS    32   /* assinaturas simultâneas         */
#define FDT_LOOP_TOPIC   64   /* comprimento máximo de tópico    */
#define FDT_LOOP_PAYLOAD 256  /* payload máximo do loopback      */

/* Transporte em processo, sem rede: publish enfileira, poll entrega. Separar
 * os dois é o que deixa um teste posicionar a entrega de um pacote atrasado
 * do lado errado da fronteira do frame, que é a patologia do §V-B. */
typedef struct {
    struct {
        char    topic[FDT_LOOP_TOPIC];
        uint8_t payload[FDT_LOOP_PAYLOAD];
        size_t  len;
    } q[FDT_LOOP_QUEUE];
    size_t qlen;

    struct {
        char          topic[FDT_LOOP_TOPIC];
        fdt_on_msg_fn cb;
        void         *user;
    } subs[FDT_LOOP_SUBS];
    size_t nsubs;
} fdt_loop_t;

/* Zera lo e devolve um fdt_transport_t apontando para ele. lo é do chamador e
 * precisa sobreviver ao transporte. */
fdt_transport_t fdt_loop_transport(fdt_loop_t *lo);

#endif /* FLEET_DT_TRANSPORT_H */
```

- [ ] **Step 4: Escrever `src/transport_loop.c`.** `fdt_topic` usa `snprintf` e checa o retorno contra `cap`. `loop_publish` recusa payload maior que `FDT_LOOP_PAYLOAD` e fila cheia, ambos com -1. `loop_poll` percorre a fila da frente para trás, e para cada mensagem chama todo assinante cujo tópico casa por `strcmp`, retorna o número de **mensagens** entregues (não de callbacks disparados), e zera `qlen` ao fim. `loop_close` zera a struct.

- [ ] **Step 5: Rodar e ver passar.** Run: `make test`. Expected: `test_transport: ok`.

- [ ] **Step 6: Commit.** `git commit -m "feat: transport interface with an in-process loopback implementation"`

**Nota de ordenação para quem executar:** esta task define `fdt_topic`, que a
Task 1 do Plano C usa. Ela não é usada pelas Tasks 1 a 5 deste plano — codec,
envelope, framesync, regulador e link budget não tocam em transporte. A ordem
dentro do Plano B pode ser 1–5 e depois 6–7, ou 6 primeiro; o que não pode é
começar o Plano C sem esta task fechada.

---

### Task 7: Adaptador MQTT e brokers em bridge

Quita **C2** e **C9**.

**Files:**
- Create: `adapters/mqtt/fdt_mqtt.h`, `adapters/mqtt/fdt_mqtt.c`
- Create: `config/mosquitto/boat.conf`, `config/mosquitto/ground.conf`, `config/mosquitto/README.md`
- Create: `tests/it_mqtt_bridge.sh`
- Modify: `Makefile` (alvo `mqtt`, alvo `it`)

**Interfaces:**
- Consumes: `fdt_transport_t`.
- Produces: `fdt_mqtt_open(const char *host, int port, const char *client_id, fdt_transport_t *out) -> int`; `fdt_mqtt_close(fdt_transport_t*) -> void`.

- [ ] **Step 1: Escrever a config de bridge**

`config/mosquitto/boat.conf` — broker local do barco. §III: "local MQTT brokers
are connected in bridge mode to avoid service interruption during temporary
connection instability".

```
listener 1883
allow_anonymous true
persistence true
persistence_location /var/lib/mosquitto/

# Bridge para o broker da estação de solo. A fila local sobrevive à queda do
# enlace; quando o Wi-Fi volta, o bridge reconecta e drena.
connection ground
address ground.local:1883
topic fleet/# out 1 "" ""
topic fleet/+/goal in 1 "" ""
bridge_protocol_version mqttv50
cleansession false
notifications true
try_private true
restart_timeout 5
```

`config/mosquitto/ground.conf` — broker da estação, sem bridge de saída:

```
listener 1883
allow_anonymous true
persistence true
persistence_location /var/lib/mosquitto/
```

`config/mosquitto/README.md` explica por que `cleansession false` e
`persistence true` são o que faz o bridge sobreviver à instabilidade que o §III
cita, e por que `topic fleet/# out` e `fleet/+/goal in` são direções opostas: a
telemetria sobe, o objetivo desce.

- [ ] **Step 2: Escrever o teste de integração**

`tests/it_mqtt_bridge.sh`: sobe dois `mosquitto` com as duas configs em portas
distintas, publica 10 estados no broker do barco, **derruba** o broker da
estação por 2 s, publica mais 10, sobe a estação de volta, e verifica que os 20
chegaram. `set -euo pipefail`; pula com código 77 (skip) se `mosquitto` não
estiver instalado.

- [ ] **Step 3: Implementar o adaptador.** `fdt_mqtt_open` faz `mosquitto_lib_init`, `mosquitto_new`, `mosquitto_connect`, e preenche `fdt_transport_t` com funções que delegam a `mosquitto_publish`/`mosquitto_subscribe`/`mosquitto_loop`. QoS 1 por padrão, porque o bridge é `out 1`.

- [ ] **Step 4: Estender o Makefile**

```make
MOSQ_CFLAGS = $(shell pkg-config --cflags libmosquitto 2>/dev/null)
MOSQ_LIBS   = $(shell pkg-config --libs libmosquitto 2>/dev/null)

.PHONY: mqtt it

mqtt: $(LIB)
	@test -n "$(MOSQ_LIBS)" || { echo "libmosquitto not found; skipping"; exit 0; }
	$(CC) $(CFLAGS) $(MOSQ_CFLAGS) -c adapters/mqtt/fdt_mqtt.c -o adapters/mqtt/fdt_mqtt.o
	ar rcs libfleetdt_mqtt.a adapters/mqtt/fdt_mqtt.o

it:
	./tests/it_mqtt_bridge.sh
```

- [ ] **Step 5: Rodar.** Run: `make mqtt && make it`. Expected: PASS, ou skip limpo sem mosquitto.

- [ ] **Step 6: Commit.** `git commit -m "feat: MQTT transport adapter and bridge-mode broker configuration"`

---

## Self-Review

**Cobertura.** C2 Task 7 · C4 Task 5 · C5 Task 4 · C9 Task 7 · C10 Task 5 ·
C12 Task 4 · C20 Task 5 (aritmética; a medição é o Plano C) · C27 Tasks 2 e 3.

**Armadilhas endereçadas.** Item 1 (48 B vs 48 KB) tem asserção explícita no
`test_linkbudget`. Item 4 (timestamp no envelope, não no modelo) é a Task 2, e
a Task 3 conta sem corrigir, respeitando D6.

**Consistência.** `fdt_env_t.seq` é `uint32_t` em todas as tasks;
`fdt_fs_accept` recebe `const fdt_env_t*`, o mesmo tipo que `fdt_env_decode`
preenche. Nada nesta camada altera assinatura do Plano A.
