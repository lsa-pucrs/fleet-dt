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
- Produces: `FDT_WIRE_STATE_BYTES` (48), `FDT_WIRE_INPUT_BYTES` (76), `FDT_WIRE_ACT_BYTES` (8); `fdt_enc_state(const fdt_state_t*, uint8_t *buf, size_t cap) -> ssize_t`; `fdt_dec_state(const uint8_t *buf, size_t len, fdt_state_t*) -> ssize_t`; e os pares `fdt_enc_input`/`fdt_dec_input`, `fdt_enc_act`/`fdt_dec_act`. Cada um retorna bytes consumidos ou `-1`.

`FDT_WIRE_INPUT_BYTES` é 76 = 19 escalares × 4 bytes. As duas vistas de câmera
da Tabela I **não vão no fio**: são ponteiros opacos, e o §III as tira do MQTT
para o RTSP (C11). O codec grava um byte de flag por vista, indicando presença,
e o receptor as reamarra pelo caminho HSDT — daí 76 + 2 = 78 no total. Fixe
`FDT_WIRE_INPUT_BYTES` em **78** e documente a decomposição no header.

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

    /* Little-endian explícito: o primeiro float é lat_deg e o formato não
     * depende do endianness do host. */
    float back;
    memcpy(&back, buf, 4);
    (void)back;  /* só vale em host LE; a checagem real é o round-trip */

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
- Produces: `fdt_stream_t {const char *name; size_t payload_bytes; double hz; unsigned vessels;}`; `fdt_link_t {double capacity_bps; double overhead_ratio;}`; `fdt_link_stream_bps(const fdt_stream_t*) -> double`; `fdt_link_total_bps(const fdt_link_t*, const fdt_stream_t*, size_t n) -> double`; `fdt_link_utilization(const fdt_link_t*, const fdt_stream_t*, size_t n) -> double`; `fdt_link_max_vessels(const fdt_link_t*, const fdt_stream_t*, size_t n, double budget) -> unsigned`.

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
     * 125 ms". 48 KB * 8 bits * 8 Hz = 3.145 Mbps sobre 100 Mbps são 3.1%, o
     * que só fecha em < 1% se a capacidade do enlace for maior que a nominal
     * de 100 Mbps, ou se o payload já vier regulado. O teste registra o
     * número, não o força. */
    double util = fdt_link_utilization(&wifi, &lsdt, 1);
    printf("48 KB at 8 Hz over 100 Mbps: %.3f%% utilization\n", 100.0 * util);
    assert(util > 0.03 && util < 0.032);

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

- [ ] **Step 3: Implementar.** `fdt_link_stream_bps` é `payload_bytes * 8 * hz * vessels`. `fdt_link_total_bps` soma os fluxos e aplica `(1 + overhead_ratio)`. `fdt_link_utilization` divide pela capacidade. `fdt_link_max_vessels` faz a divisão inteira do orçamento pelo custo de um vaso, tratando custo zero como "ilimitado" retornando `UINT_MAX`.

- [ ] **Step 4: Rodar e ver passar.**

- [ ] **Step 5: Commit.** `git commit -m "feat: link budget model over LSDT and HSDT stream profiles"`

---

### Task 6: Interface de transporte e loopback

**Files:**
- Create: `include/fleet_dt/transport.h`, `src/transport_loop.c`
- Test: `tests/test_transport.c`

**Interfaces:**
- Produces: `fdt_transport_t` com `publish(void *self, const char *topic, const uint8_t *buf, size_t len, int qos)`, `subscribe(void *self, const char *topic, fdt_on_msg_fn cb, void *user)`, `poll(void *self, int timeout_ms)`, `close(void *self)`, e `void *self`; `typedef void (*fdt_on_msg_fn)(const char *topic, const uint8_t *buf, size_t len, void *user)`; `fdt_loop_transport(fdt_loop_t*) -> fdt_transport_t` — implementação em processo, sem rede, para teste.
- Tópicos, fixados como constantes: `fleet/{vessel}/lsdt` para `Iᵗ`, `fleet/{vessel}/state` para `Bᵗ`, `fleet/{vessel}/act` para `Aᵗ`, `fleet/{vessel}/goal` para `gᵗ`. Helper `fdt_topic(char *buf, size_t cap, const char *kind, unsigned vessel) -> int`.

- [ ] **Step 1–5:** Teste que publica em `fleet/0/state`, verifica que o assinante de `fleet/0/state` recebe e o de `fleet/1/state` não, que `poll` drena a fila e retorna a contagem entregue, e que `fdt_topic` recusa buffer curto. Implementação com fila estática de mensagens e casamento exato de tópico (sem curinga, porque o §III não usa nenhum). Commit: `feat: transport interface with an in-process loopback implementation`.

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
