#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""CPU-only check of the actual REGDMA teardown functions (cc required).

Run from any directory. Hardware acknowledgement, delays and resource release
are instrumented boundaries, not an emulation of REGDMA or the outer KMS unbind.
The never-acknowledged case also has a host timeout to catch unbounded loops.
"""
from pathlib import Path
import os
import subprocess
import tempfile


def extract(source, signature):
    start = source.index(signature)
    pos = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[start:pos]


root = Path(__file__).resolve().parents[5]
source = (root / 'drivers/gpu/drm/msm/disp/dpu1/dpu_lutdma.c').read_text()
fixture = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
struct dpu_kms { void *dev; };
struct dpu_lutdma {
    struct dpu_kms *kms;
    bool faulted, inflight, leaked;
    unsigned int active_ctl;
    int lock;
    void *bo, *vm, *mmio;
    unsigned long start, size;
};
static unsigned int resets, elapsed_ms, retires, frees, logs, ack_at;
#define WRITE_ONCE(p,v) ((p) = (v))
static void mutex_lock(int *lock) { assert(!*lock); *lock = 1; }
static void mutex_unlock(int *lock) { assert(*lock); *lock = 0; }
static int queue_reset(struct dpu_lutdma *dma, unsigned int ctl) {
    assert(dma->lock && ctl == dma->active_ctl);
    resets++;
    if (ack_at && resets >= ack_at) return 0;
    elapsed_ms += 2; /* existing bounded 2 ms poll */
    return -1;
}
static void retire_reference(struct dpu_lutdma *dma) {
    assert(dma->lock); retires++; dma->inflight = false;
}
static void msleep(unsigned int ms) { assert(ms == 100); elapsed_ms += ms; }
#define drm_err(...) (logs++)
#define msm_gem_kernel_put(...) (frees++)
#define drm_gpuvm_put(...) (frees++)
#define iounmap(...) (frees++)
#define release_mem_region(...) (frees++)
#define kfree(...) (frees++)
'''
fixture += extract(source, 'void dpu_lutdma_quiesce(') + '\n'
fixture += extract(source, 'void dpu_lutdma_destroy(') + '\n'
fixture += r'''
int main(void) {
    struct dpu_kms kms = {0};
    struct dpu_lutdma dma = {.kms = &kms, .active_ctl = 1};
    dpu_lutdma_quiesce(NULL); dpu_lutdma_destroy(NULL);
    dpu_lutdma_destroy(&dma);
    assert(frees == 5 && !resets && !retires && !logs);
    frees = 0; dma.inflight = true; ack_at = 3;
    dpu_lutdma_destroy(&dma);
    assert(resets == 3 && retires == 1 && frees == 5 && !logs);
    assert(!dma.inflight && !dma.leaked && elapsed_ms == 204);
    resets = retires = frees = elapsed_ms = ack_at = 0;
    dma.inflight = true;
    dpu_lutdma_quiesce(&dma);
    assert(dma.faulted && dma.leaked && dma.inflight);
    assert(resets == 20 && elapsed_ms == 2040 && logs == 1);
    assert(!retires && !frees && !dma.lock);
    /* Outer owners may already have gone away. Never dereference KMS again. */
    dma.kms = NULL;
    for (int i = 0; i < 3; i++) {
        dpu_lutdma_quiesce(&dma); dpu_lutdma_destroy(&dma);
    }
    assert(resets == 20 && elapsed_ms == 2040 && logs == 1);
    assert(!retires && !frees && !dma.lock);
    puts("PASS: idle, acknowledged reset, permanently unacknowledged reset; "
         "bounded return, retained resources, no subsequent reset/release/log");
}
'''
with tempfile.TemporaryDirectory(prefix='dpu-lutdma-quiesce-') as tmp:
    path = Path(tmp)
    (path / 'fixture.c').write_text(fixture)
    subprocess.run([os.environ.get('CC', 'cc'), '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                    str(path / 'fixture.c'), '-o', str(path / 'fixture')], check=True)
    subprocess.run([str(path / 'fixture')], check=True, timeout=5)
