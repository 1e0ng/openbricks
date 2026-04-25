// SPDX-License-Identifier: MIT
//
// Two-state α-β observer (position, velocity). Same algorithm and
// numerics in firmware + sim — the byte-identical compile is the
// whole point of splitting it out from the binding shells.
//
// Algorithm (per tick):
//
//     predict:  p̂' = p̂ + v̂ · dt
//               v̂' = v̂
//     innovate: r  = y - p̂'
//     update:   p̂  = p̂' + α · r
//               v̂  = v̂' + (β / dt) · r
//
// With α = 0.5 and β = 0.15 (the defaults the wrappers pass in) the
// filter is close to critically damped: tracks step changes in
// position within a few samples and attenuates high-frequency noise
// by ~10x.

#include "observer_core.h"

void ob_observer_init(ob_observer_t *o, ob_float_t alpha, ob_float_t beta) {
    o->alpha   = alpha;
    o->beta    = beta;
    o->pos_hat = 0.0;
    o->vel_hat = 0.0;
}

void ob_observer_reset(ob_observer_t *o, ob_float_t pos) {
    o->pos_hat = pos;
    o->vel_hat = 0.0;
}

void ob_observer_update(ob_observer_t *o, ob_float_t measured_pos, ob_float_t dt) {
    if (dt <= 0.0) {
        return;
    }
    ob_float_t pos_pred = o->pos_hat + o->vel_hat * dt;
    ob_float_t residual = measured_pos - pos_pred;
    o->pos_hat = pos_pred   + o->alpha * residual;
    o->vel_hat = o->vel_hat + (o->beta / dt) * residual;
}
