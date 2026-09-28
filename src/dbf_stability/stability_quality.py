"""Describe the computed spectrum without equating small roots with kinematic modes."""

import numpy as np
import pandas as pd


def describe_spectrum(eigenvalues, tolerance=1e-5):
    eig = np.asarray(eigenvalues, dtype=complex)
    if eig.ndim != 1 or not eig.size or not np.isfinite(eig).all():
        raise ValueError('Stability spectrum must be a nonempty finite vector')
    if not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError('Stability tolerance must be finite and positive')
    rows = []
    for value in sorted(eig, key=lambda v: v.real, reverse=True):
        classification = (
            'unstable' if value.real > tolerance else 'stable' if value.real < -tolerance else 'near_neutral'
        )
        rows.append(
            dict(
                real_1_s=float(value.real),
                imag_rad_s=float(value.imag),
                frequency_hz=float(abs(value.imag) / (2 * np.pi)),
                damping_ratio=float(-value.real / abs(value)) if abs(value) > 1e-10 else None,
                classification=classification,
            )
        )
    unstable = any(r['classification'] == 'unstable' for r in rows)
    near_neutral = sum(r['classification'] == 'near_neutral' for r in rows)
    maximum = float(eig.real.max())
    return dict(
        modes=pd.DataFrame(rows),
        unstable=unstable,
        stability_status='unstable_detected' if unstable else 'near_neutral' if near_neutral else 'damped',
        max_real_eigenvalue_1_s=maximum,
        growth_time_s=1 / maximum if unstable else None,
        near_neutral_modes=near_neutral,
        eigenvalue_tolerance_1_s=float(tolerance),
        linearization_verified=False,
        note='Local fixed-length linearization; near-zero real parts are unresolved, '
        'not identified kinematic modes. No deployment, capture or flight-safety verdict.',
    )
