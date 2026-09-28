import numpy as np
import pytest
from scipy.integrate import solve_ivp
from dbf_stability.math3d import rotation, quaternion, qdot, spring_force, contact_force
from dbf_stability.config import changed


def test_frames_and_pitch_sign():
    r = rotation(quaternion(pitch=np.pi / 2))
    assert np.allclose(r @ np.array([1, 0, 0]), [0, 0, -1], atol=1e-12)
    assert np.allclose(np.cross([1, 0, 0], [0, 0, -1]), [0, 1, 0])


def test_quaternion_kinematics():
    w = np.array([0.1, -0.2, 0.3])
    q = quaternion(0.3, 0.4, 0.2)
    dt = 1e-6
    actual = (rotation(q + qdot(q, w) * dt) - rotation(q)) / dt
    from dbf_stability.math3d import skew

    assert np.allclose(actual, rotation(q) @ skew(w), atol=1e-6)


def test_tension_only_cable_static():
    f, T = spring_force(np.array([1.1, 0, 0]), np.zeros(3), 1, 100, 1)
    assert T == pytest.approx(10)
    assert np.allclose(f, [10, 0, 0])
    assert spring_force(np.array([0.8, 0, 0]), np.array([0, 0, 0]), 1, 100, 1)[1] == 0


def test_damped_oscillator_against_solution():
    mass, k, c = 1.0, 9.0, 0.8

    def rhs(t, y):
        f, _ = spring_force(np.array([y[0], 0, 0]), np.array([y[1], 0, 0]), 1, k, c, False)
        return [y[1], -f[0] / mass]

    t = np.linspace(0, 4, 101)
    sol = solve_ivp(rhs, [0, 4], [1.1, 0], t_eval=t, method='Radau', rtol=1e-9, atol=1e-11)
    a = c / (2 * mass)
    wd = np.sqrt(k / mass - a * a)
    exact = 1 + 0.1 * np.exp(-a * t) * (np.cos(wd * t) + a / wd * np.sin(wd * t))
    assert np.max(abs(sol.y[0] - exact)) < 1e-8


def test_friction_and_damping_do_not_add_energy():
    normal, tangent = contact_force(0.01, 0.2, [1, 0, 0], 100, 1, 0.2)
    assert normal == pytest.approx(0.8)
    assert tangent[0] < 0
    assert contact_force(-0.01, -100, [0, 0, 0], 100, 1, 0.2)[0] == 0


def test_elastic_collision_peak_impulse():
    # Point mass meeting a frictionless elastic wall, analytic half-period.
    mass, k, v = 2.0, 200.0, 0.5
    period = np.pi * np.sqrt(mass / k)

    def rhs(t, y):
        f, _ = contact_force(max(0.0, -y[0]), y[1], [0, 0, 0], k, 0, 0)
        return [y[1], f / mass, f]

    sol = solve_ivp(rhs, [0, period], [0, -v, 0], method='Radau', rtol=1e-9, atol=1e-11, dense_output=True)
    assert sol.y[1, -1] == pytest.approx(v, rel=1e-6)
    assert sol.y[2, -1] == pytest.approx(2 * mass * v, rel=1e-6)
    assert -sol.sol(period / 2)[0] * k == pytest.approx(v * np.sqrt(k * mass), rel=1e-6)


def test_reject_missing_drag_and_invalid_mass(cfg):
    with pytest.raises(ValueError):
        changed(cfg, **{'sensor.mass_kg': -1})
    with pytest.raises(ValueError):
        changed(cfg, **{'cable.EA_N': 0})


def test_finite_door_does_not_push_distant_backside():
    from dbf_stability.math3d import sphere_box_contact

    half = [0.1, 0.1, 0.003]
    normal, penetration = sphere_box_contact([0, 0, -0.16], np.zeros(3), np.eye(3), half, 0.006)
    assert penetration < 0
    normal, penetration = sphere_box_contact([0, 0, -0.005], np.zeros(3), np.eye(3), half, 0.006)
    assert penetration == pytest.approx(0.004)
    assert np.allclose(normal, [0, 0, -1])


def test_small_angle_pendulum_and_elastic_energy():
    mass, length, g, ea = 0.1, 2.0, 9.80665, 10000.0
    initial_angle = 0.01
    static_length = length * (1 + mass * g / ea)
    initial = np.r_[static_length * np.array([np.sin(initial_angle), np.cos(initial_angle)]), [0.0, 0.0]]

    def rhs(t, y):
        force, _ = spring_force(np.r_[y[:2], 0.0], np.r_[y[2:], 0.0], length, ea, 0.0)
        return np.r_[y[2:], np.array([0, g]) - force[:2] / mass]

    period = 2 * np.pi * np.sqrt(static_length / g)
    sol = solve_ivp(rhs, [0, period], initial, method='Radau', rtol=1e-9, atol=1e-11, dense_output=True)
    mid = sol.sol(period / 2)
    assert np.arctan2(mid[0], mid[1]) == pytest.approx(-initial_angle, rel=3e-4)
    r = np.linalg.norm(sol.y[:2], axis=0)
    energy = (
        0.5 * mass * (sol.y[2:] ** 2).sum(axis=0)
        - mass * g * sol.y[1]
        + 0.5 * ea / length * np.maximum(0, r - length) ** 2
    )
    assert np.ptp(energy) < 1e-7
