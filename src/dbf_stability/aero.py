"""Aerodynamic table construction, dispatched on aero.backend."""

from .avl import build_avl_database
from .flow5 import build_flow5_database
from .hybrid import build_hybrid_database

BUILDERS = dict(avl=build_avl_database, flow5=build_flow5_database, hybrid=build_hybrid_database)


def build_aero_database(config, output=None, workers=None):
    backend = config['aero'].get('backend', 'avl').lower()
    if backend not in BUILDERS:
        raise ValueError(f'Unknown aerodynamic backend: {backend}')
    return BUILDERS[backend](config, output, workers)
